
package com.developer27.xamera

import android.Manifest
import android.annotation.SuppressLint
import android.content.Intent
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.ColorMatrix
import android.graphics.ColorMatrixColorFilter
import android.graphics.Paint
import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraManager
import android.os.Bundle
import android.os.Environment
import android.preference.PreferenceManager
import android.util.Log
import android.util.SparseIntArray
import android.view.Surface
import android.view.TextureView
import android.view.View
import android.widget.Toast
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.developer27.xamera.camera.CameraHelper
import com.developer27.xamera.databinding.ActivityMainBinding
import com.developer27.xamera.videoprocessing.ProcessedFrameRecorder
import com.developer27.xamera.videoprocessing.ProcessedVideoRecorder
import com.developer27.xamera.videoprocessing.Settings
import com.developer27.xamera.videoprocessing.VideoProcessor
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.GpuDelegate
import org.tensorflow.lite.nnapi.NnApiDelegate
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel

/**
 * MainActivity:
 * - Sets up the camera preview and UI controls.
 * - Processes frames via VideoProcessor (which applies OpenCV overlays such as drawn traces).
 * - Records the processed frames to a video file.
 * - When the user stops tracking, the app first saves the tracking (line) data file
 *   (which is saved in Documents/tracking) and then automatically saves the Letter Inference Data
 *   in Documents/2d_letter with a timestamp.
 */
class MainActivity : AppCompatActivity() {
    private lateinit var viewBinding: ActivityMainBinding
    private lateinit var sharedPreferences: SharedPreferences
    private lateinit var cameraManager: CameraManager
    private lateinit var cameraHelper: CameraHelper
    // Global TFLite Interpreter instance.
    private var tfliteInterpreter: Interpreter? = null

    // Custom recorder to save processed video frames.
    private var processedVideoRecorder: ProcessedVideoRecorder? = null
    private var processedFrameRecorder: ProcessedFrameRecorder? = null

    // VideoProcessor applies detection and drawing overlays.
    private var videoProcessor: VideoProcessor? = null

    // Flags to control the processing/recording states.
    private var isRecording = false
    private var isProcessing = false
    private var isProcessingFrame = false

    // This variable is for inference result.
    private var inferenceResult = ""

    // New: Stores the tracking coordinates received from VideoProcessor.
    private var trackingCoordinates: String = ""

    // Declare these as member variables or inside onCreate before setting the listener.
    var isLetterSelected = true
    var isDigitSelected = !isLetterSelected  // Inverse of letter selection

    // Permissions required by the app.
    private val REQUIRED_PERMISSIONS = arrayOf(
        Manifest.permission.CAMERA,
        Manifest.permission.RECORD_AUDIO
    )
    private lateinit var requestPermissionLauncher: ActivityResultLauncher<Array<String>>

    companion object {
        private const val SETTINGS_REQUEST_CODE = 1
        // Mapping for camera rotation.
        private val ORIENTATIONS = SparseIntArray().apply {
            append(Surface.ROTATION_0, 90)
            append(Surface.ROTATION_90, 0)
            append(Surface.ROTATION_180, 270)
            append(Surface.ROTATION_270, 180)
        }
    }

    // TextureView listener for the camera preview.
    private val textureListener = object : TextureView.SurfaceTextureListener {
        @SuppressLint("MissingPermission")
        override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
            if (allPermissionsGranted()) {
                cameraHelper.openCamera()
            } else {
                requestPermissionLauncher.launch(REQUIRED_PERMISSIONS)
            }
        }
        override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) {}
        override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean = false
        override fun onSurfaceTextureUpdated(surface: SurfaceTexture) {
            if (isProcessing) {
                processFrameWithVideoProcessor()
            }
        }
    }

    @SuppressLint("MissingPermission")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        viewBinding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(viewBinding.root)

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(this)
        cameraManager = getSystemService(CAMERA_SERVICE) as CameraManager

        cameraHelper = CameraHelper(this, viewBinding, sharedPreferences)
        videoProcessor = VideoProcessor(this)

        viewBinding.processedFrameView.visibility = View.GONE

        requestPermissionLauncher =
            registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
                val camGranted = permissions[Manifest.permission.CAMERA] ?: false
                val micGranted = permissions[Manifest.permission.RECORD_AUDIO] ?: false
                if (camGranted && micGranted) {
                    if (viewBinding.viewFinder.isAvailable) {
                        cameraHelper.openCamera()
                    } else {
                        viewBinding.viewFinder.surfaceTextureListener = textureListener
                    }
                } else {
                    Toast.makeText(this, "Camera & Audio permissions are required.", Toast.LENGTH_SHORT).show()
                }
            }

        if (allPermissionsGranted()) {
            if (viewBinding.viewFinder.isAvailable) {
                cameraHelper.openCamera()
            } else {
                viewBinding.viewFinder.surfaceTextureListener = textureListener
            }
        } else {
            requestPermissionLauncher.launch(REQUIRED_PERMISSIONS)
        }

        viewBinding.startProcessingButton.setOnClickListener {
            if (isRecording) {
                stopProcessingAndRecording()
            } else {
                startProcessingAndRecording()
            }
        }
        viewBinding.switchCameraButton.setOnClickListener { switchCamera() }
        viewBinding.aboutButton.setOnClickListener {
            startActivity(Intent(this, AboutXameraActivity::class.java))
        }
        viewBinding.settingsButton.setOnClickListener {
            startActivity(Intent(this, SettingsActivity::class.java))
        }

        // Set up the single RadioButton (radioToggle) as a toggle control.
        // Initially set to "Digit".
        viewBinding.radioToggle.setTextColor(Color.WHITE)
        viewBinding.radioToggle.buttonTintList = ContextCompat.getColorStateList(this, android.R.color.white)
        viewBinding.radioToggle.text = "Digit"
        viewBinding.radioToggle.isChecked = true

        // Initialize the RadioButton: When checked (isLetterSelected true) it should display "Letter".
        viewBinding.radioToggle.text = if (isLetterSelected) "Letter" else "Digit"
        viewBinding.radioToggle.isChecked = isLetterSelected

        viewBinding.radioToggle.setOnClickListener {
            // Toggle the letter selection state.
            isLetterSelected = !isLetterSelected

            // Update the digit selection state as its inverse.
            isDigitSelected = !isLetterSelected

            // Update the RadioButton's text and checked state.
            viewBinding.radioToggle.text = if (isLetterSelected) "Letter" else "Digit"
            viewBinding.radioToggle.isChecked = isLetterSelected
        }

        // Loads in both the ML models.
        loadTFLiteModelOnStartupThreaded("YOLOv3_float32.tflite")
        loadTFLiteModelOnStartupThreaded("DigitRecog_float32.tflite")

        cameraHelper.setupZoomControls()
        sharedPreferences.registerOnSharedPreferenceChangeListener { _, key ->
            if (key == "shutter_speed") {
                cameraHelper.updateShutterSpeed()
            }
        }
    }

    private fun startProcessingAndRecording() {
        isRecording = true
        isProcessing = true
        viewBinding.startProcessingButton.text = "Stop Tracking"
        viewBinding.startProcessingButton.backgroundTintList =
            ContextCompat.getColorStateList(this, R.color.red)
        viewBinding.processedFrameView.visibility = View.VISIBLE

        videoProcessor?.reset()

        if (Settings.ExportData.videoDATA) {
            val dims = videoProcessor?.getModelDimensions()
            val width = dims?.first ?: 416
            val height = dims?.second ?: 416
            val outputPath = getProcessedVideoOutputPath()
            processedVideoRecorder = ProcessedVideoRecorder(width, height, outputPath)
            processedVideoRecorder?.start()
        }
    }

    private fun stopProcessingAndRecording() {
        isRecording = false
        isProcessing = false
        viewBinding.startProcessingButton.text = "Start Tracking"
        viewBinding.startProcessingButton.backgroundTintList =
            ContextCompat.getColorStateList(this, R.color.blue)
        viewBinding.processedFrameView.visibility = View.GONE
        viewBinding.processedFrameView.setImageBitmap(null)
        processedVideoRecorder?.stop()
        processedVideoRecorder = null

        val outputPath = getProcessedImageOutputPath()
        processedFrameRecorder = ProcessedFrameRecorder(outputPath)
        with(Settings.ExportData) {
            if (frameIMG) {
                val bitmap = videoProcessor?.exportTraceForInference()
                if (bitmap != null) { processedFrameRecorder?.save(bitmap) }
            }
        }

        // Initialize the inference results.
        initializeInferenceResult()

        // Retrieve the tracking coordinates from VideoProcessor.
        trackingCoordinates = videoProcessor?.getTrackingCoordinatesString() ?: ""

        // Start XameraAR Activity when the user stops processing.
        val intent = Intent(this, com.xamera.ar.core.components.java.sharedcamera.SharedCameraActivity::class.java)
        // Pass the letter (or inference result) for the 2D Letter Cube.
        intent.putExtra("LETTER_KEY", inferenceResult)
        // Use trackingCoordinates if available; otherwise fallback to a default coordinate string.
        val pathCoordinates = if (trackingCoordinates.isNotEmpty()) {
            trackingCoordinates
        } else {
            "0.0,0.0,0.0;5.0,10.0,-5.0;-5.0,15.0,10.0;20.0,-5.0,5.0;-10.0,0.0,-10.0;10.0,-15.0,15.0;0.0,20.0,-5.0"
        }
        intent.putExtra("PATH_COORDINATES", pathCoordinates)
        startActivity(intent)
    }

    // Inference algorithm for letters and digits.
    private fun initializeInferenceResult() {
        if (isLetterSelected) {
            // Inference logic for letters
            inferenceResult = "ML - Inference: Letters"
        } else if (isDigitSelected) {
            // Inference logic for digits
            inferenceResult = runDigitRecognitionInference()
        }
    }

    // Run the digit inference using the exported trace bitmap.
    private fun runDigitRecognitionInference(): String {
        // Get the 28x28 trace image.
        val digitBitmap = videoProcessor?.exportTraceForInference()
        if (digitBitmap == null) {
            Log.e("MainActivity", "No digit image available for inference")
            return "Error"
        }
        // Convert the image to grayscale (but keep ARGB_8888 config).
        val grayBitmap = convertToGrayscale(digitBitmap)
        // Convert the grayscale bitmap into a ByteBuffer containing 784 floats.
        val inputBuffer = convertBitmapToGrayscaleByteBuffer(grayBitmap)

        // Prepare output array assuming 10 classes (digits 0-9).
        val outputArray = Array(1) { FloatArray(10) }

        if (tfliteInterpreter == null) {
            Log.e("MainActivity", "Digit model interpreter not set")
            return "Error"
        }
        // Run inference.
        tfliteInterpreter?.run(inputBuffer, outputArray)

        // Determine the predicted digit.
        val predictedDigit = outputArray[0].indices.maxByOrNull { outputArray[0][it] } ?: -1
        Log.d("MainActivity", "Digit model predicted: $predictedDigit")
        return predictedDigit.toString()
    }

    /**
     * Converts a bitmap to grayscale while preserving the ARGB_8888 configuration.
     */
    private fun convertToGrayscale(bitmap: Bitmap): Bitmap {
        val grayscaleBitmap = Bitmap.createBitmap(bitmap.width, bitmap.height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(grayscaleBitmap)
        val paint = Paint()
        val colorMatrix = ColorMatrix().apply { setSaturation(0f) }
        val filter = ColorMatrixColorFilter(colorMatrix)
        paint.colorFilter = filter
        canvas.drawBitmap(bitmap, 0f, 0f, paint)
        return grayscaleBitmap
    }

    /**
     * Converts a 28x28 ARGB_8888 bitmap (with grayscale values) into a ByteBuffer
     * of shape [1, 28, 28, 1] (784 floats). Each pixel’s red channel is used.
     */
    private fun convertBitmapToGrayscaleByteBuffer(bitmap: Bitmap): ByteBuffer {
        val inputSize = bitmap.width * bitmap.height // should be 28*28 = 784
        val byteBuffer = ByteBuffer.allocateDirect(inputSize * 4) // float size = 4 bytes
        byteBuffer.order(ByteOrder.nativeOrder())

        val intValues = IntArray(bitmap.width * bitmap.height)
        bitmap.getPixels(intValues, 0, bitmap.width, 0, 0, bitmap.width, bitmap.height)

        // Extract the red channel from each pixel (since in grayscale, R=G=B).
        for (pixel in intValues) {
            // Extract red component from ARGB.
            val r = (pixel shr 16 and 0xFF).toFloat()
            // Normalize the value to [0, 1]. Adjust normalization as per your model's training.
            val normalized = r / 255.0f
            byteBuffer.putFloat(normalized)
        }
        return byteBuffer
    }

    private fun processFrameWithVideoProcessor() {
        if (isProcessingFrame) return
        val bitmap = viewBinding.viewFinder.bitmap ?: return
        isProcessingFrame = true
        videoProcessor?.processFrame(bitmap) { processedFrames ->
            runOnUiThread {
                processedFrames?.let { (outputBitmap, preprocessedBitmap) ->
                    if (isProcessing) {
                        viewBinding.processedFrameView.setImageBitmap(outputBitmap)
                        with(Settings.ExportData) {
                            if (videoDATA) {
                                processedVideoRecorder?.recordFrame(preprocessedBitmap)
                            }
                        }
                    }
                }
                isProcessingFrame = false
            }
        }
    }

    private fun getProcessedVideoOutputPath(): String {
        @Suppress("DEPRECATION")
        val moviesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_MOVIES)
        if (!moviesDir.exists()) {
            moviesDir.mkdirs()
        }
        return File(moviesDir, "Processed_${System.currentTimeMillis()}.mp4").absolutePath
    }

    private fun getProcessedImageOutputPath(): String {
        @Suppress("DEPRECATION")
        val picturesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES)
        if (!picturesDir.exists()) {
            picturesDir.mkdirs()
        }
        return File(picturesDir, "Processed_${System.currentTimeMillis()}.jpg").absolutePath
    }

    // Loads the TFlite Model:
    // - First sets number of threads for CPU fallback.
    // - Second checks if NNAPI is available.
    // - Third checks if GPU delegate is available.
    // If both are unavailable, CPU is used for inference.
    private fun loadTFLiteModelOnStartupThreaded(modelName: String) {
        Thread {
            val bestLoadedPath = copyAssetModelBlocking(modelName)
            runOnUiThread {
                if (bestLoadedPath.isNotEmpty()) {
                    try {
                        val options = Interpreter.Options().apply {
                            // Use all available cores for CPU fallback.
                            setNumThreads(Runtime.getRuntime().availableProcessors())
                        }

                        var delegateAdded = false

                        // Attempt to add NNAPI delegate.
                        try {
                            val nnApiDelegate = NnApiDelegate()
                            options.addDelegate(nnApiDelegate)
                            delegateAdded = true
                            Log.d("MainActivity", "NNAPI delegate added successfully.")
                        } catch (e: Exception) {
                            Log.d("MainActivity", "NNAPI delegate unavailable, falling back to GPU delegate.", e)
                        }

                        // If NNAPI wasn't added, try the GPU delegate instead.
                        if (!delegateAdded) {
                            try {
                                val gpuDelegate = GpuDelegate()
                                options.addDelegate(gpuDelegate)
                                Log.d("MainActivity", "GPU delegate added successfully.")
                            } catch (e: Exception) {
                                Log.d("MainActivity", "GPU delegate unavailable, will use CPU only.", e)
                            }
                        }

                        // Initialize the interpreter with the best options.
                        when (modelName) {
                            "YOLOv3_float32.tflite" -> {
                                videoProcessor?.setInterpreter(Interpreter(loadMappedFile(bestLoadedPath), options))
                            }
                            "DigitRecog_float32.tflite" -> {
                                tfliteInterpreter = Interpreter(loadMappedFile(bestLoadedPath), options)
                            }
                            else -> Log.d("MainActivity", "No model processing method defined for $modelName")
                        }

                    } catch (e: Exception) {
                        Toast.makeText(this, "Error loading TFLite model: ${e.message}", Toast.LENGTH_LONG).show()
                        Log.d("MainActivity", "TFLite Interpreter error", e)
                    }
                } else {
                    Toast.makeText(this, "Failed to copy or load $modelName", Toast.LENGTH_SHORT).show()
                }
            }
        }.start()
    }

    private fun loadMappedFile(modelPath: String): MappedByteBuffer {
        val file = File(modelPath)
        val fileInputStream = file.inputStream()
        val fileChannel = fileInputStream.channel
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, 0, file.length())
    }

    private fun copyAssetModelBlocking(assetName: String): String {
        return try {
            val outFile = File(filesDir, assetName)
            if (outFile.exists() && outFile.length() > 0) {
                return outFile.absolutePath
            }
            assets.open(assetName).use { input ->
                FileOutputStream(outFile).use { output ->
                    val buffer = ByteArray(4 * 1024)
                    var bytesRead: Int
                    while (input.read(buffer).also { bytesRead = it } != -1) {
                        output.write(buffer, 0, bytesRead)
                    }
                    output.flush()
                }
            }
            outFile.absolutePath
        } catch (e: Exception) {
            Log.e("MainActivity", "Error copying asset $assetName: ${e.message}")
            ""
        }
    }

    private var isFrontCamera = false
    private fun switchCamera() {
        if (isRecording) {
            stopProcessingAndRecording()
        }
        isFrontCamera = !isFrontCamera
        cameraHelper.isFrontCamera = isFrontCamera
        cameraHelper.closeCamera()
        cameraHelper.openCamera()
    }

    override fun onResume() {
        super.onResume()
        cameraHelper.startBackgroundThread()
        if (viewBinding.viewFinder.isAvailable) {
            if (allPermissionsGranted()) {
                cameraHelper.openCamera()
            } else {
                requestPermissionLauncher.launch(REQUIRED_PERMISSIONS)
            }
        } else {
            viewBinding.viewFinder.surfaceTextureListener = textureListener
        }
    }

    override fun onPause() {
        if (isRecording) {
            stopProcessingAndRecording()
        }
        cameraHelper.closeCamera()
        cameraHelper.stopBackgroundThread()
        super.onPause()
    }

    private fun allPermissionsGranted(): Boolean {
        return REQUIRED_PERMISSIONS.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
    }
}