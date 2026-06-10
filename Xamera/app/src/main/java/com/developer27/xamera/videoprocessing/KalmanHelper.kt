package com.developer27.xamera.videoprocessing

import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.video.KalmanFilter

class KalmanHelper {
    private val kalmanFilter: KalmanFilter = KalmanFilter(4, 2).apply {
        _transitionMatrix = Mat.eye(4, 4, CvType.CV_32F).apply {
            put(0, 2, 1.0)
            put(1, 3, 1.0)
        }
        _measurementMatrix = Mat.eye(2, 4, CvType.CV_32F)
        _processNoiseCov = Mat.eye(4, 4, CvType.CV_32F).apply { setTo(Scalar(1e-4)) }
        _measurementNoiseCov = Mat.eye(2, 2, CvType.CV_32F).apply { setTo(Scalar(1e-2)) }
        _errorCovPost = Mat.eye(4, 4, CvType.CV_32F)
    }

    fun filter(point: Point): Pair<Double, Double> {
        val measurement = Mat(2, 1, CvType.CV_32F).apply {
            put(0, 0, point.x)
            put(1, 0, point.y)
        }
        kalmanFilter.predict()
        val corrected = kalmanFilter.correct(measurement)
        return corrected[0, 0][0] to corrected[1, 0][0]
    }
}
