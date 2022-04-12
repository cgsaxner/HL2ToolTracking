#include "pch.h"

LinearKalmanFilter::LinearKalmanFilter(cv::Point2f point, float delta)
{
	kf.init(m_state_size, m_meas_size, m_contr_size, m_type);

	cv::Mat state(m_state_size, 1, m_type);  // [x,y,v_x,v_y,w,h]
	cv::Mat meas(m_meas_size, 1, m_type);    // [z_x,z_y,z_w,z_h]
	//cv::Mat procNoise(stateSize, 1, type)
	// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	cv::setIdentity(kf.transitionMatrix);
	SetTransitionMatrix(delta);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	kf.measurementMatrix = cv::Mat::zeros(m_meas_size, m_state_size, m_type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     0    0  ]
	// [ 0    Ey  0     0     0    0  ]
	// [ 0    0   Ev_x  0     0    0  ]
	// [ 0    0   0     Ev_y  0    0  ]
	// [ 0    0   0     0     Ew   0  ]
	// [ 0    0   0     0     0    Eh ]
	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf.processNoiseCov.at<float>(0) = 1e-2f;
	kf.processNoiseCov.at<float>(7) = 1e-2f;
	kf.processNoiseCov.at<float>(14) = 5.0f;
	kf.processNoiseCov.at<float>(21) = 5.0f;
	kf.processNoiseCov.at<float>(28) = 1e-2f;
	kf.processNoiseCov.at<float>(35) = 1e-2f;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

	// Set the state matrix with the first measurement
	meas.at<float>(0) = point.x;
	meas.at<float>(1) = point.y;
	meas.at<float>(2) = m_roi_width;
	meas.at<float>(3) = m_roi_width;

	kf.errorCovPre.at<float>(0) = 1.0f; // px
	kf.errorCovPre.at<float>(7) = 1.0f; // px
	kf.errorCovPre.at<float>(14) = 1.0f;
	kf.errorCovPre.at<float>(21) = 1.0f;
	kf.errorCovPre.at<float>(28) = 1.0f; // px
	kf.errorCovPre.at<float>(35) = 1.0f; // px

	state.at<float>(0) = meas.at<float>(0);
	state.at<float>(1) = meas.at<float>(1);
	state.at<float>(2) = 0.0f;
	state.at<float>(3) = 0.0f;
	state.at<float>(4) = meas.at<float>(2);
	state.at<float>(5) = meas.at<float>(3);

	kf.statePost = state;

	initialized = true;
}

cv::Rect LinearKalmanFilter::Predict(float delta)
{
	SetTransitionMatrix(delta);

	cv::Mat state = kf.predict();

	cv::Rect pred_rect;
	pred_rect.width = (int)state.at<float>(4);
	pred_rect.height = (int)state.at<float>(5);
	pred_rect.x = (int)state.at<float>(0) - pred_rect.width / 2;
	pred_rect.y = (int)state.at<float>(1) - pred_rect.height / 2;

	return pred_rect;
}

void LinearKalmanFilter::PredictAndCorrect(cv::Point2f point)
{
	cv::Mat meas(m_meas_size, 1, m_type);

	meas.at<float>(0) = point.x;
	meas.at<float>(1) = point.y;
	meas.at<float>(2) = m_roi_width;
	meas.at<float>(3) = m_roi_width;

	// first predict, to update the internal statePre variable
	cv::Mat prediction = kf.predict();

	// then correct with the new measurement
	kf.correct(meas);
}

void LinearKalmanFilter::SetTransitionMatrix(float delta)
{
	kf.transitionMatrix.at<float>(2) = delta;
	kf.transitionMatrix.at<float>(9) = delta;
}