#pragma once
class LinearKalmanFilter
{
public:
	LinearKalmanFilter(cv::Point2f point, float delta);
	LinearKalmanFilter() = default;
	~LinearKalmanFilter() = default;

	cv::Rect Predict(float delta);
	void PredictAndCorrect(cv::Point2f point, float delta);

	cv::KalmanFilter kf;
	bool initialized = false;

private:
	void SetTransitionMatrix(float delta);

	float m_delta_time = 10.0f;
	int m_state_size = 6;
	int m_meas_size = 4;
	int m_contr_size = 0;
	unsigned int m_type = CV_32F;
	float m_accel_noise_mag = 0.5f;

	float m_roi_width = 40.0f;
};

