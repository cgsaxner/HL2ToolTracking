#pragma once

#define NOMINMAX
#define _USE_MATH_DEFINES

struct ScaatUpdateFrame
{
	float delta;
	uint64_t timestamp;
	ResearchModeSensorType type;
	bool hasAssignment;
	std::vector<int> assignment;
	std::vector<cv::Point2f> updatePoints;
	Eigen::Matrix4f refToCamTransform;
};

class ScaatEKF
{
public:
	ScaatEKF(
		int stateSize,
		int measurementSize,
		int controlSize,
		float lfFx, float lfFy, float lfCx, float lfCy,
		float rfFx, float rfFy, float rfCx, float rfCy,
		cv::Mat modelToLfInit);

	ScaatEKF() = default;
	~ScaatEKF() = default;

	void MainUpdateLoop(
		_In_ Marker marker,
		_In_ uint64_t timestamp,
		_In_ std::vector<cv::Point2f> blobs,
		_In_ ResearchModeSensorType sensorType,
		_In_ cv::Mat camToRefTransform,
		_Out_ Eigen::Matrix4f& modelToRefTransformE);

	void StateToTransform(
		_In_ cv::Mat stateX,
		_Out_ cv::Mat& transform);

	void StateToTransform(
		_In_ cv::Mat stateX,
		_Out_ Eigen::Matrix4f& transform);

public:
	bool initialized = false;

	int lfSkippedFrames;
	int rfSkippedFrames;

	cv::Mat statePreX;
	cv::Mat statePostX;
	cv::Mat gainK;

	uint64_t lastTimestamp = 0;

private:
	void TimeUpdate(float deltaT);

	void MeasurementUpdate(
		_In_ ResearchModeSensorType type,
		_In_ cv::Mat measurement,
		_In_ Eigen::Vector3f modelPoint,
		_In_ Eigen::Matrix4f refToCamTransform,
		_Out_ cv::Mat& measurementPrediction);

	void PredictAndCorrect(
		_In_ std::vector<int> assignment,
		_In_ std::vector<cv::Point2f> pointsSorted,
		_In_ ResearchModeSensorType type,
		_In_ Eigen::Matrix4f refToCamTransform,
		_Inout_ Marker& marker,
		_Inout_ float& delta);

	bool ComputeAssignment(
		_In_ std::vector<cv::Point2f> pointsSorted,
		_In_ std::vector<cv::Point2f> lastPoints,
		_Out_ std::vector<int>& assignment);

	float CalculateDistance(
		const cv::Point2f p1,
		const cv::Point2f p2);

	void MeasurementFunctionh(
		_In_ cv::Mat stateX,
		_In_ Eigen::Matrix4f worldToCam,
		_In_ Eigen::Vector3f modelPoint,
		_In_ float fx, _In_ float fy,
		_In_ float cx, _In_ float cy);

	void SetStateTransitionMatrixA(float deltaT);

	void SetProcessNoiseCovQ(float deltaT);

private:
	std::vector<cv::Scalar> colors;

	cv::Mat transitionMatrixA;
	cv::Mat measurementPredZ;
	cv::Mat measurementMatrixH;
	cv::Mat processNoiseCovQ;
	cv::Mat measurementNoiseCovR;
	cv::Mat errorCovPreP;
	cv::Mat errorCovPostP;

	cv::Mat temp1;
	cv::Mat temp2;
	cv::Mat temp3;
	cv::Mat temp4;
	cv::Mat temp5;

	float m_lfFx = 0.0f;
	float m_lfFy = 0.0f;
	float m_lfCx = 0.0f;
	float m_lfCy = 0.0f;

	float m_rfFx = 0.0f;
	float m_rfFy = 0.0f;
	float m_rfCx = 0.0f;
	float m_rfCy = 0.0f;

	float qwExt = 1.0f;
	float qxExt = 0.0f;
	float qyExt = 0.0f;
	float qzExt = 0.0f;

	int m_state_size = 12;
	int m_meas_size = 2;
	int m_contr_size = 0;
	unsigned int m_type = CV_32F;
	float m_accel_noise_mag = 0.5f;
};


