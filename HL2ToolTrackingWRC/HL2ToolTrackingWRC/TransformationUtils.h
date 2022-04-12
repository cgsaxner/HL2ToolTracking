#pragma once

#define NOMINMAX
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void ProjectOnRMFrame(
	float fx, float fy,
	float cx, float cy,
	std::vector<Eigen::Vector3d> cameraPoints,
	std::vector<cv::Point2f>& pixels);

void TransformationError(
	_In_ cv::Mat& reference,
	_In_ Eigen::Matrix4f& source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector);

void TransformationError(
	_In_ Eigen::Matrix4d& reference,
	_In_ cv::Mat& source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector);

void TransformationError(
	_In_ Eigen::Matrix4d& reference,
	_In_ Eigen::Matrix4f& source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector);

void ComputeDifferenceBetweenMats(
	_In_ cv::Mat reference,
	_In_ cv::Mat source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector);

void Rotation3D(int alpha, int beta, int gamma, cv::Mat& R);

void Rotation3D(int alpha, int beta, int gamma, Eigen::Matrix3d& R);


void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const cv::Mat sourceToTargetTransform,
	_Out_ std::vector<Eigen::Vector3d>& modelPoints);

void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const cv::Mat sourceToTargetTransform,
	_Out_ cv::Mat& targetPoints);

void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ std::vector<Eigen::Vector3d>& targetPoints);

void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ cv::Mat& targetPoints);

void TransformPoints(
	_In_ std::vector<Eigen::Vector3f> sourcePoints,
	_In_ const cv::Mat sourceToTargetTransform,
	_Out_ std::vector<Eigen::Vector3d>& targetPoints);

template <typename InVecT>
void TransformPoints(
	_In_ std::vector<Eigen::Matrix<InVecT, 3, 1>> sourcePoints,
	_In_ const cv::Mat sourceToTargetTransform,
	_Out_ cv::Mat& targetPoints)
{
	targetPoints = cv::Mat::zeros(3, sourcePoints.size(), CV_32F);
	for (int j = 0; j < sourcePoints.size(); j++)
	{
		cv::Mat sourcePoint = (cv::Mat_<float>(4, 1) <<
			sourcePoints[j][0], sourcePoints[j][1], sourcePoints[j][2], 1.0f);
		cv::Mat targetPoint = (sourceToTargetTransform * sourcePoint).t()(cv::Rect(0, 0, 3, 1)).t();
		targetPoint.copyTo(targetPoints(cv::Rect(j, 0, 1, targetPoints.rows)));
	}
}

template <typename InVecT, typename OutVecT>
void TransformPoints(
	_In_ std::vector<Eigen::Matrix<InVecT, 3, 1>> sourcePoints,
	_In_ const cv::Mat sourceToTargetTransform,
	_Out_ std::vector<Eigen::Matrix<OutVecT, 3, 1>>& targetPoints)
{
	targetPoints.clear();
	for (int j = 0; j < sourcePoints.size(); j++)
	{
		cv::Mat sourcePoint = (cv::Mat_<float>(4, 1) <<
			sourcePoints[j][0], sourcePoints[j][1], sourcePoints[j][2], 1.0f);
		cv::Mat targetPoint = sourceToTargetTransform * sourcePoint;
		targetPoints.push_back(Eigen::Matrix<OutVecT, 3, 1>(
			targetPoint.at<float>(0),
			targetPoint.at<float>(1),
			targetPoint.at<float>(2)));
	}
}

template <typename InVecT, typename OutVecT>
void TransformPoints(
	_In_ std::vector<Eigen::Matrix<InVecT, 3, 1>> sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ std::vector<Eigen::Matrix<OutVecT, 3, 1>>& targetPoints)
{
	targetPoints.clear();
	for (int i = 0; i < sourcePoints.size(); i++)
	{
		Eigen::Matrix<OutVecT, 3, 1> targetPoint = sourceToTargetTransform *
			Eigen::Matrix<InVecT, 3, 1>(sourcePoints[i][0], sourcePoints[i][1], sourcePoints[i][2], 1.0f);
		targetPoints.push_back(Eigen::Matrix<OutVecT, 3, 1>(targetPoint[0], targetPoint[1], targetPoint[2]));
	}
}

void TransformPoints(
	_In_ std::vector<Eigen::Vector3f> sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ cv::Mat& targetPoints);

// Checks if a matrix is a valid rotation matrix.
bool IsRotationMatrix(cv::Mat& R);

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
void RotationMatrixToEulerAngles(cv::Mat& R, cv::Mat& rvec);

// Calculates rotation matrix given euler angles.
cv::Mat EulerAnglesToRotationMatrix(cv::Mat& rvec);

void RotationMatrixToQuaternion(
	_In_ cv::Mat& R,
	_Out_ Eigen::Vector4f& q);

void TransformationToQuaternionAndTranslation(
	_In_ cv::Mat& T,
	_Out_ std::vector<float>& t);

void TransformationToQuaternionAndTranslation(
	_In_ Eigen::Matrix4f& T,
	_Out_ std::vector<float>& t);

void QuaternionToRotationMatrix(
	_In_ const double x,
	_In_ const double y,
	_In_ const double z,
	_In_ const double w,
	_Out_ Eigen::Matrix3d& R);