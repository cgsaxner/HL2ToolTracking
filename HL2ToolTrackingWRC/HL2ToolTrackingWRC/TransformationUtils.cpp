#include "pch.h"

void ProjectOnRMFrame(
	float fx, float fy,
	float cx, float cy,
	std::vector<Eigen::Vector3d> cameraPoints,
	std::vector<cv::Point2f>& pixels)
{
	pixels.clear();
	for (int i = 0; i < cameraPoints.size(); i++)
	{
		double xP = cameraPoints[i][0] / cameraPoints[i][2];
		double yP = cameraPoints[i][1] / cameraPoints[i][2];

		float u = (fx * (float)xP + cx);
		float v = (fy * (float)yP + cy);

		pixels.push_back(cv::Point2f(u, v));
	}
}

void TransformationError(
	_In_ cv::Mat& reference,
	_In_ Eigen::Matrix4f& source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector)
{
	cv::Mat referenceD, sourceD;
	cv::Mat sourceCV = cv::Mat(4, 4, CV_32F, source.data()).t();
	reference.convertTo(referenceD, CV_64F);
	sourceCV.convertTo(sourceD, CV_64F);

	ComputeDifferenceBetweenMats(
		referenceD, sourceD, angularError,
		translationalError, translationVector);
}

void TransformationError(
	_In_ Eigen::Matrix4d& reference,
	_In_ cv::Mat& source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector)
{
	cv::Mat referenceCV = cv::Mat(4, 4, CV_64F, reference.data()).t();
	cv::Mat sourceD;
	source.convertTo(sourceD, CV_64F);

	ComputeDifferenceBetweenMats(
		referenceCV, sourceD, angularError,
		translationalError, translationVector);
}

void TransformationError(
	_In_ Eigen::Matrix4d& reference,
	_In_ Eigen::Matrix4f& source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector)
{
	cv::Mat referenceCV = cv::Mat(4, 4, CV_64F, reference.data()).t();
	cv::Mat sourceF = cv::Mat(4, 4, CV_32F, source.data()).t();
	cv::Mat sourceD;
	sourceF.convertTo(sourceD, CV_64F);

	ComputeDifferenceBetweenMats(
		referenceCV, sourceD, angularError,
		translationalError, translationVector);
}

void TransformationError(
	_In_ Eigen::Matrix4f& reference,
	_In_ Eigen::Matrix4f& source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector)
{
	cv::Mat referenceCV = cv::Mat(4, 4, CV_32F, reference.data()).t();
	cv::Mat sourceF = cv::Mat(4, 4, CV_32F, source.data()).t();
	cv::Mat sourceD, referenceD;
	referenceCV.convertTo(referenceD, CV_64F);
	sourceF.convertTo(sourceD, CV_64F);

	ComputeDifferenceBetweenMats(
		referenceD, sourceD, angularError, translationalError, translationVector);
}

void ComputeDifferenceBetweenMats(
	_In_ cv::Mat reference,
	_In_ cv::Mat source,
	_Out_ double& angularError,
	_Out_ double& translationalError,
	_Out_ cv::Mat& translationVector)
{
	cv::Mat diffRotVec;
	translationVector =
		reference(cv::Rect(3, 0, 1, 3)) - source(cv::Rect(3, 0, 1, 3));
	cv::Mat diffRot =
		reference(cv::Rect(0, 0, 3, 3)).inv() * source(cv::Rect(0, 0, 3, 3));
	cv::Rodrigues(diffRot, diffRotVec);

	angularError = cv::norm(diffRotVec);
	translationalError = cv::norm(translationVector);
}

void QuaternionToRotationMatrix(
	_In_ const double x,
	_In_ const double y,
	_In_ const double z,
	_In_ const double w,
	_Out_ Eigen::Matrix3d& R)
{
	Eigen::Quaterniond quat = Eigen::Quaterniond(w, x, y, z);
	R = quat.normalized().toRotationMatrix();
}

// [qx, qy, qz, qw, tx, ty, tz]
void TransformationToQuaternionAndTranslation(
	_In_ cv::Mat& T,
	_Out_ std::vector<float>& t)
{
	t = std::vector<float>(7);
	float trace = T.at<float>(0, 0) + T.at<float>(1, 1) + T.at<float>(2, 2);
	if (trace > 0) {// I changed M_EPSILON to 0
		float s = 0.5f / sqrtf(trace + 1.0f);
		t[3] = 0.25f / s; // w
		t[0] = (T.at<float>(2, 1) - T.at<float>(1, 2)) * s; // x
		t[1] = (T.at<float>(0, 2) - T.at<float>(2, 0)) * s; // y
		t[2] = (T.at<float>(1, 0) - T.at<float>(0, 1)) * s;	// z
	}
	else {
		if (T.at<float>(0, 0) > T.at<float>(1, 1) && T.at<float>(0, 0) > T.at<float>(2, 2)) {
			float s = 2.0f * sqrtf(1.0f + T.at<float>(0, 0) - T.at<float>(1, 1) - T.at<float>(2, 2));
			t[3] = (T.at<float>(2, 1) - T.at<float>(1, 2)) / s;
			t[0] = 0.25f * s;
			t[1] = (T.at<float>(0, 1) + T.at<float>(1, 0)) / s;
			t[2] = (T.at<float>(0, 2) + T.at<float>(2, 0)) / s;
		}
		else if (T.at<float>(1, 1) > T.at<float>(2, 2)) {
			float s = 2.0f * sqrtf(1.0f + T.at<float>(1, 1) - T.at<float>(0, 0) - T.at<float>(2, 2));
			t[3] = (T.at<float>(0, 2) - T.at<float>(2, 0)) / s;
			t[0] = (T.at<float>(0, 1) + T.at<float>(1, 0)) / s;
			t[1] = 0.25f * s;
			t[2] = (T.at<float>(1, 2) + T.at<float>(2, 1)) / s;
		}
		else {
			float s = 2.0f * sqrtf(1.0f + T.at<float>(2, 2) - T.at<float>(0, 0) - T.at<float>(1, 1));
			t[3] = (T.at<float>(1, 0) - T.at<float>(0, 1)) / s;
			t[0] = (T.at<float>(0, 2) + T.at<float>(2, 0)) / s;
			t[1] = (T.at<float>(1, 2) + T.at<float>(2, 1)) / s;
			t[2] = 0.25f * s;
		}
	}
	t[4] = T.at<float>(0, 3);
	t[5] = T.at<float>(1, 3);
	t[6] = T.at<float>(2, 3);
}

// [qx, qy, qz, qw, tx, ty, tz]
void TransformationToQuaternionAndTranslation(
	_In_ Eigen::Matrix4f& T,
	_Out_ std::vector<float>& t)
{
	t = std::vector<float>(7);
	float trace = T(0, 0) + T(1, 1) + T(2, 2);
	if (trace > 0) {// I changed M_EPSILON to 0
		float s = 0.5f / sqrtf(trace + 1.0f);
		t[3] = 0.25f / s;
		t[0] = (T(2, 1) - T(1, 2)) * s;
		t[1] = (T(0, 2) - T(2, 0)) * s;
		t[2] = (T(1, 0) - T(0, 1)) * s;
	}
	else {
		if (T(0, 0) > T(1, 1) && T(0, 0) > T(2, 2)) {
			float s = 2.0f * sqrtf(1.0f + T(0, 0) - T(1, 1) - T(2, 2));
			t[3] = (T(2, 1) - T(1, 2)) / s;
			t[0] = 0.25f * s;
			t[1] = (T(0, 1) + T(1, 0)) / s;
			t[2] = (T(0, 2) + T(2, 0)) / s;
		}
		else if (T(1, 1) > T(2, 2)) {
			float s = 2.0f * sqrtf(1.0f + T(1, 1) - T(0, 0) - T(2, 2));
			t[3] = (T(0, 2) - T(2, 0)) / s;
			t[0] = (T(0, 1) + T(1, 0)) / s;
			t[1] = 0.25f * s;
			t[2] = (T(1, 2) + T(2, 1)) / s;
		}
		else {
			float s = 2.0f * sqrtf(1.0f + T(2, 2) - T(0, 0) - T(1, 1));
			t[3] = (T(1, 0) - T(0, 1)) / s;
			t[0] = (T(0, 2) + T(2, 0)) / s;
			t[1] = (T(1, 2) + T(2, 1)) / s;
			t[2] = 0.25f * s;
		}
	}
	t[4] = T(0, 3);
	t[5] = T(1, 3);
	t[6] = T(2, 3);
}

void RotationMatrixToQuaternion(
	_In_ cv::Mat& R,
	_Out_ Eigen::Vector4f& q)
{
	float trace = R.at<float>(0, 0) + R.at<float>(1, 1) + R.at<float>(2, 2);
	if (trace > 0) {// I changed M_EPSILON to 0
		float s = 0.5f / sqrtf(trace + 1.0f);
		q[3] = 0.25f / s;
		q[0] = (R.at<float>(2, 1) - R.at<float>(1, 2)) * s;
		q[1] = (R.at<float>(0, 2) - R.at<float>(2, 0)) * s;
		q[2] = (R.at<float>(1, 0) - R.at<float>(0, 1)) * s;
	}
	else {
		if (R.at<float>(0, 0) > R.at<float>(1, 1) && R.at<float>(0, 0) > R.at<float>(2, 2)) {
			float s = 2.0f * sqrtf(1.0f + R.at<float>(0, 0) - R.at<float>(1, 1) - R.at<float>(2, 2));
			q[3] = (R.at<float>(2, 1) - R.at<float>(1, 2)) / s;
			q[0] = 0.25f * s;
			q[1] = (R.at<float>(0, 1) + R.at<float>(1, 0)) / s;
			q[2] = (R.at<float>(0, 2) + R.at<float>(2, 0)) / s;
		}
		else if (R.at<float>(1, 1) > R.at<float>(2, 2)) {
			float s = 2.0f * sqrtf(1.0f + R.at<float>(1, 1) - R.at<float>(0, 0) - R.at<float>(2, 2));
			q[3] = (R.at<float>(0, 2) - R.at<float>(2, 0)) / s;
			q[0] = (R.at<float>(0, 1) + R.at<float>(1, 0)) / s;
			q[1] = 0.25f * s;
			q[2] = (R.at<float>(1, 2) + R.at<float>(2, 1)) / s;
		}
		else {
			float s = 2.0f * sqrtf(1.0f + R.at<float>(2, 2) - R.at<float>(0, 0) - R.at<float>(1, 1));
			q[3] = (R.at<float>(1, 0) - R.at<float>(0, 1)) / s;
			q[0] = (R.at<float>(0, 2) + R.at<float>(2, 0)) / s;
			q[1] = (R.at<float>(1, 2) + R.at<float>(2, 1)) / s;
			q[2] = 0.25f * s;
		}
	}
}

void Rotation3D(int alpha, int beta, int gamma, cv::Mat& R)
{
	// Rotation matrices around the X, Y, and Z axis
	float alphaInRadian = alpha * (float)M_PI / 180.0f;
	float betaInRadian = beta * (float)M_PI / 180.0f;
	float gammaInRadian = gamma * (float)M_PI / 180.0f;
	cv::Mat RX = (cv::Mat_<float>(3, 3) <<
		1.0f, 0.0f, 0.0f,
		0.0f, cosf(alphaInRadian), sinf(alphaInRadian),
		0.0f, -sinf(alphaInRadian), cosf(alphaInRadian));
	cv::Mat RY = (cv::Mat_<float>(3, 3) <<
		cosf(betaInRadian), 0.0f, sinf(betaInRadian),
		0.0f, 1.0f, 0.0f,
		-sinf(betaInRadian), 0.0f, cosf(betaInRadian));
	cv::Mat RZ = (cv::Mat_<float>(3, 3) <<
		cosf(gammaInRadian), sinf(gammaInRadian), 0.0f,
		-sinf(gammaInRadian), cosf(gammaInRadian), 0.0f,
		0.0f, 0.0f, 1.0f);
	// Composed rotation matrix with (RX, RY, RZ)
	R = RX * RY * RZ;
}

void Rotation3D(int alpha, int beta, int gamma, Eigen::Matrix3d& R)
{
	// Rotation matrices around the X, Y, and Z axis
	double alphaInRadian = alpha * M_PI / 180.0;
	double betaInRadian = beta * M_PI / 180.0;
	double gammaInRadian = gamma * M_PI / 180.0;
	Eigen::Matrix3d RX, RY, RZ;
	RX <<
		1, 0, 0,
		0, cos(alphaInRadian), sin(alphaInRadian),
		0, -sin(alphaInRadian), cos(alphaInRadian);
	RY <<
		cos(betaInRadian), 0, sin(betaInRadian),
		0, 1, 0,
		-sin(betaInRadian), 0, cos(betaInRadian);
	RZ <<
		cos(gammaInRadian), sin(gammaInRadian), 0,
		-sin(gammaInRadian), cos(gammaInRadian), 0,
		0, 0, 1;

	// Composed rotation matrix with (RX, RY, RZ)
	R = RX * RY * RZ;

}

void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const cv::Mat sourceToTargetTransform,
	_Out_ std::vector<Eigen::Vector3d>& modelPoints)
{
	modelPoints.clear();
	for (int j = 0; j < sourcePoints.cols; j++)
	{
		cv::Mat sourcePoint = cv::Mat::ones(4, 1, CV_32F);
		sourcePoints(cv::Rect(j, 0, 1, sourcePoints.rows)).copyTo(sourcePoint(cv::Rect(0, 0, 1, 3)));
		cv::Mat targetPoint = (sourceToTargetTransform * sourcePoint).t()(cv::Rect(0, 0, 3, 1)).t();
		modelPoints.push_back(Eigen::Vector3d(
			targetPoint.at<float>(0),
			targetPoint.at<float>(1),
			targetPoint.at<float>(2)));
	}
}

void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const cv::Mat sourceToTargetTransform,
	_Out_ cv::Mat& targetPoints)
{
	targetPoints = cv::Mat::zeros(3, sourcePoints.cols, CV_32F);
	for (int j = 0; j < sourcePoints.cols; j++)
	{
		cv::Mat sourcePoint = cv::Mat::ones(4, 1, CV_32F);
		sourcePoints(cv::Rect(j, 0, 1, sourcePoints.rows)).copyTo(sourcePoint(cv::Rect(0, 0, 1, 3)));
		cv::Mat targetPoint = (sourceToTargetTransform * sourcePoint).t()(cv::Rect(0, 0, 3, 1)).t();
		targetPoint.copyTo(targetPoints(cv::Rect(j, 0, 1, targetPoints.rows)));
	}
}

void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ std::vector<Eigen::Vector3d>& targetPoints)
{
	targetPoints.clear();
	for (int j = 0; j < sourcePoints.cols; j++)
	{
		Eigen::Vector4f sourcePoint =
			Eigen::Vector4f(sourcePoints.col(j).at<float>(0), sourcePoints.col(j).at<float>(1), sourcePoints.col(j).at<float>(2), 1.0f);
		Eigen::Vector4f targetPoint = sourceToTargetTransform * sourcePoint;
		targetPoints.push_back(Eigen::Vector3d(
			targetPoint[0],
			targetPoint[1],
			targetPoint[2]));
	}
}

void TransformPoints(
	_In_ cv::Mat sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ cv::Mat& targetPoints)
{
	targetPoints = cv::Mat::zeros(3, sourcePoints.cols, CV_32F);
	for (int j = 0; j < sourcePoints.cols; j++)
	{
		Eigen::Vector4f sourcePoint =
			Eigen::Vector4f(sourcePoints.col(j).at<float>(0), sourcePoints.col(j).at<float>(1), sourcePoints.col(j).at<float>(2), 1.0f);
		Eigen::Vector4f targetPoint = sourceToTargetTransform * sourcePoint;
		targetPoints.col(j).at<float>(0) = targetPoint[0];
		targetPoints.col(j).at<float>(1) = targetPoint[1];
		targetPoints.col(j).at<float>(2) = targetPoint[2];
	}
}

void TransformPoints(
	_In_ std::vector<Eigen::Vector3f> sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ std::vector<Eigen::Vector3d>& targetPoints)
{
	targetPoints.clear();
	for (int i = 0; i < sourcePoints.size(); i++)
	{
		Eigen::Vector4f targetPoint = sourceToTargetTransform *
			Eigen::Vector4f(sourcePoints[i][0], sourcePoints[i][1], sourcePoints[i][2], 1.0f);
		targetPoints.push_back(Eigen::Vector3d(targetPoint[0], targetPoint[1], targetPoint[2]));
	}
}

void TransformPoints(
	_In_ std::vector<Eigen::Vector3f> sourcePoints,
	_In_ const Eigen::Matrix4f sourceToTargetTransform,
	_Out_ cv::Mat& targetPoints)
{
	targetPoints = cv::Mat::zeros(3, sourcePoints.size(), CV_32F);
	for (unsigned int i = 0; i < sourcePoints.size(); i++)
	{
		Eigen::Vector4f targetPoint = sourceToTargetTransform *
			Eigen::Vector4f(sourcePoints[i][0], sourcePoints[i][1], sourcePoints[i][2], 1.0f);
		targetPoints.col(i).at<float>(0) = targetPoint[0];
		targetPoints.col(i).at<float>(1) = targetPoint[1];
		targetPoints.col(i).at<float>(2) = targetPoint[2];
	}
}

// Checks if a matrix is a valid rotation matrix.
bool IsRotationMatrix(cv::Mat& R)
{
	cv::Mat Rt;
	transpose(R, Rt);
	cv::Mat shouldBeIdentity = Rt * R;
	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
	return  norm(I, shouldBeIdentity) < 1e-5;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
void RotationMatrixToEulerAngles(cv::Mat& R, cv::Mat& rvec)
{
	assert(IsRotationMatrix(R));
	rvec = cv::Mat(3, 1, CV_32F);
	float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
		y = atan2(-R.at<float>(2, 0), sy);
		z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
	}
	else
	{
		x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
		y = atan2(-R.at<float>(2, 0), sy);
		z = 0;
	}
	rvec.at<float>(0, 0) = x;
	rvec.at<float>(1, 0) = y;
	rvec.at<float>(2, 0) = z;
}

// Calculates rotation matrix given euler angles.
cv::Mat EulerAnglesToRotationMatrix(cv::Mat& rvec)
{
	// Calculate rotation about x axis
	cv::Mat R_x = (cv::Mat_<float>(3, 3) <<
		1, 0, 0,
		0, cos(rvec.at<float>(0)), -sin(rvec.at<float>(0)),
		0, sin(rvec.at<float>(0)), cos(rvec.at<float>(0))
		);

	// Calculate rotation about y axis
	cv::Mat R_y = (cv::Mat_<float>(3, 3) <<
		cos(rvec.at<float>(1)), 0, sin(rvec.at<float>(1)),
		0, 1, 0,
		-sin(rvec.at<float>(1)), 0, cos(rvec.at<float>(1))
		);

	// Calculate rotation about z axis
	cv::Mat R_z = (cv::Mat_<float>(3, 3) <<
		cos(rvec.at<float>(2)), -sin(rvec.at<float>(2)), 0,
		sin(rvec.at<float>(2)), cos(rvec.at<float>(2)), 0,
		0, 0, 1);

	// Combined rotation matrix
	cv::Mat R = R_z * R_y * R_x;

	return R;

}