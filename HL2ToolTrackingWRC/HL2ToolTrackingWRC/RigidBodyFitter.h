#pragma once
#define NOMINMAX

class edge_match
{
public:
	float dist;
	int a, b, x, y;
	edge_match(float dist, int a, int b, int x, int y)
		: dist(dist), a(a), b(b), x(x), y(y)
	{
		;
	}
};

class edge_match_comp
{
public:
	bool operator() (const edge_match& lhs, const edge_match& rhs) const
	{
		return (lhs.dist > rhs.dist);
	}
};

class RigidBodyFitter
{
public:
	RigidBodyFitter(
		cv::Mat fMat,
		cv::Mat lfProj,
		cv::Mat rfProj);
	RigidBodyFitter() = default;
	~RigidBodyFitter() = default;

	void Triangulate(
		_In_ std::vector<cv::Point2f> lfPoints,
		_In_ std::vector<cv::Point2f> rfPoints,
		_Out_ cv::Mat& lfCameraPoints);

	void FitRigidBody(
		_In_ const cv::Mat& lfPoints,
		_In_ Marker& marker,
		_In_ float& max_deviation,
		_Out_ float& deviation,
		_Out_ cv::Mat& transform,
		_Out_ std::vector<int>& assignment);

private:
	cv::Mat m_fMat, m_lfProj, m_rfProj;
	AssignmentProblemSolver m_solver;

private:
	void Fit3DPointsToObjectTemplate(
		_In_ const cv::Mat& points_3D,
		_In_ Marker& marker,
		_Out_ cv::Mat& RT,
		_Out_ float& avg_deviation,
		_Out_ std::vector<int>& assignment);

	void FitTwoPointSets(
		_In_ const cv::Mat& point_set_0,
		_In_ const cv::Mat& point_set_1,
		_In_ int num_points,
		_Out_ cv::Mat& RT,
		_Out_ float& avg_deviation);
};

