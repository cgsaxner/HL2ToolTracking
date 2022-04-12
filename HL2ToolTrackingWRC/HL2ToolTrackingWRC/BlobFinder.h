#pragma once 

struct Center
{
	cv::Point2d location;
	double radius;
	double confidence;
};

class BlobFinder
{
public:
	BlobFinder(
		cv::Mat fMat);
	BlobFinder() = default;
	~BlobFinder() = default;

	void MatchBlobs(
		_In_ std::vector<cv::Point2f>& lfPointCandidates,
		_In_ std::vector<cv::Point2f>& rfPointCandidates,
		_Out_ std::vector<cv::Point2f>& lfPointsSorted,
		_Out_ std::vector<cv::Point2f>& rfPointsSorted);

	void DetectBlobs(
		_In_ const cv::Mat& image,
		_Out_ std::vector<cv::Point2f>& blobs);

public:
	bool instrumentFound = false;

private:
	void PruneBlobs(
		_In_ std::vector<cv::Point2f> targetBlobs,
		_In_ std::vector<cv::Point2f> sourceBlobs,
		_In_ std::vector<cv::Point3f> targetEpilines,
		_In_ std::vector<cv::Point3f> sourceEpilines,
		_Out_ std::vector<cv::Point2f>& targetPointsSorted,
		_Out_ std::vector<cv::Point2f>& sourcePointsSorted);

	void FilterBlobs(
		_In_ const cv::Mat& threshImage,
		_Out_ std::vector<Center>& centers);

private:
	// camera parameters
	cv::Mat m_fMat;

	// blob detector
	cv::SimpleBlobDetector::Params m_params;

	// assignment problem solver
	AssignmentProblemSolver m_solver;
};

