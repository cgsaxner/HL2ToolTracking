#include "pch.h"

#define DBG_ENABLE_VERBOSE_LOGGING 0

BlobFinder::BlobFinder(
	cv::Mat fMat) :
	m_fMat(fMat)
{
	m_params.minThreshold = 240;
	m_params.maxThreshold = 255;

	m_params.filterByColor = false;
	m_params.blobColor = 0;

	m_params.filterByArea = true;
	m_params.minArea = 30;
	m_params.maxArea = 800;

	m_params.filterByCircularity = true;
	m_params.minCircularity = 0.4f;

	m_params.filterByConvexity = true;
	m_params.minConvexity = 0.2f;

	m_params.filterByInertia = true;
	m_params.minInertiaRatio = 0.4f;

	m_solver = AssignmentProblemSolver();
}

void BlobFinder::DetectBlobs(
	_In_ const cv::Mat& image,
	_Out_ std::vector<cv::Point2f>& blobs)
{
	cv::Mat blur, mask;
	std::vector<cv::KeyPoint> keypoints;
	blobs.clear();

#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"BlobFinder::DetectBlobs: Detecting blobs...\n");
#endif
	cv::medianBlur(image, blur, 3);
	cv::threshold(blur, mask, m_params.minThreshold, m_params.maxThreshold, cv::THRESH_BINARY);
	std::vector<Center> curCenters;
	std::vector<std::vector<Center>> centers;

	// find the blobs...
	FilterBlobs(mask, curCenters);

	// check if blobs are new...
	std::vector<std::vector<Center>> newCenters;
	for (size_t i = 0; i < curCenters.size(); i++)
	{
		bool isNew = true;
		for (size_t j = 0; j < centers.size(); j++)
		{
			double dist = norm(centers[j][centers[j].size() / 2].location - curCenters[i].location);
			isNew = dist >= m_params.minDistBetweenBlobs && dist >= centers[j][centers[j].size() / 2].radius && dist >= curCenters[i].radius;
			if (!isNew)
			{
				centers[j].push_back(curCenters[i]);

				size_t k = centers[j].size() - 1;
				while (k > 0 && curCenters[i].radius < centers[j][k - 1].radius)
				{
					centers[j][k] = centers[j][k - 1];
					k--;
				}
				centers[j][k] = curCenters[i];
				break;
			}
		}
		if (isNew)
			newCenters.push_back(std::vector<Center>(1, curCenters[i]));
	}
	std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));

#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"BlobFinder::DetectBlobs: Computing keypoints...\n");
#endif

	for (size_t i = 0; i < centers.size(); i++)
	{
		cv::Point2d sumPoint(0, 0);
		double normalizer = 0;
		for (size_t j = 0; j < centers[i].size(); j++)
		{
			sumPoint += centers[i][j].confidence * centers[i][j].location;
			normalizer += centers[i][j].confidence;
		}
		sumPoint *= (1. / normalizer);
		cv::KeyPoint kpt(sumPoint, (float)(centers[i][centers[i].size() / 2].radius) * 2.0f);
		keypoints.push_back(kpt);
	}

	// if keypoints were found...
	if (!keypoints.empty())
	{
		for (int i = 0; i < keypoints.size(); i++)
		{
			float x = keypoints[i].pt.x;
			float y = keypoints[i].pt.y;
			blobs.push_back(cv::Point2f(x, y));
		}
	}

#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"BlobFinder::DetectBlobs: Done.\n");
#endif
}

void BlobFinder::MatchBlobs(
	_In_ std::vector<cv::Point2f>& lfPointCandidates,
	_In_ std::vector<cv::Point2f>& rfPointCandidates,
	_Out_ std::vector<cv::Point2f>& lfPointsSorted,
	_Out_ std::vector<cv::Point2f>& rfPointsSorted)
{
	lfPointsSorted.clear();
	rfPointsSorted.clear();

	std::vector<cv::Point3f> lfEpilines, rfEpilines;

	// prune blobs
	cv::computeCorrespondEpilines(lfPointCandidates, 1, m_fMat, rfEpilines);
	cv::computeCorrespondEpilines(rfPointCandidates, 2, m_fMat, lfEpilines);

	if (lfPointCandidates.size() < rfPointCandidates.size())
	{
		PruneBlobs(lfPointCandidates, rfPointCandidates, lfEpilines, rfEpilines,
			lfPointsSorted, rfPointsSorted);
	}
	else
	{
		PruneBlobs(rfPointCandidates, lfPointCandidates, rfEpilines, lfEpilines,
			rfPointsSorted, lfPointsSorted);
	}
}

void BlobFinder::PruneBlobs(
	_In_ std::vector<cv::Point2f> targetBlobs,
	_In_ std::vector<cv::Point2f> sourceBlobs,
	_In_ std::vector<cv::Point3f> targetEpilines,
	_In_ std::vector<cv::Point3f> sourceEpilines,
	_Out_ std::vector<cv::Point2f>& targetPointsSorted,
	_Out_ std::vector<cv::Point2f>& sourcePointsSorted)
{
	targetPointsSorted.clear();
	sourcePointsSorted.clear();

	// sorting points to make sure they correspond
	const size_t N = sourceBlobs.size(); // # Blobs in source
	const size_t M = targetBlobs.size(); // # Blobs in target

	std::vector<int> assignment(N, -1);
	std::vector<float> cost_matrix(N * M);

	// create cost matrix
	// iterate over all blobs in left frame and intersect with the epilines from right frame
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < M; j++)
		{
			cv::Point2f sourceBlob = sourceBlobs[i];
			cv::Point3f sourceLine = sourceEpilines[j];
			float error = abs(sourceLine.x * sourceBlob.x +
				sourceLine.y * sourceBlob.y +
				sourceLine.z);
			if (error > 50.0f)
			{
				error = 1000.0f;
			}
			cost_matrix[i + j * N] = error;
		}
	}
	// solve assignment problem -> for each blob in left frame, 
	// find the closest matching point in right frame!
	m_solver.Solve(cost_matrix, N, M, assignment);

	// clean assignment from pairs with large distance
	for (size_t i = 0; i < assignment.size(); i++)
	{
		if (assignment[i] != -1)
		{
			if (cost_matrix[i + assignment[i] * N] > 5)
			{
				assignment[i] = -1;
			}
		}
	}

	// sort the points -> target to source
	for (size_t i = 0; i < assignment.size(); i++)
	{
		if (assignment[i] != -1)
		{
			cv::Point2f targetPoint(
				targetBlobs[assignment[i]].x,
				targetBlobs[assignment[i]].y);

			sourcePointsSorted.push_back(sourceBlobs[i]);
			targetPointsSorted.push_back(targetPoint);
		}
	}
}


void BlobFinder::FilterBlobs(
	_In_ const cv::Mat& threshImage,
	_Out_ std::vector<Center>& centers)
{
	centers.clear();
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(threshImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"BlobFinder::FilterBlobs: Filtering contours...\n");
#endif
	for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
	{
		Center center;
		center.confidence = 1;
		cv::Moments moms = cv::moments(contours[contourIdx]);
		if (m_params.filterByArea)
		{
			double area = moms.m00;
			if (area < m_params.minArea || area >= m_params.maxArea)
				continue;
		}

		if (m_params.filterByCircularity)
		{
			double area = moms.m00;
			double perimeter = arcLength(contours[contourIdx], true);
			double ratio = 4 * CV_PI * area / (perimeter * perimeter);
			if (ratio < m_params.minCircularity || ratio >= m_params.maxCircularity)
				continue;
		}

		if (m_params.filterByInertia)
		{
			double denominator = std::sqrt(std::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
			const double eps = 1e-2;
			double ratio;
			if (denominator > eps)
			{
				double cosmin = (moms.mu20 - moms.mu02) / denominator;
				double sinmin = 2 * moms.mu11 / denominator;
				double cosmax = -cosmin;
				double sinmax = -sinmin;

				double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
				double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
				ratio = imin / imax;
			}
			else
			{
				ratio = 1;
			}

			if (ratio < m_params.minInertiaRatio || ratio >= m_params.maxInertiaRatio)
				continue;

			center.confidence = ratio * ratio;
		}

		if (m_params.filterByConvexity)
		{
			std::vector<cv::Point> hull;
			convexHull(contours[contourIdx], hull);
			double area = moms.m00;
			double hullArea = contourArea(hull);
			if (fabs(hullArea) < DBL_EPSILON)
				continue;
			double ratio = area / hullArea;
			if (ratio < m_params.minConvexity || ratio >= m_params.maxConvexity)
				continue;
		}

		if (moms.m00 == 0.0)
			continue;
		center.location = cv::Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

		if (m_params.filterByColor)
		{
			if (threshImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != m_params.blobColor)
				continue;
		}
		//compute blob radius
		{
			std::vector<double> dists;
			for (size_t pointIdx = 0;
				pointIdx < contours[contourIdx].size();
				pointIdx++)
			{
				cv::Point2d pt = contours[contourIdx][pointIdx];
				dists.push_back(norm(center.location - pt));
			}
			std::sort(dists.begin(), dists.end());
			center.radius =
				(dists[(dists.size() - 1) / 2] +
					dists[dists.size() / 2]) / 2.;
		}
		centers.push_back(center);
	}
#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"BlobFinder::FilterBlobs: Done.\n");
#endif
}
