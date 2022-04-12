#pragma once
class Marker
{
public:
	Marker(cv::Mat points);
	Marker() = default;
	~Marker() = default;

	int numPoints;
	cv::Mat markerPoints;
	cv::Mat distanceMatrix;
	float minDistance;
	float maxDistance;

	cv::Mat lastWorldPoints;
	cv::Mat lastTransform;
	std::vector<cv::Point2f> lastLfPos;
	std::vector<cv::Point2f> lastRfPos;
};


