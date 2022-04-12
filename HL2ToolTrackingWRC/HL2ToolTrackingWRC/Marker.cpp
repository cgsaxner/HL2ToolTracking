#include "pch.h"

Marker::Marker(cv::Mat points)
{
	markerPoints = points.t();
	numPoints = markerPoints.cols;
	lastLfPos = std::vector<cv::Point2f>(numPoints, cv::Point2f(-1, -1));
	lastRfPos = std::vector<cv::Point2f>(numPoints, cv::Point2f(-1, -1));

	// get edge lengths of Template
	distanceMatrix = cv::Mat::zeros(numPoints, numPoints, CV_32F);
	float edges_template_max = 0;
	float edges_template_min = std::numeric_limits<float>::infinity();
	for (int a = 0; a < numPoints; a++)
	{
		// compute complete matrix
		for (int b = 0; b < numPoints; b++)
		{
			float dist = (float)norm(markerPoints.col(a) - markerPoints.col(b));
			distanceMatrix.at<float>(a, b) = dist;
			if (dist > edges_template_max)
				edges_template_max = dist;
			if ((a != b) && (dist < edges_template_min))
				edges_template_min = dist;
		}
	}
	minDistance = edges_template_min;
	maxDistance = edges_template_max;
}
