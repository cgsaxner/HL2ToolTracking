#include "pch.h"

Track::Track(int id, cv::Point2f point) :
	m_id(id),
	m_point(point)
{
	skipped_frames = 0;
}

void Track::Initialize(cv::Point2f point, uint64_t timestamp)
{
	m_point = point;
	m_timestamp = timestamp;
	kf = LinearKalmanFilter(point, 1);
	initialized = true;
}

void Track::Update(cv::Point2f point, uint64_t timestamp)
{
	m_point = point;
	float delta = (float)(timestamp - m_timestamp) * 1e-7f;
	kf.PredictAndCorrect(m_point, delta);
	m_timestamp = timestamp;
}