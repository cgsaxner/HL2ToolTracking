#pragma once
class Track
{
public:
	Track(int id, cv::Point2f pix);
	~Track() = default;

	LinearKalmanFilter kf;
	bool initialized = false;
	int skipped_frames;
	cv::Point2f m_point;

	void Update(cv::Point2f point, uint64_t timestamp);
	void Initialize(cv::Point2f point, uint64_t timestamp);

private:
	int m_id;
	uint64_t m_timestamp;
};

