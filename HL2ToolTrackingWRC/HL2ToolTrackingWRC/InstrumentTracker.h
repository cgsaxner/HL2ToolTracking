#pragma once
#include "HL2ToolTracking.h"

struct UpdateFrame
{
	ResearchModeSensorTimestamp timestamp;
	ResearchModeSensorType sensorType;
	std::vector<cv::Point2f> blobs;
	cv::Mat camToWorldTransform;
};

class FrameCmp
{
public:
	bool operator() (const UpdateFrame& lhs, const UpdateFrame& rhs) const
	{
		return (lhs.timestamp.HostTicks > rhs.timestamp.HostTicks);
	}
};


class InstrumentTracker
{
public:
	InstrumentTracker() = default;
	InstrumentTracker(
		std::shared_ptr<MultiFrameBuffer> multiFrameBuffer,
		const GUID& guid,
		const winrt::Windows::Perception::Spatial::SpatialCoordinateSystem& coordSystem,
		winrt::HL2ToolTrackingWRC::implementation::HL2ToolTracking* pToolTrackingPlugin);

	~InstrumentTracker();

	void Start();
	void Stop();

	static void Update(
		InstrumentTracker* pTracker
	);

	bool isRunning = false;

private:
	bool GetUpdateFrame(
		_In_ std::shared_ptr<IResearchModeSensorFrame> frame,
		_Inout_ UpdateFrame& updateFrame);

	void SetLocator(const GUID& guid);

private:
	winrt::HL2ToolTrackingWRC::implementation::HL2ToolTracking* m_pToolTrackingPlugin;
	
	// spatial locators
	winrt::Windows::Perception::Spatial::SpatialLocator m_locator = nullptr;
	winrt::Windows::Perception::Spatial::SpatialCoordinateSystem m_worldCoordSystem = nullptr;
	winrt::Windows::Perception::Spatial::SpatialLocation m_location = nullptr;

	std::shared_ptr<MultiFrameBuffer> m_pMultiFrameBuffer;
	BlobFinder m_BlobFinder;
	RigidBodyFitter m_RigidBodyFitter;
	Marker m_Marker;
	ScaatEKF m_ScaatEKF;

	bool m_fExit = true;
	std::thread m_instrumentPositionUpdateThread;
	std::mutex _framesMutex;
	std::atomic<bool> m_fProcessingInProgress = false;

	long long m_lastCommonTimestamp = 0;
	const long long c_timeTolerance = 10000; // hundreds of nanoseconds

	// definition of the model points
	cv::Mat pointsModel = (cv::Mat_<float>(5, 3) <<
		2.57336e-7f, 2.42391e-6f, 3.36463e-6f,
		0.0442368f, 0.0349529f, 0.0726091f,
		-0.0936104f, 0.0113667f, 0.0732692f,
		0.000176568f, -0.000130322f, 0.108399f,
		-0.00748328f, 0.0856192f, 0.0642646f);

	// definition of camera parameters
	float lfFxHL2 = 362.76819f;
	float lfFyHL2 = 363.63627f;
	float lfCxHL2 = 316.65636f;
	float lfCyHL2 = 231.20529f;
	cv::Mat lfDistHL2 = (cv::Mat_<float>(1, 5) <<
		-0.01818f, 0.01685f, -0.00494f, 0.00170f, 0.0f);

	float rfFxHL2 = 364.68835f;
	float rfFyHL2 = 364.63003f;
	float rfCxHL2 = 321.11678f;
	float rfCyHL2 = 233.32648f;
	cv::Mat rfDistHL2 = (cv::Mat_<float>(1, 5) <<
		-0.02002f, 0.01935f, -0.00306f, -0.00216f, 0.0f);

	cv::Mat rfToLf = (cv::Mat_<float>(4, 4) <<
		-0.999988716987f, 0.004749329171f, 0.000098853715f, -0.001400949679f,
		-0.004750308829f, -0.999856389093f, -0.016267629061f, -0.098276565847f,
		0.000021579193f, -0.016267915098f, 0.999867668481f, 0.002508806869f,
		0.0f, 0.0f, 0.0f, 1.0f);

	cv::Mat lfIntrHL2 = (cv::Mat_<float>(3, 3) <<
		lfFxHL2, 0.0f, lfCxHL2,
		0.0f, lfFyHL2, lfCyHL2,
		0.0f, 0.0f, 1.0f);

	cv::Mat rfIntrHL2 = (cv::Mat_<float>(3, 3) <<
		rfFxHL2, 0.0f, rfCxHL2,
		0.0f, rfFyHL2, rfCyHL2,
		0.0f, 0.0f, 1.0f);

	cv::Mat lfToRf = rfToLf.inv();

	cv::Mat eMatHL2 = (cv::Mat_<float>(3, 3) <<
		0.0f, -rfToLf.at<float>(2, 3), rfToLf.at<float>(1, 3),
		rfToLf.at<float>(2, 3), 0.0f, -rfToLf.at<float>(0, 3),
		-rfToLf.at<float>(1, 3), rfToLf.at<float>(0, 3), 0.0f) *
		rfToLf(cv::Rect(0, 0, 3, 3));

	cv::Mat fMatHL2 = (rfIntrHL2.t().inv()) *
		eMatHL2 * lfIntrHL2.inv();

	cv::Mat rfProjHL2 = rfIntrHL2 *
		rfToLf(cv::Rect(0, 0, 4, 3));

	cv::Mat lfProjHL2 = lfIntrHL2 *
		cv::Mat::eye(4, 4, CV_32F)(cv::Rect(0, 0, 4, 3));
};

