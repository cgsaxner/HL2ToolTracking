#include "pch.h"

#define DBG_ENABLE_INFORMATIONAL_LOGGING 1
#define DBG_ENABLE_VERBOSE_LOGGING 0
#define DBG_ENABLE_ERROR_LOGGING 1

using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Foundation::Numerics;

InstrumentTracker::InstrumentTracker(
	std::shared_ptr<MultiFrameBuffer> multiFrameBuffer,
	const GUID& guid,
	const winrt::Windows::Perception::Spatial::SpatialCoordinateSystem& coordSystem,
	winrt::HL2ToolTrackingWRC::implementation::HL2ToolTracking* pToolTrackingPlugin)
{
	m_worldCoordSystem = coordSystem;
	SetLocator(guid);

	m_pMultiFrameBuffer = multiFrameBuffer;
	m_pToolTrackingPlugin = pToolTrackingPlugin;

	m_BlobFinder = BlobFinder(fMatHL2);
	m_RigidBodyFitter = RigidBodyFitter(fMatHL2, lfProjHL2, rfProjHL2);
	m_Marker = Marker(pointsModel);

	isRunning = true;
}

InstrumentTracker::~InstrumentTracker()
{
	m_fExit = true;
	if (m_instrumentPositionUpdateThread.joinable())
	{
		m_instrumentPositionUpdateThread.join();
	}
}

void InstrumentTracker::Stop()
{
	m_fExit = true;

	if (m_instrumentPositionUpdateThread.joinable())
	{
		m_instrumentPositionUpdateThread.join();
	}

	isRunning = false;
}


void InstrumentTracker::Start()
{
	m_fExit = false;

	m_instrumentPositionUpdateThread = std::thread(Update, this);

	isRunning = true;
}

void InstrumentTracker::SetLocator(const GUID& guid)
{
	m_locator = Preview::SpatialGraphInteropPreview::CreateLocatorForNode(guid);
}

bool InstrumentTracker::GetUpdateFrame(
	_In_ std::shared_ptr<IResearchModeSensorFrame> frame,
	_Inout_ UpdateFrame& updateFrame)
{
	ResearchModeSensorResolution resolution;
	IResearchModeSensorVLCFrame* pFrameTmp = nullptr;
	const BYTE* pImgTmp = nullptr;
	std::vector<BYTE> imgData;
	size_t outBufferCount;
	cv::Mat image;

	std::vector<cv::Point2f> blobsDist;

	auto perceptionTimestamp = PerceptionTimestampHelper::FromSystemRelativeTargetTime(
		HundredsOfNanoseconds(checkAndConvertUnsigned(updateFrame.timestamp.HostTicks)));

	// locate the rig wrt the world coordinate system
	auto location = m_locator.TryLocateAtTimestamp(perceptionTimestamp, m_worldCoordSystem);
	if (!location)
	{
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugStringW(L"InstrumentTracking::GetUpdateFrame: Failed to locate frame!\n");
#endif
		return false;
	}

	frame->GetResolution(&resolution);
	HRESULT hr = frame->QueryInterface(IID_PPV_ARGS(&pFrameTmp));

	if (!pFrameTmp || !SUCCEEDED(hr))
	{
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugStringW(L"InstrumentTracking::GetUpdateFrame: Failed to grab frame data!\n");
#endif
		return false;
	}

	std::shared_ptr<IResearchModeSensorVLCFrame> pVLCFrame(
		pFrameTmp, [](IResearchModeSensorVLCFrame* sf) { sf->Release(); });

	hr = pVLCFrame->GetBuffer(&pImgTmp, &outBufferCount);

	if (!pImgTmp || !SUCCEEDED(hr))
	{
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugStringW(L"InstrumentTracking::GetUpdateFrame: Failed to get the buffer!\n");
#endif
		return false;
	}

#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"InstrumentTracking::GetUpdateFrame Converting images to cv::Mat...\n");
#endif

	imgData.reserve(outBufferCount);
	imgData.insert(imgData.end(), pImgTmp, pImgTmp + outBufferCount);

	image = cv::Mat(resolution.Height, resolution.Width, CV_8UC1, imgData.data());
	if (!image.data)
	{
		return false;
	}

	m_BlobFinder.DetectBlobs(image, blobsDist);

	if (blobsDist.empty())
	{
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugStringW(L"InstrumentTracking::GetUpdateFrame: No blobs found! returning...\n");
#endif
		return false;
	}

	const float4x4 rig2worldTransform =
		make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position());

	cv::Mat rigToWorldCv = (cv::Mat_<float>(4, 4) <<
		rig2worldTransform.m11, rig2worldTransform.m12, rig2worldTransform.m13, rig2worldTransform.m14,
		rig2worldTransform.m21, rig2worldTransform.m22, rig2worldTransform.m23, rig2worldTransform.m24,
		rig2worldTransform.m31, rig2worldTransform.m32, rig2worldTransform.m33, rig2worldTransform.m34,
		rig2worldTransform.m41, rig2worldTransform.m42, rig2worldTransform.m43, rig2worldTransform.m44);

#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"InstrumentTracking::GetUpdateFrame: Undistort...\n");
#endif
	if (updateFrame.sensorType == ResearchModeSensorType::LEFT_FRONT)
	{
		updateFrame.camToWorldTransform = rigToWorldCv.t();

		cv::undistortPoints(blobsDist, updateFrame.blobs,
			lfIntrHL2, lfDistHL2, cv::noArray(), lfIntrHL2);
	}
	else if (updateFrame.sensorType == ResearchModeSensorType::RIGHT_FRONT)
	{
		updateFrame.camToWorldTransform = rigToWorldCv.t() * rfToLf;

		cv::undistortPoints(blobsDist, updateFrame.blobs,
			rfIntrHL2, rfDistHL2, cv::noArray(), rfIntrHL2);
	}
	else
	{
		return false;
	}
	return true;
}

void InstrumentTracker::Update(
	InstrumentTracker* pTracker
)
{
	while (!pTracker->m_fExit)
	{
		std::priority_queue<UpdateFrame, std::vector<UpdateFrame>, FrameCmp> updateFramesScaat;
		std::map<ResearchModeSensorType, UpdateFrame> updateFramesTriang;

		// first, try to get a matching frame pair
		long long commonTime;
		commonTime = pTracker->m_pMultiFrameBuffer->GetTimestampForSensorPair(
			ResearchModeSensorType::LEFT_FRONT, ResearchModeSensorType::RIGHT_FRONT, pTracker->c_timeTolerance);

#if DBG_ENABLE_VERBOSE_LOGGING
		wchar_t msgBuffer[500];
		swprintf_s(msgBuffer, L"InstrumentTracking::Update: common time %llu.\n",
			commonTime);
		OutputDebugStringW(msgBuffer);
#endif

		pTracker->m_fProcessingInProgress = true;

		if (commonTime != 0 && commonTime > pTracker->m_lastCommonTimestamp)
		{
			ResearchModeSensorTimestamp lfTime, rfTime;
			std::shared_ptr<IResearchModeSensorFrame> lfFrame(nullptr);
			std::shared_ptr<IResearchModeSensorFrame> rfFrame(nullptr);
			{
				std::lock_guard<std::mutex> lock(pTracker->_framesMutex);
				lfFrame = pTracker->m_pMultiFrameBuffer->GetFrameForTime(
					ResearchModeSensorType::LEFT_FRONT, commonTime, pTracker->c_timeTolerance, lfTime);

				rfFrame = pTracker->m_pMultiFrameBuffer->GetFrameForTime(
					ResearchModeSensorType::RIGHT_FRONT, commonTime, pTracker->c_timeTolerance, rfTime);
			}
			if (lfFrame)
			{
				UpdateFrame lfUpdateFrame;
				lfUpdateFrame.sensorType = ResearchModeSensorType::LEFT_FRONT;
				lfUpdateFrame.timestamp = lfTime;
				auto sucess = pTracker->GetUpdateFrame(lfFrame, lfUpdateFrame);
				if (sucess)
				{
					updateFramesTriang[lfUpdateFrame.sensorType] = lfUpdateFrame;
					updateFramesScaat.push(lfUpdateFrame);
				}
			}
			if (rfFrame)
			{
				UpdateFrame rfUpdateFrame;
				rfUpdateFrame.sensorType = ResearchModeSensorType::RIGHT_FRONT;
				rfUpdateFrame.timestamp = rfTime;
				auto sucess = pTracker->GetUpdateFrame(rfFrame, rfUpdateFrame);
				if (sucess)
				{
					updateFramesTriang[rfUpdateFrame.sensorType] = rfUpdateFrame;
					updateFramesScaat.push(rfUpdateFrame);
				}
			}
		}
		else
		{
#if DBG_ENABLE_VERBOSE_LOGGING
			OutputDebugStringW(L"InstrumentTracking::Update: No matching frame pair found or timestamp already processed.\n");
#endif
			ResearchModeSensorType sensorType;
			ResearchModeSensorTimestamp timestamp;
			std::vector<cv::Point2f> blobs;
			cv::Mat camToWorldTransform;
			std::shared_ptr<IResearchModeSensorFrame> frame(nullptr);
			{
				std::lock_guard<std::mutex> lock(pTracker->_framesMutex);
				frame = pTracker->m_pMultiFrameBuffer->GetLatestFrame(
					ResearchModeSensorType::LEFT_FRONT, ResearchModeSensorType::RIGHT_FRONT,
					sensorType, timestamp);

				if (frame)
				{
					UpdateFrame updateFrame;
					updateFrame.sensorType = sensorType;
					updateFrame.timestamp = timestamp;

					auto sucess = pTracker->GetUpdateFrame(frame, updateFrame);
					if (sucess)
					{
						updateFramesScaat.push(updateFrame);
					}
				}
			}
		}

		if (updateFramesTriang.size() == 2) // stereo triangulation
		{
#if DBG_ENABLE_VERBOSE_LOGGING
			OutputDebugStringW(L"InstrumentTracker::OnUpdate: Stereo Triangulation path...\n");
#endif
			if (updateFramesTriang[ResearchModeSensorType::LEFT_FRONT].blobs.size() >= 3 &&
				updateFramesTriang[ResearchModeSensorType::LEFT_FRONT].blobs.size() >= 3)
			{
				std::vector<cv::Point2f> lfBlobsSorted, rfBlobsSorted;
				pTracker->m_BlobFinder.MatchBlobs(
					updateFramesTriang[ResearchModeSensorType::LEFT_FRONT].blobs,
					updateFramesTriang[ResearchModeSensorType::RIGHT_FRONT].blobs,
					lfBlobsSorted, rfBlobsSorted);

				if (lfBlobsSorted.size() >= 3 &&
					rfBlobsSorted.size() >= 3 &&
					lfBlobsSorted.size() == rfBlobsSorted.size())
				{
#if DBG_ENABLE_VERBOSE_LOGGING
					OutputDebugStringW(L"InstrumentTracker::OnUpdate: Enough matching blobs found. Triangulating...\n");
#endif
					cv::Mat lfCameraPoints;
					cv::Mat worldPoints;
					pTracker->m_RigidBodyFitter.Triangulate(lfBlobsSorted, rfBlobsSorted, lfCameraPoints);

					TransformPoints(
						lfCameraPoints.t(),
						updateFramesTriang[ResearchModeSensorType::LEFT_FRONT].camToWorldTransform,
						worldPoints);

					worldPoints = worldPoints.t();

#if DBG_ENABLE_VERBOSE_LOGGING
					memset(msgBuffer, 0, 500);
					swprintf_s(msgBuffer, L"InstrumentTracker::OnUpdate: Done. Triangulated %i points. Rigid Body Fitting...\n",
						worldPoints.rows);
					OutputDebugStringW(msgBuffer);
#endif
					float deviation = 100.0f;
					float max_deviation = 0.005f;
					std::vector<int> assignment = std::vector<int>(pTracker->m_Marker.numPoints, -1);
					cv::Mat modelToWorldTransformTriangulation;

					pTracker->m_RigidBodyFitter.FitRigidBody(worldPoints, pTracker->m_Marker,
						max_deviation, deviation, modelToWorldTransformTriangulation, assignment);

					if (deviation < max_deviation)
					{
						std::vector<Eigen::Vector3d> modelPointsWorldE, modelPointsLfE, modelPointsRfE;
						std::vector<cv::Point2f> lfPixels, rfPixels;
						cv::Mat modelPointsWorldCV;
						TransformPoints(pTracker->m_Marker.markerPoints,
							modelToWorldTransformTriangulation, modelPointsWorldE);
						TransformPoints(pTracker->m_Marker.markerPoints,
							modelToWorldTransformTriangulation, modelPointsWorldCV);
						TransformPoints(modelPointsWorldE,
							updateFramesTriang[ResearchModeSensorType::LEFT_FRONT].camToWorldTransform.inv(),
							modelPointsLfE);

						TransformPoints(modelPointsWorldE,
							updateFramesTriang[ResearchModeSensorType::RIGHT_FRONT].camToWorldTransform.inv(),
							modelPointsRfE);

						ProjectOnRMFrame(pTracker->lfFxHL2, pTracker->lfFyHL2,
							pTracker->lfCxHL2, pTracker->lfCyHL2,
							modelPointsLfE, lfPixels);
						ProjectOnRMFrame(pTracker->rfFxHL2, pTracker->rfFyHL2,
							pTracker->rfCxHL2, pTracker->rfCyHL2,
							modelPointsRfE, rfPixels);

						pTracker->m_Marker.lastLfPos = lfPixels;
						pTracker->m_Marker.lastRfPos = rfPixels;
						pTracker->m_Marker.lastWorldPoints = modelPointsWorldCV;
						pTracker->m_Marker.lastTransform = modelToWorldTransformTriangulation;

						pTracker->m_ScaatEKF = ScaatEKF(
							12, 2, 0, pTracker->lfFxHL2, pTracker->lfFyHL2,
							pTracker->lfCxHL2, pTracker->lfCyHL2,
							pTracker->rfFxHL2, pTracker->rfFyHL2,
							pTracker->rfCxHL2, pTracker->rfCyHL2,
							modelToWorldTransformTriangulation);

						pTracker->m_ScaatEKF.lastTimestamp = commonTime;

						TransformationToQuaternionAndTranslation(modelToWorldTransformTriangulation,
							pTracker->m_pToolTrackingPlugin->m_toolPosition);
					}
					else
					{
#if DBG_ENABLE_VERBOSE_LOGGING
						OutputDebugStringW(L"InstrumentTracker::OnUpdate: Rigid body fitting failed.\n");
#endif
					}
#if DBG_ENABLE_VERBOSE_LOGGING
				}
				else
				{
					OutputDebugStringW(L"InstrumentTracker::OnUpdate: Not enough matching blobs found.\n");
				}
			}
			else
			{
				OutputDebugStringW(L"InstrumentTracker::OnUpdate: Not enough blobs found.\n");
			}
#else
		}
	}
#endif
}

		// SCAAT path
		if (pTracker->m_ScaatEKF.initialized && updateFramesScaat.size() > 0)
		{
#if DBG_ENABLE_VERBOSE_LOGGING
			OutputDebugStringW(L"InstrumentTracker::OnUpdate: Done. SCAAT update loop...\n");
#endif

#if DBG_ENABLE_TIMING_LOGGING
			auto start_scaat = std::chrono::high_resolution_clock::now();
#endif // DBG_ENABLE_TIMING_LOGGING

			Eigen::Matrix4f modelToWorldTransformScaat = Eigen::Matrix4f::Identity();

			while (!updateFramesScaat.empty())
			{
				UpdateFrame updateFrame = updateFramesScaat.top();
				pTracker->m_ScaatEKF.MainUpdateLoop(
					pTracker->m_Marker, (long long)updateFrame.timestamp.HostTicks, updateFrame.blobs,
					updateFrame.sensorType, updateFrame.camToWorldTransform, modelToWorldTransformScaat);

				std::vector<Eigen::Vector3d> modelPointsWorldE, modelPointsCamE;
				cv::Mat modelPointsWorldCV;
				TransformPoints(pTracker->m_Marker.markerPoints,
					modelToWorldTransformScaat, modelPointsWorldE);
				TransformPoints(pTracker->m_Marker.markerPoints,
					modelToWorldTransformScaat, modelPointsWorldCV);
				TransformPoints(modelPointsWorldE,
					updateFrame.camToWorldTransform.inv(), modelPointsCamE);

				if (updateFrame.sensorType == ResearchModeSensorType::LEFT_FRONT)
				{
					std::vector<cv::Point2f> lfPixels;
					ProjectOnRMFrame(pTracker->lfFxHL2, pTracker->lfFyHL2,
						pTracker->lfCxHL2, pTracker->lfCyHL2,
						modelPointsCamE, lfPixels);
					pTracker->m_Marker.lastLfPos = lfPixels;
				}
				else if (updateFrame.sensorType == ResearchModeSensorType::RIGHT_FRONT)
				{
					std::vector<cv::Point2f> rfPixels;
					ProjectOnRMFrame(pTracker->rfFxHL2, pTracker->rfFyHL2,
						pTracker->rfCxHL2, pTracker->rfCyHL2,
						modelPointsCamE, rfPixels);
					pTracker->m_Marker.lastRfPos = rfPixels;
				}
				pTracker->m_Marker.lastWorldPoints = modelPointsWorldCV;
				updateFramesScaat.pop();
			}

			//modelToWorldTransformScaat.row(2) *= -1;

			TransformationToQuaternionAndTranslation(modelToWorldTransformScaat,
				pTracker->m_pToolTrackingPlugin->m_toolPosition);

#if DBG_ENABLE_VERBOSE_LOGGING
			OutputDebugStringW(L"InstrumentTracker::OnUpdate: Scaat sucessful.\n");
#endif
#if DBG_ENABLE_TIMING_LOGGING
			auto end_scaat = std::chrono::high_resolution_clock::now();
			auto diff_scaat = end_scaat - start_scaat;
			auto duration_scaat =
				std::chrono::duration_cast<std::chrono::nanoseconds>(diff_scaat).count() * 1e-6;
			m_scaatTime += duration_scaat;

			memset(msgBuffer, 0, 200);
			swprintf_s(msgBuffer, L"InstrumentTracking::Update: Scaat update and pose estimation took %.3f ms.\n",
				duration_scaat);
			OutputDebugStringW(msgBuffer);
#endif
		}
		pTracker->m_fProcessingInProgress = false;
	}
}
