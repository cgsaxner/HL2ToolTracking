#include "pch.h"
#include "HL2ToolTracking.h"
#include "HL2ToolTracking.g.cpp"

extern "C"
HMODULE LoadLibraryA(
	LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;
static ResearchModeSensorConsent imuAccessCheck;
static HANDLE imuConsentGiven;

namespace winrt::HL2ToolTrackingWRC::implementation
{
	void HL2ToolTracking::Initialize(
		winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& refCoord)
	{
		m_worldOrigin = refCoord;
		InitializeResearchModeSensors();
		InitializeResearchModeProcessing();
	}

	void HL2ToolTracking::StartTracking()
	{
		// start the LF and RF processing
		m_pLFProcessor->Start();
		m_pRFProcessor->Start();

		// start the instrument tracker
		m_pInstrumentTracker->Start();
	}

	void HL2ToolTracking::StopTracking()
	{
		// stop the processors from sending frames to the multi frame buffer
		if (m_pLFProcessor && m_pLFProcessor->isRunning)
		{
			m_pLFProcessor->Stop();
		}
		if (m_pRFProcessor && m_pRFProcessor->isRunning)
		{
			m_pRFProcessor->Stop();
		}

		// stop the instrument tracker
		if (m_pInstrumentTracker && m_pInstrumentTracker->isRunning)
		{
			m_pInstrumentTracker->Stop();
		}

		// release the frames in the frame buffer
		if (m_pMultiFrameBuffer)
		{
			m_pMultiFrameBuffer->Release();
		}
	}

	com_array<float> HL2ToolTracking::ToolPosition()
	{
		return com_array<float>(m_toolPosition);
	}

	void HL2ToolTracking::InitializeResearchModeSensors()
	{
		//HRESULT hr = S_OK;
		size_t sensorCount = 0;
		camConsentGiven = CreateEvent(nullptr, true, false, nullptr);

		// Load research mode library
		HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
		if (hrResearchMode)
		{
#if DBG_ENABLE_VERBOSE_LOGGING
			OutputDebugStringW(L"Image2FaceHoloLens2::InitializeSensors: Creating sensor device...\n");
#endif
			// create the research mode sensor device
			typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
			PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>
				(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
			if (pfnCreate)
			{
				winrt::check_hresult(pfnCreate(&m_pSensorDevice));
			}
			else
			{
				winrt::check_hresult(E_INVALIDARG);
			}
		}

		// manage consent
		winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
		winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(CamAccessOnComplete));
		winrt::check_hresult(m_pSensorDeviceConsent->RequestIMUAccessAsync(ImuAccessOnComplete));

		m_pSensorDevice->DisableEyeSelection();

		winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
		m_sensorDescriptors.resize(sensorCount);

		winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(),
			m_sensorDescriptors.size(), &sensorCount));

		for (const auto& sensorDescriptor : m_sensorDescriptors)
		{
			wchar_t msgBuffer[200];
			if (sensorDescriptor.sensorType == LEFT_FRONT)
			{
				winrt::check_hresult(m_pSensorDevice->GetSensor(
					sensorDescriptor.sensorType, &m_pLFCameraSensor));
				swprintf_s(msgBuffer, L"Image2FaceHoloLens2::InitializeSensors: Sensor %ls\n", 
					m_pLFCameraSensor->GetFriendlyName());
				OutputDebugStringW(msgBuffer);
			}

			if (sensorDescriptor.sensorType == RIGHT_FRONT)
			{
				winrt::check_hresult(m_pSensorDevice->GetSensor(
					sensorDescriptor.sensorType, &m_pRFCameraSensor));
				swprintf_s(msgBuffer, L"Image2FaceHoloLens2::InitializeSensors: Sensor %ls\n",
					m_pRFCameraSensor->GetFriendlyName());
				OutputDebugStringW(msgBuffer);
			}
		}
		OutputDebugStringW(L"Image2FaceHoloLens2::InitializeSensors: Done.\n");
		return;
	}

	void HL2ToolTracking::InitializeResearchModeProcessing()
	{
		// Get RigNode id which will be used to initialize
		// the spatial locators for camera readers objects
		// Note: LF-VLC is the RigNode here
		GUID guid;
		GetRigNodeId(guid);

		// initialize the multi frame buffer
		auto multiFrameBuffer = std::make_shared<MultiFrameBuffer>();
		m_pMultiFrameBuffer = multiFrameBuffer;

		// initialize the instrument tracker
		m_pInstrumentTracker = std::make_unique<InstrumentTracker>(
			m_pMultiFrameBuffer, guid, m_worldOrigin, this);

		// initialize the processors
		if (m_pLFCameraSensor)
		{
			auto lfProcessor = std::make_shared<ResearchModeFrameProcessor>(
				m_pLFCameraSensor, camConsentGiven, &camAccessCheck,
				0, m_pMultiFrameBuffer);

			m_pLFProcessor = lfProcessor;
		}
		if (m_pRFCameraSensor)
		{
			auto rfProcessor = std::make_shared<ResearchModeFrameProcessor>(
				m_pRFCameraSensor, camConsentGiven, &camAccessCheck,
				0, m_pMultiFrameBuffer);

			m_pRFProcessor = rfProcessor;
		}
	}

	void HL2ToolTracking::CamAccessOnComplete(ResearchModeSensorConsent consent)
	{
		camAccessCheck = consent;
		SetEvent(camConsentGiven);
	}

	void HL2ToolTracking::ImuAccessOnComplete(ResearchModeSensorConsent consent)
	{
		imuAccessCheck = consent;
		SetEvent(imuConsentGiven);
	}

	void HL2ToolTracking::DisableSensors()
	{
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugString(L"Image2FaceHoloLens2::DisableSensors: Disabling sensors...\n");
#endif // DBG_ENABLE_VERBOSE_LOGGING
		if (m_pLFCameraSensor)
		{
			m_pLFCameraSensor->Release();
		}
		if (m_pRFCameraSensor)
		{
			m_pRFCameraSensor->Release();
		}
		if (m_pSensorDevice)
		{
			m_pSensorDevice->EnableEyeSelection();
			m_pSensorDevice->Release();
		}
		if (m_pSensorDeviceConsent)
		{
			m_pSensorDeviceConsent->Release();
		}
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugString(L"Image2FaceHoloLens2::DisableSensors: Done.\n");
#endif // DBG_ENABLE_VERBOSE_LOGGING
	}

	void HL2ToolTracking::GetRigNodeId(GUID& outGuid)
	{
		IResearchModeSensorDevicePerception* pSensorDevicePerception;
		winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
		winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&outGuid));
		pSensorDevicePerception->Release();
	}
}
