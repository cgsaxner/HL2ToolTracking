#pragma once
#include "HL2ToolTracking.g.h"

// forward declarations
class InstrumentTracker;

namespace winrt::HL2ToolTrackingWRC::implementation
{
    struct HL2ToolTracking : HL2ToolTrackingT<HL2ToolTracking>
    {
    public:
        HL2ToolTracking() = default;

        void Initialize(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& refCoord);

        void StartTracking();

        void StopTracking();

        com_array<float> ToolPosition();

        std::vector<float> m_toolPosition = std::vector<float>(7, 0.0f);

    private:
        void InitializeResearchModeSensors();

        void InitializeResearchModeProcessing();

        void DisableSensors();

        void GetRigNodeId(GUID& outGuid);

        static void CamAccessOnComplete(ResearchModeSensorConsent consent);
        static void ImuAccessOnComplete(ResearchModeSensorConsent consent);

    private:
        Windows::Perception::Spatial::SpatialCoordinateSystem m_worldOrigin = nullptr;

        IResearchModeSensorDevice* m_pSensorDevice;
        IResearchModeSensorDeviceConsent* m_pSensorDeviceConsent;
        std::vector<ResearchModeSensorDescriptor> m_sensorDescriptors;

        IResearchModeSensor* m_pLFCameraSensor = nullptr;
        IResearchModeSensor* m_pRFCameraSensor = nullptr;

        std::shared_ptr<ResearchModeFrameProcessor> m_pLFProcessor = nullptr;
        std::shared_ptr<ResearchModeFrameProcessor> m_pRFProcessor = nullptr;

        std::shared_ptr<MultiFrameBuffer> m_pMultiFrameBuffer = nullptr;
        std::shared_ptr<InstrumentTracker> m_pInstrumentTracker;
    };
}

namespace winrt::HL2ToolTrackingWRC::factory_implementation
{
    struct HL2ToolTracking : HL2ToolTrackingT<HL2ToolTracking, implementation::HL2ToolTracking>
    {
    };
}
