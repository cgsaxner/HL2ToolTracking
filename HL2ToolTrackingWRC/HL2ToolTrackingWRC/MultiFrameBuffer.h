#pragma once
class MultiFrameBuffer : public IResearchModeFrameSink
{
public:
    void Send(
        std::shared_ptr<IResearchModeSensorFrame> frame,
        ResearchModeSensorType pSensorType);

    std::shared_ptr<IResearchModeSensorFrame> GetFrameForTime(
        ResearchModeSensorType sensor,
        long long Timestamp,
        long long toleranceInHundredsOfNanoseconds,
        ResearchModeSensorTimestamp& outTimestamp);

    long long GetTimestampForSensorPair(
        ResearchModeSensorType a,
        ResearchModeSensorType b,
        long long toleranceInHundredsOfNanoseconds);

    std::shared_ptr<IResearchModeSensorFrame> GetLatestFrame(
        ResearchModeSensorType a,
        ResearchModeSensorType b,
        ResearchModeSensorType& return_type,
        ResearchModeSensorTimestamp& timestamp);

    void Release();

private:
    std::map<ResearchModeSensorType, std::deque<std::shared_ptr<IResearchModeSensorFrame>>> _frames;
    std::mutex _framesMutex;
    std::mutex _qMutex;
};

