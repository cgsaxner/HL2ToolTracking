#include "pch.h"


#define DBG_ENABLE_VERBOSE_LOGGING 0
#define DBG_ENABLE_INFO_LOGGING 1
#define DBG_ENABLE_ERROR_LOGGING 1


void MultiFrameBuffer::Send(
    std::shared_ptr<IResearchModeSensorFrame> pSensorFrame,
    ResearchModeSensorType pSensorType)
{
    std::lock_guard<std::mutex> lock(_framesMutex);
    auto& buffer = _frames[pSensorType];
    buffer.push_back(pSensorFrame);
#if DBG_ENABLE_VERBOSE_LOGGING
    OutputDebugStringW(L"MultiFrameBuffer::Send: Frame added.\n");
#endif
    while (buffer.size() > 5)
    {
        buffer.pop_front();
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"MultiFrameBuffer::Send: Frame popped.\n");
#endif
    }
}

long long MultiFrameBuffer::GetTimestampForSensorPair(
    ResearchModeSensorType a,
    ResearchModeSensorType b,
    long long toleranceInHundredsOfNanoseconds)
{
    std::vector<ResearchModeSensorTimestamp> vta;
    std::vector<ResearchModeSensorTimestamp> vtb;

    long long best = 0;
    {
        std::lock_guard<std::mutex> lock(_framesMutex);
        std::deque<std::shared_ptr<IResearchModeSensorFrame>> buffer = _frames[a];
        for (int i = 0; i < _frames[a].size(); i++)
        {
            ResearchModeSensorTimestamp timestamp;
            std::shared_ptr<IResearchModeSensorFrame> frame = _frames[a][i];
            winrt::check_hresult(frame->GetTimeStamp(&timestamp));
            vta.push_back(timestamp);
        }

        buffer = _frames[b];
        for (int i = 0; i < _frames[b].size(); i++)
        {
            ResearchModeSensorTimestamp timestamp;
            std::shared_ptr<IResearchModeSensorFrame> frame = _frames[b][i];
            winrt::check_hresult(frame->GetTimeStamp(&timestamp));
            vtb.push_back(timestamp);
        }
    }


    for (ResearchModeSensorTimestamp const& ta : vta)
    {
        for (ResearchModeSensorTimestamp const& tb : vtb)
        {
            long long delta = tb.HostTicks - ta.HostTicks;
            if (std::abs(delta) < toleranceInHundredsOfNanoseconds)
            {
                if ((long long)ta.HostTicks - best > 0)
                {
                    best = ta.HostTicks;
                }
            }
        }
    }
    return best;
}

std::shared_ptr<IResearchModeSensorFrame> MultiFrameBuffer::GetFrameForTime(
    ResearchModeSensorType sensor,
    long long Timestamp,
    long long toleranceInHundredsOfNanoseconds,
    ResearchModeSensorTimestamp& outTimestamp)
{
    {
        std::lock_guard<std::mutex> lock(_framesMutex);
        std::deque<std::shared_ptr<IResearchModeSensorFrame>> buffer = _frames[sensor];
        for (auto& f : buffer)
        {
            ResearchModeSensorTimestamp timestamp;
            winrt::check_hresult(f->GetTimeStamp(&timestamp));
            long long delta = std::abs(Timestamp - (long long)timestamp.HostTicks);

            if (delta < toleranceInHundredsOfNanoseconds)
            {
                outTimestamp = timestamp;
                return f;
            }
        }
    }
    return nullptr;
}

std::shared_ptr<IResearchModeSensorFrame> MultiFrameBuffer::GetLatestFrame(
    ResearchModeSensorType a,
    ResearchModeSensorType b,
    ResearchModeSensorType& return_type,
    ResearchModeSensorTimestamp& timestamp)
{
    {
        std::lock_guard<std::mutex> lock(_framesMutex);
        auto& buffera = _frames[a];
        auto& bufferb = _frames[b];
        if (buffera.empty() && bufferb.empty())
        {
            return  nullptr;
        }
        else if (buffera.empty())
        {
            return bufferb.back();
        }
        else if (bufferb.empty())
        {
            return buffera.back();
        }
        ResearchModeSensorTimestamp timestampa, timestampb;
        winrt::check_hresult(buffera.back()->GetTimeStamp(&timestampa));
        winrt::check_hresult(bufferb.back()->GetTimeStamp(&timestampb));

        if ((long long)timestampa.HostTicks < (long long)timestampb.HostTicks)
        {
            return_type = a;
            timestamp = timestampa;
            return buffera.back();
        }
        else
        {
            return_type = b;
            timestamp = timestampb;
            return bufferb.back();
        }
    }
}

void MultiFrameBuffer::Release()
{
    std::lock_guard<std::mutex> lock(_framesMutex);
    _frames.clear();
}