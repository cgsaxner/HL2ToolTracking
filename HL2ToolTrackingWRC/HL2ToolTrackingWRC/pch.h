#pragma once
#include <unknwn.h>
#include <winrt/base.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Networking.Sockets.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Graphics.Imaging.h>

#include <stdio.h>
#include <wchar.h>
#include <comdef.h>
#include <MemoryBuffer.h>
#include <ppltasks.h>
#include <shared_mutex>
#include <deque>
#include <queue>
#include <codecvt>
#include <clocale>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen>

#include "ResearchModeApi.h"
#include "IResearchModeFrameSink.h"
#include "ResearchModeFrameProcessor.h"
#include "MultiFrameBuffer.h"
#include "HungarianAlg.h"
#include "TimeConverter.h"
#include "TransformationUtils.h"
#include "BlobFinder.h"
#include "Marker.h"
#include "RigidBodyFitter.h"
#include "ScaatEKF.h"
#include "InstrumentTracker.h"


