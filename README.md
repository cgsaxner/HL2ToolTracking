# HL2ToolTracking

This repository contains code for inside-out instrument tracking using the HoloLens 2. For a detailed explanation of the 
and evaluaiton of our method, please see our paper [Inside-Out Instrument Tracking for Surgical Navigation in Augmented 
Reality]. 

**Note:** This repository is still work in progress. 

## Cpp/WinRT Plugin

The Plugin for tool tracking is implemented in 
[Cpp/WinRT](https://docs.microsoft.com/en-us/windows/uwp/cpp-and-winrt-apis/) using the 
[HoloLens 2 Research Mode API](https://github.com/microsoft/HoloLens2ForCV) and [OpenCV]. Building the Plugin generates
 _.dll_ and _.winmd_ files which can be bundled with and consumend by UWP applications, e.g. a Unity app (instructions 
 and a sample will be added to this repo soon)

### Prerequisites
* **Camera Calibration:** The HoloLens 2 needs to be calibrated accurately, e.g. intrinsic parameters of the left-front 
and right-front VLC 
cameras need to be known, as well as the extrinsic transformation matrix between them. We used [this toolbox] for 
obtaining a good calibration.
* **Instrument definition** The tracked instrument needs to be marked with >= 4 spherical markers. We use reflective 
infrared markers. Their 3D configuration on the instrument needs to be known.
* Both camera parameters and marker configuration need to be supplied to the tracking plugin. Currently, they need to 
be manually pasted into the `InstrumentTracker.h` file before building. Support for loading these parameters from a 
config file will be added in the future.

### Dependencies
* [OpenCV] and [OpenCV-contrib] >= 4.3.0 compiled for UWP / ARM64 (see below for instructions)
* [Eigen] >= 3.3.6
* [Visual Studio 2019 or higher](https://developer.microsoft.com/en-us/windows/downloads) (community edition is fine) 
with UWP support
* [Windows SDK 18362 or higher](https://developer.microsoft.com/en-US/windows/downloads/windows-10-sdk)

### Installation
* Open the `HL2ToolTrackingWRC.sln` in VS
* [optional, depending on VS version] Install 
[Cpp/WinRT NuGet package](https://www.nuget.org/packages/Microsoft.Windows.CppWinRT/)
* Configure the project to use dependencies
* Build for Release / ARM64
* This generates Windows Runtime component dll and WinMD files

## Compile OpenCV for UWP / HoloLens
Tested with CMake 3.18.4 GUI, OpenCV 4.3.0 and Visual Studio 2017

1. Download [OpenCV ](https://github.com/opencv/opencv/releases) and [OpenCV contrib](https://github.com/opencv/opencv_contrib) from GitHub
2. Open CMake GUI. Set the source folder to the downloaded release.
3. Add following entries in cmake:
   1.   CMAKE_SYSTEM_NAME=WindowsStore
   2.   CMAKE_SYSTEM_VERSION=10.0
   4.   CMAKE_SYSTEM_PROCESSOR=aarch64
4. Hit configure. Use Visual Studio 15 2017 as generator.
5. Add the path to opencv_contrib/modules to OPENCV_EXTRA_MODULES_PATH
6. Configure again.
7. Disable the following modules from OpenCV (they are not compatible with UWP):
   1. BUILD_opencv_videoio, BUILD_opencv_python_bindings_generator, BUILD_opencv_python_java_generator, BUILD_opencv_ts, BUILD_opencv_hdf, BUILD_opencv_xfeatures2d, BUILD_opencv_js, BUILD_opencv_dnn_objectdetect, BUILD_opencv_gapi, BUILD_opencv_saliency, BUILD_opencv_line_descriptor, BUILD_opencv_bsegm, BUILD_opencv_rgbd, BUILD_opencv_tracking
   2. Optional: disable BUILD_TESTS and BUILD_PERF_TESTS for a faster build.
8. Configure again. Make sure the summary states `Windows RT support: YES`
9. Hit generate. No errors should appear during generation (otherwise, you might need to disable more modules).
10. Open the OpenCV.sln with Visual Studio 2017.
11. Switch configuration to Release, ARM64. Build the BUILD_ALL, then the INSTALL project. There should be no errors.

## Work in progress
- [x] First version of tool tracking plugin code released
- [ ] Unity sample app released
- [ ] Add detailed install instructions
- [ ] Improvements to blob detection released
- [ ] Support for config files

## Citation
If you find our work useful, please consider citing our paper

    @inproceedings{gsaxner2021inside,
      title={Inside-out instrument tracking for surgical navigation in augmented reality},
      author={Gsaxner, Christina and Li, Jianning and Pepe, Antonio and Schmalstieg, Dieter and Egger, Jan},
      booktitle={Proceedings of the 27th ACM Symposium on Virtual Reality Software and Technology},
      pages={1--11},
      year={2021}
    }

## Acknowledgement
This repository borrows code from [TIY] and [MTT]. Please acknowledge their work if you find this 
repo useful! 

[TIY]: https://github.com/gaschler/tiy
[MTT]: https://github.com/Smorodov/Multitarget-tracker
[OpenCV]: https://github.com/opencv/opencv
[OpenCV-contrib]: https://github.com/opencv/opencv_contrib
[Eigen]: https://eigen.tuxfamily.org/index.php?title=Main_Page
[Inside-Out Instrument Tracking for Surgical Navigation in Augmented Reality]: https://arbook.icg.tugraz.at/schmalstieg/Schmalstieg_402.pdf
[this toolbox]: https://github.com/RobVisLab/camera_calibration
