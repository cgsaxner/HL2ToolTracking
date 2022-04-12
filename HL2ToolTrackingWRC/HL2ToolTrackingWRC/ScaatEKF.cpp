#include "pch.h"

#define DBG_ENABLE_INFORMATIONAL_LOGGING 0
#define DBG_ENABLE_VERBOSE_LOGGING 0

// see https://docs.opencv.org/master/dc/d2c/tutorial_real_time_pose.html
ScaatEKF::ScaatEKF(
    int stateSize,
    int measurementSize,
    int controlSize,
    float lfFx, float lfFy, float lfCx, float lfCy,
    float rfFx, float rfFy, float rfCx, float rfCy,
    cv::Mat modelToLfInit) :
    m_state_size(stateSize), m_meas_size(measurementSize), m_contr_size(controlSize),
    m_lfFx(lfFx), m_lfFy(lfFy), m_lfCx(lfCx), m_lfCy(lfCy),
    m_rfFx(rfFx), m_rfFy(rfFy), m_rfCx(rfCx), m_rfCy(rfCy)
{
    // initialize matrices
    statePreX = cv::Mat::zeros(stateSize, 1, CV_32F);
    statePostX = cv::Mat::zeros(stateSize, 1, CV_32F);
    transitionMatrixA = cv::Mat::eye(stateSize, stateSize, CV_32F);

    processNoiseCovQ = cv::Mat::eye(stateSize, stateSize, CV_32F);
    measurementPredZ = cv::Mat::zeros(measurementSize, 1, CV_32F);
    measurementMatrixH = cv::Mat::zeros(measurementSize, stateSize, CV_32F);
    measurementNoiseCovR = cv::Mat::eye(measurementSize, measurementSize, CV_32F);

    errorCovPreP = cv::Mat::zeros(stateSize, stateSize, CV_32F);
    errorCovPostP = cv::Mat::zeros(stateSize, stateSize, CV_32F);
    gainK = cv::Mat::zeros(stateSize, measurementSize, CV_32F);

    temp1.create(stateSize, stateSize, CV_32F);
    temp2.create(measurementSize, stateSize, CV_32F);
    temp3.create(measurementSize, measurementSize, CV_32F);
    temp4.create(measurementSize, stateSize, CV_32F);
    temp5.create(measurementSize, 1, CV_32F);

    // set initial values

    // process noise covariance matrix Q
    // uncertainty in the state elements since previous sample
    // the higher, the more adaptable the filter is to quick changes
    // the lower, the more quick changes will be smoothed
    cv::setIdentity(processNoiseCovQ, cv::Scalar::all(1));       // set process noise
    SetProcessNoiseCovQ(1);

    // measurement noise covariance matrix R
    // uncertainty in the actual measurement
    // the higher, the noisier the measurement is assumed to be -> smoother results
    // the lower, the more accurate the measurement is -> more adaptable to quick changes
    cv::setIdentity(measurementNoiseCovR, cv::Scalar::all(1e-8));   // set measurement noise

    // posteriori state error covariance matrix P
    // covariance of the error in the estimated state -> filter's estimate of its own uncertainty
    cv::setIdentity(errorCovPreP, cv::Scalar::all(0));
    cv::setIdentity(errorCovPostP, cv::Scalar::all(0));             // error covariance

    cv::setIdentity(transitionMatrixA);
    //SetStateTransitionMatrixA(deltaT);

    // set the initial state
    cv::Mat R = modelToLfInit(cv::Rect(0, 0, 3, 3));
    cv::Mat t = modelToLfInit(cv::Rect(3, 0, 1, 3));

    statePostX.at<float>(0) = t.at<float>(0); //x
    statePostX.at<float>(1) = t.at<float>(1); //y
    statePostX.at<float>(2) = t.at<float>(2); //z

    // convert rotation matrix to quaternion
    Eigen::Vector4f quaternion;
    RotationMatrixToQuaternion(R, quaternion);

    // set the external rotation
    qxExt = quaternion[0];
    qyExt = quaternion[1];
    qzExt = quaternion[2];
    qwExt = quaternion[3];

    initialized = true;
}

void ScaatEKF::MainUpdateLoop(
    _In_ Marker marker,
    _In_ uint64_t timestamp,
    _In_ std::vector<cv::Point2f> blobs,
    _In_ ResearchModeSensorType sensorType,
    _In_ cv::Mat camToRefTransform,
    _Out_ Eigen::Matrix4f& modelToRefTransformE)
{
    modelToRefTransformE = Eigen::Matrix4f::Identity();

    //--------------------------------------------------------------------------------------
    // Determine assignment and update points
    // Todo: simplify! only update points or assignment are needed.
    //--------------------------------------------------------------------------------------

    std::vector<int> assignment = std::vector<int>(marker.numPoints, -1);
    bool hasAssignment = false;
    ScaatUpdateFrame updateFrame;

    Eigen::Matrix4f refToCamTransform =
        Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>
        (camToRefTransform.ptr<float>(), camToRefTransform.rows, camToRefTransform.cols).inverse();

    if (timestamp > lastTimestamp)
    {
#if  DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"ScaatEKF::MainUpdateLoop: Creating update frame.\n");
#endif //  DBG_ENABLE_VERBOSE_LOGGING
        if (sensorType == ResearchModeSensorType::LEFT_FRONT)
        {
            hasAssignment = ComputeAssignment(blobs, marker.lastLfPos, assignment);
            updateFrame = ScaatUpdateFrame{ (float)(timestamp - lastTimestamp) * 1e-7f,
                                            timestamp, sensorType, hasAssignment, assignment,
                                            blobs, refToCamTransform };
        }
        else if (sensorType == ResearchModeSensorType::RIGHT_FRONT)
        {
            hasAssignment = ComputeAssignment(blobs, marker.lastRfPos, assignment);
            updateFrame = ScaatUpdateFrame{ (float)(timestamp - lastTimestamp) * 1e-7f,
                                            timestamp, sensorType, hasAssignment, assignment,
                                            blobs, refToCamTransform
            };
        }

    }
    else
    {
#if  DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"ScaatEKF::MainUpdateLoop: No new measurement.\n");
#endif //  DBG_ENABLE_VERBOSE_LOGGING
        StateToTransform(statePostX, modelToRefTransformE);
        return;
    }

    if (updateFrame.hasAssignment)
    {
#if  DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"ScaatEKF::MainUpdateLoop: Predicting and correcting...\n");
#endif //  DBG_ENABLE_VERBOSE_LOGGING
        auto start_scaat4 = std::chrono::high_resolution_clock::now();
        PredictAndCorrect(
            updateFrame.assignment, updateFrame.updatePoints, updateFrame.type,
            updateFrame.refToCamTransform, marker, updateFrame.delta);
    }
    else
    {
        TimeUpdate(updateFrame.delta);
    }

    lastTimestamp = updateFrame.timestamp;
    StateToTransform(statePostX, modelToRefTransformE);
}

void ScaatEKF::PredictAndCorrect(
    _In_ std::vector<int> assignment,
    _In_ std::vector<cv::Point2f> pointsSorted,
    _In_ ResearchModeSensorType type,
    _In_ Eigen::Matrix4f refToCamTransform,
    _Inout_ Marker& marker,
    _Inout_ float& delta)
{
    if (delta > 0.1)
    {
        delta = delta / 10;
    }
    else if (delta > 0.05)
    {
        delta = delta / 2;
    }
    float factor = 1.0f;

    for (int i = 0; i < marker.numPoints; i++)
    {
        if (assignment[i] != -1)
        {
#if  DBG_ENABLE_VERBOSE_LOGGING
            OutputDebugStringW(L"ScaatEKF::MainUpdateLoop: Prediction step...\n");
#endif //  DBG_ENABLE_VERBOSE_LOGGING
            //--------------------------------------------------------------------------------------
            // Prediction step
            //--------------------------------------------------------------------------------------
            TimeUpdate(delta);
            SetProcessNoiseCovQ(factor);

            cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F);
            measurement.at<float>(0) = pointsSorted[assignment[i]].x;
            measurement.at<float>(1) = pointsSorted[assignment[i]].y;

            Eigen::Vector3f markerPoint = Eigen::Vector3f(
                marker.markerPoints.at<float>(0, i),
                marker.markerPoints.at<float>(1, i),
                marker.markerPoints.at<float>(2, i));

            //--------------------------------------------------------------------------------------
            // Correction step
            //--------------------------------------------------------------------------------------
            cv::Mat measurementPrediction;
            MeasurementUpdate(type, measurement, markerPoint,
                refToCamTransform, measurementPrediction);

            factor = factor * 0.5f;
            delta = 0;

            Eigen::Matrix4f modelToRefTransformEst;
            StateToTransform(statePostX, modelToRefTransformEst);
        }
    }
}

void ScaatEKF::SetStateTransitionMatrixA(float deltaT)
{
    // State transition matrix A
    // Linear model; see Welch2016 p75 eq 4.5
    transitionMatrixA.at<float>(0, 3) = deltaT;
    transitionMatrixA.at<float>(1, 4) = deltaT;
    transitionMatrixA.at<float>(2, 5) = deltaT;

    transitionMatrixA.at<float>(6, 9) = deltaT;
    transitionMatrixA.at<float>(7, 10) = deltaT;
    transitionMatrixA.at<float>(8, 11) = deltaT;
}

void ScaatEKF::SetProcessNoiseCovQ(float deltaT)
{
    float mu_xyz = 1e-1f;
    float mu_rot = 1.0f;

    processNoiseCovQ.at<float>(0, 0) = mu_xyz * deltaT * 1;
    processNoiseCovQ.at<float>(0, 3) = mu_xyz * deltaT * 2;
    processNoiseCovQ.at<float>(1, 1) = mu_xyz * deltaT * 1;
    processNoiseCovQ.at<float>(1, 4) = mu_xyz * deltaT * 2;
    processNoiseCovQ.at<float>(2, 2) = mu_xyz * deltaT * 1;
    processNoiseCovQ.at<float>(2, 5) = mu_xyz * deltaT * 2;
    processNoiseCovQ.at<float>(3, 0) = mu_xyz * deltaT * 2;
    processNoiseCovQ.at<float>(3, 3) = mu_xyz * deltaT * 5;
    processNoiseCovQ.at<float>(4, 1) = mu_xyz * deltaT * 2;
    processNoiseCovQ.at<float>(4, 4) = mu_xyz * deltaT * 5;
    processNoiseCovQ.at<float>(5, 2) = mu_xyz * deltaT * 2;
    processNoiseCovQ.at<float>(5, 5) = mu_xyz * deltaT * 5;
    processNoiseCovQ.at<float>(6, 6) = mu_rot * deltaT * 2;
    processNoiseCovQ.at<float>(6, 9) = mu_rot * deltaT * 4;
    processNoiseCovQ.at<float>(7, 7) = mu_rot * deltaT * 2;
    processNoiseCovQ.at<float>(7, 10) = mu_rot * deltaT * 4;
    processNoiseCovQ.at<float>(8, 8) = mu_rot * deltaT * 2;
    processNoiseCovQ.at<float>(8, 11) = mu_rot * deltaT * 4;
    processNoiseCovQ.at<float>(9, 6) = mu_rot * deltaT * 4;
    processNoiseCovQ.at<float>(9, 9) = mu_rot * deltaT * 10;
    processNoiseCovQ.at<float>(10, 7) = mu_rot * deltaT * 4;
    processNoiseCovQ.at<float>(10, 10) = mu_rot * deltaT * 10;
    processNoiseCovQ.at<float>(11, 8) = mu_rot * deltaT * 4;
    processNoiseCovQ.at<float>(11, 11) = mu_rot * deltaT * 10;
}


// prediction step
void ScaatEKF::TimeUpdate(float deltaT)
{
    // update the state transition matrix
    // TODO: should this be the jacobian of the transition function? 
    // see https://stackoverflow.com/questions/21533093/opencv-how-to-use-the-kalmanfilter-class-as-extendedkf
    SetStateTransitionMatrixA(deltaT);

    // Welch2016 p84 eq 4.14
    // update the state: x'(k) = A*x(k)
    statePreX = transitionMatrixA * statePostX;

    // Welch2016 p84 eq 4.14
    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrixA * errorCovPostP;

    // P'(k) = temp1*At + Q = A * P(k) * At + Q
    gemm(temp1, transitionMatrixA, 1, processNoiseCovQ, 1, errorCovPreP, cv::GEMM_2_T);

    // handle the case when there will be no measurement before the next predict.
    statePreX.copyTo(statePostX);
    errorCovPreP.copyTo(errorCovPostP);
}

// correction step
void ScaatEKF::MeasurementUpdate(
    _In_ ResearchModeSensorType type,
    _In_ cv::Mat measurement,
    _In_ Eigen::Vector3f modelPoint,
    _In_ Eigen::Matrix4f refToCamTransform,
    _Out_ cv::Mat& measurementPrediction)
{
    // ---------------------
    // Compute the measurement prediction from the state prediction, as well as measurement jacobian H
    if (type == ResearchModeSensorType::LEFT_FRONT)
    {
        MeasurementFunctionh(statePreX, refToCamTransform, modelPoint, m_lfFx, m_lfFy, m_lfCx, m_lfCy);
    }
    else if (type == ResearchModeSensorType::RIGHT_FRONT)
    {
        MeasurementFunctionh(statePreX, refToCamTransform, modelPoint, m_rfFx, m_rfFy, m_rfCx, m_rfCy);
    }
    else
    {
        return;
    }
    measurementPrediction = measurementPredZ;

    // ---------------------
    // Compute Kalman gain 
    //
    // K = P' * Ht * (H * P' * Ht + R).inv() 
    // Welch2016 p87 eq 4.16

    // temp2 = H*P'(k)
    temp2 = measurementMatrixH * errorCovPreP;

    // temp3 = temp2  * Ht + R = H*P'(k) * Ht + R
    gemm(temp2, measurementMatrixH, 1, measurementNoiseCovR, 1, temp3, cv::GEMM_2_T);
    //temp3 = temp2 * measurementMatrixH.t() + measurementNoiseCovR;

    //cv::Mat temp25 = temp2 * measurementMatrixH.t();

    // temp4 = Kt(k)= inv(temp3) * temp2 = (H * P'(k) * Ht + R).inv() * H * P'
    solve(temp3, temp2, temp4, cv::DECOMP_SVD);
    //temp4 = temp3.inv() * temp2;

    // K(k)
    gainK = temp4.t();
    // ---------------------

    // Measurement residual
    // deltaZ = z - measurementPredZ = z - h(statePreX, a -> external quaternion, b, c -> device parameters)
    // Welch2016 p92 eq 4.19
    // temp5 = z(k) - measurementPredZ
    temp5 = measurement - measurementPredZ;

    cv::Mat stateUpdate = gainK * temp5;

    // ---------------------
    // Correct state and error covariance
    //
    // Welch2016 p 94 eq 4.20
    // x(k) = x'(k) + K(k) * temp5
    statePostX = statePreX + gainK * temp5;

    // P(k) = P'(k) - K(k)*temp2
    errorCovPostP = errorCovPreP - gainK * temp2;
    // ---------------------

    // ---------------------
    // Update the external quaternion and zero the incremental rotation
    //
    // Welch2016 p 96 eq 4.21
    float roll = statePostX.at<float>(6);
    float pitch = statePostX.at<float>(7);
    float yaw = statePostX.at<float>(8);

    float cyaw = cos(yaw * 0.5f);
    float syaw = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    float iw = cr * cp * cyaw + sr * sp * syaw;
    float ix = sr * cp * cyaw - cr * sp * syaw;
    float iy = cr * sp * cyaw + sr * cp * syaw;
    float iz = cr * cp * syaw - sr * sp * cyaw;

    // combine incremental rotation with external rotation
    float qx = qxExt * iw + qyExt * iz - qzExt * iy + qwExt * ix;
    float qy = -qxExt * iz + qyExt * iw + qzExt * ix + qwExt * iy;
    float qz = qxExt * iy - qyExt * ix + qzExt * iw + qwExt * iz;
    float qw = -qxExt * ix - qyExt * iy - qzExt * iz + qwExt * iw;

    qxExt = qx;
    qyExt = qy;
    qzExt = qz;
    qwExt = qw;

    statePostX.at<float>(6) = 0.0f;
    statePostX.at<float>(7) = 0.0f;
    statePostX.at<float>(8) = 0.0f;
}

void ScaatEKF::StateToTransform(
    _In_ cv::Mat stateX,
    _Out_ cv::Mat& modelToWorldTransformEst)
{
    modelToWorldTransformEst = cv::Mat::eye(4, 4, CV_32F);

    modelToWorldTransformEst.at<float>(0, 3) = stateX.at<float>(0);
    modelToWorldTransformEst.at<float>(1, 3) = stateX.at<float>(1);
    modelToWorldTransformEst.at<float>(2, 3) = stateX.at<float>(2);

    modelToWorldTransformEst.at<float>(0, 0) = 1 - 2 * (qyExt * qyExt + qzExt * qzExt);
    modelToWorldTransformEst.at<float>(0, 1) = 2 * (qxExt * qyExt - qzExt * qwExt);
    modelToWorldTransformEst.at<float>(0, 2) = 2 * (qxExt * qzExt + qyExt * qwExt);

    modelToWorldTransformEst.at<float>(1, 0) = 2 * (qxExt * qyExt + qzExt * qwExt);
    modelToWorldTransformEst.at<float>(1, 1) = 1 - 2 * (qxExt * qxExt + qzExt * qzExt);
    modelToWorldTransformEst.at<float>(1, 2) = 2 * (qyExt * qzExt - qxExt * qwExt);

    modelToWorldTransformEst.at<float>(2, 0) = 2 * (qxExt * qzExt - qyExt * qwExt);
    modelToWorldTransformEst.at<float>(2, 1) = 2 * (qyExt * qzExt + qxExt * qwExt);
    modelToWorldTransformEst.at<float>(2, 2) = 1 - 2 * (qxExt * qxExt + qyExt * qyExt);
}

void ScaatEKF::StateToTransform(
    _In_ cv::Mat stateX,
    _Out_ Eigen::Matrix4f& modelToWorldTransformEst)
{
    modelToWorldTransformEst = Eigen::Matrix4f::Identity(4, 4);

    modelToWorldTransformEst(0, 3) = stateX.at<float>(0);
    modelToWorldTransformEst(1, 3) = stateX.at<float>(1);
    modelToWorldTransformEst(2, 3) = stateX.at<float>(2);

    modelToWorldTransformEst(0, 0) = 1 - 2 * (qyExt * qyExt + qzExt * qzExt);
    modelToWorldTransformEst(0, 1) = 2 * (qxExt * qyExt - qzExt * qwExt);
    modelToWorldTransformEst(0, 2) = 2 * (qxExt * qzExt + qyExt * qwExt);

    modelToWorldTransformEst(1, 0) = 2 * (qxExt * qyExt + qzExt * qwExt);
    modelToWorldTransformEst(1, 1) = 1 - 2 * (qxExt * qxExt + qzExt * qzExt);
    modelToWorldTransformEst(1, 2) = 2 * (qyExt * qzExt - qxExt * qwExt);

    modelToWorldTransformEst(2, 0) = 2 * (qxExt * qzExt - qyExt * qwExt);
    modelToWorldTransformEst(2, 1) = 2 * (qyExt * qzExt + qxExt * qwExt);
    modelToWorldTransformEst(2, 2) = 1 - 2 * (qxExt * qxExt + qyExt * qyExt);
}

// the measurement function h() maps from a state x to a measurement z; in that, it uses information about the current camera
// simultaneously, the measurement jacobian H is calculated
void ScaatEKF::MeasurementFunctionh(
    _In_ cv::Mat stateX,
    _In_ Eigen::Matrix4f worldToCam,
    _In_ Eigen::Vector3f modelPoint,
    _In_ float fx, _In_ float fy,
    _In_ float cx, _In_ float cy)
{
    // incremental rotation to quaternion
    float roll = stateX.at<float>(6);
    float pitch = stateX.at<float>(7);
    float yaw = stateX.at<float>(8);

    float cyaw = cos(yaw * 0.5f);
    float syaw = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    // derivatives
    float dcyaw_dyaw = -0.5f * sin(yaw * 0.5f);
    float dsyaw_dyaw = 0.5f * cos(yaw * 0.5f);

    float dcp_dpitch = -0.5f * sin(pitch * 0.5f);
    float dsp_dpitch = 0.5f * cos(pitch * 0.5f);

    float dcr_droll = -0.5f * sin(roll * 0.5f);
    float dsr_droll = 0.5f * cos(roll * 0.5f);

    // incremental rotation
    float iw = cr * cp * cyaw + sr * sp * syaw;
    float ix = sr * cp * cyaw - cr * sp * syaw;
    float iy = cr * sp * cyaw + sr * cp * syaw;
    float iz = cr * cp * syaw - sr * sp * cyaw;

    // derivatives
    float diw_droll = dcr_droll * cp * cyaw + dsr_droll * sp * syaw;
    float diw_dpitch = cr * dcp_dpitch * cyaw + sr * dsp_dpitch * syaw;
    float diw_dyaw = cr * cp * dcyaw_dyaw + sr * sp * dsyaw_dyaw;

    float dix_droll = dsr_droll * cp * cyaw - dcr_droll * sp * syaw;
    float dix_dpitch = sr * dcp_dpitch * cyaw - cr * dsp_dpitch * syaw;
    float dix_dyaw = sr * cp * dcyaw_dyaw - cr * sp * dsyaw_dyaw;

    float diy_droll = dcr_droll * sp * cyaw + dsr_droll * cp * syaw;
    float diy_dpitch = cr * dsp_dpitch * cyaw + sr * dcp_dpitch * syaw;
    float diy_dyaw = cr * sp * dcyaw_dyaw + sr * cp * dsyaw_dyaw;

    float diz_droll = dcr_droll * cp * syaw - dsr_droll * sp * cyaw;
    float diz_dpitch = cr * dcp_dpitch * syaw - sr * dsp_dpitch * cyaw;
    float diz_dyaw = cr * cp * dsyaw_dyaw - sr * sp * dcyaw_dyaw;

    // combine incremental rotation with external rotation
    float qx = qxExt * iw + qyExt * iz - qzExt * iy + qwExt * ix;
    float qy = -qxExt * iz + qyExt * iw + qzExt * ix + qwExt * iy;
    float qz = qxExt * iy - qyExt * ix + qzExt * iw + qwExt * iz;
    float qw = -qxExt * ix - qyExt * iy - qzExt * iz + qwExt * iw;

    // derivatives
    float dqx_droll = qxExt * diw_droll + qyExt * diz_droll - qzExt * diy_droll + qwExt * dix_droll;
    float dqx_dpitch = qxExt * diw_dpitch + qyExt * diz_dpitch - qzExt * diy_dpitch + qwExt * dix_dpitch;
    float dqx_dyaw = qxExt * diw_dyaw + qyExt * diz_dyaw - qzExt * diy_dyaw + qwExt * dix_dyaw;

    float dqy_droll = -qxExt * diz_droll + qyExt * diw_droll + qzExt * dix_droll + qwExt * diy_droll;
    float dqy_dpitch = -qxExt * diz_dpitch + qyExt * diw_dpitch + qzExt * dix_dpitch + qwExt * diy_dpitch;
    float dqy_dyaw = -qxExt * diz_dyaw + qyExt * diw_dyaw + qzExt * dix_dyaw + qwExt * diy_dyaw;

    float dqz_droll = qxExt * diy_droll - qyExt * dix_droll + qzExt * diw_droll + qwExt * diz_droll;
    float dqz_dpitch = qxExt * diy_dpitch - qyExt * dix_dpitch + qzExt * diw_dpitch + qwExt * diz_dpitch;
    float dqz_dyaw = qxExt * diy_dyaw - qyExt * dix_dyaw + qzExt * diw_dyaw + qwExt * diz_dyaw;

    float dqw_droll = -qxExt * dix_droll - qyExt * diy_droll - qzExt * diz_droll + qwExt * diw_droll;
    float dqw_dpitch = -qxExt * dix_dpitch - qyExt * diy_dpitch - qzExt * diz_dpitch + qwExt * diw_dpitch;
    float dqw_dyaw = -qxExt * dix_dyaw - qyExt * diy_dyaw - qzExt * diz_dyaw + qwExt * diw_dyaw;

    // create a rotation matrix from the quaternion
    float m00 = 1 - 2 * (qy * qy + qz * qz);
    float m01 = 2 * (qx * qy - qz * qw);
    float m02 = 2 * (qx * qz + qy * qw);

    float m10 = 2 * (qx * qy + qz * qw);
    float m11 = 1 - 2 * (qx * qx + qz * qz);
    float m12 = 2 * (qy * qz - qx * qw);

    float m20 = 2 * (qx * qz - qy * qw);
    float m21 = 2 * (qy * qz + qx * qw);
    float m22 = 1 - 2 * (qx * qx + qy * qy);

    // derivatives
    float dm00_droll = -4 * (qy * dqy_droll + qz * dqz_droll);
    float dm00_dpitch = -4 * (qy * dqy_dpitch + qz * dqz_dpitch);
    float dm00_dyaw = -4 * (qy * dqy_dyaw + qz * dqz_dyaw);

    float dm01_droll = 2 * (qy * dqx_droll + qx * dqy_droll - qw * dqz_droll - qz * dqw_droll);
    float dm01_dpitch = 2 * (qy * dqx_dpitch + qx * dqy_dpitch - qw * dqz_dpitch - qz * dqw_dpitch);
    float dm01_dyaw = 2 * (qy * dqx_dyaw + qx * dqy_dyaw - qw * dqz_dyaw - qz * dqw_dyaw);

    float dm02_droll = 2 * (qz * dqx_droll + qx * dqz_droll + qw * dqy_droll + qy * dqw_droll);
    float dm02_dpitch = 2 * (qz * dqx_dpitch + qx * dqz_dpitch + qw * dqy_dpitch + qy * dqw_dpitch);
    float dm02_dyaw = 2 * (qz * dqx_dyaw + qx * dqz_dyaw + qw * dqy_dyaw + qy * dqw_dyaw);

    float dm10_droll = 2 * (qy * dqx_droll + qx * dqy_droll + qw * dqz_droll + qz * dqw_droll);
    float dm10_dpitch = 2 * (qy * dqx_dpitch + qx * dqy_dpitch + qw * dqz_dpitch + qz * dqw_dpitch);
    float dm10_dyaw = 2 * (qy * dqx_dyaw + qx * dqy_dyaw + qw * dqz_dyaw + qz * dqw_dyaw);

    float dm11_droll = -4 * (qx * dqx_droll + qz * dqz_droll);
    float dm11_dpitch = -4 * (qx * dqx_dpitch + qz * dqz_dpitch);
    float dm11_dyaw = -4 * (qx * dqx_dyaw + qz * dqz_dyaw);

    float dm12_droll = 2 * (qz * dqy_droll + qy * dqz_droll - qw * dqx_droll - qx * dqw_droll);
    float dm12_dpitch = 2 * (qz * dqy_dpitch + qy * dqz_dpitch - qw * dqx_dpitch - qx * dqw_dpitch);
    float dm12_dyaw = 2 * (qz * dqy_dyaw + qy * dqz_dyaw - qw * dqx_dyaw - qx * dqw_dyaw);

    float dm20_droll = 2 * (qz * dqx_droll + qx * dqz_droll - qw * dqy_droll - qy * dqw_droll);
    float dm20_dpitch = 2 * (qz * dqx_dpitch + qx * dqz_dpitch - qw * dqy_dpitch - qy * dqw_dpitch);
    float dm20_dyaw = 2 * (qz * dqx_dyaw + qx * dqz_dyaw - qw * dqy_dyaw - qy * dqw_dyaw);

    float dm21_droll = 2 * (qz * dqy_droll + qy * dqz_droll + qw * dqx_droll + qx * dqw_droll);
    float dm21_dpitch = 2 * (qz * dqy_dpitch + qy * dqz_dpitch + qw * dqx_dpitch + qx * dqw_dpitch);
    float dm21_dyaw = 2 * (qz * dqy_dyaw + qy * dqz_dyaw + qw * dqx_dyaw + qx * dqw_dyaw);

    float dm22_droll = -4 * (qx * dqx_droll + qy * dqy_droll);
    float dm22_dpitch = -4 * (qx * dqx_dpitch + qy * dqy_dpitch);
    float dm22_dyaw = -4 * (qx * dqx_dyaw + qy * dqy_dyaw);

    // translation
    float x = stateX.at<float>(0);
    float y = stateX.at<float>(1);
    float z = stateX.at<float>(2);

    // rotate and translate model point to world space
    float X = modelPoint(0) * m00 + modelPoint(1) * m01 + modelPoint(2) * m02 + x;
    float Y = modelPoint(0) * m10 + modelPoint(1) * m11 + modelPoint(2) * m12 + y;
    float Z = modelPoint(0) * m20 + modelPoint(1) * m21 + modelPoint(2) * m22 + z;

    // derivatives
    float dX_dx = 1;
    float dX_droll = modelPoint(0) * dm00_droll + modelPoint(1) * dm01_droll + modelPoint(2) * dm02_droll;
    float dX_dpitch = modelPoint(0) * dm00_dpitch + modelPoint(1) * dm01_dpitch + modelPoint(2) * dm02_dpitch;
    float dX_dyaw = modelPoint(0) * dm00_dyaw + modelPoint(1) * dm01_dyaw + modelPoint(2) * dm02_dyaw;

    float dY_dy = 1;
    float dY_droll = modelPoint(0) * dm10_droll + modelPoint(1) * dm11_droll + modelPoint(2) * dm12_droll;
    float dY_dpitch = modelPoint(0) * dm10_dpitch + modelPoint(1) * dm11_dpitch + modelPoint(2) * dm12_dpitch;
    float dY_dyaw = modelPoint(0) * dm10_dyaw + modelPoint(1) * dm11_dyaw + modelPoint(2) * dm12_dyaw;

    float dZ_dz = 1;
    float dZ_droll = modelPoint(0) * dm20_droll + modelPoint(1) * dm21_droll + modelPoint(2) * dm22_droll;
    float dZ_dpitch = modelPoint(0) * dm20_dpitch + modelPoint(1) * dm21_dpitch + modelPoint(2) * dm22_dpitch;
    float dZ_dyaw = modelPoint(0) * dm20_dyaw + modelPoint(1) * dm21_dyaw + modelPoint(2) * dm22_dyaw;

    // rotate and translate point to camera space
    float X2 = X * worldToCam(0, 0) + Y * worldToCam(0, 1) + Z * worldToCam(0, 2) + worldToCam(0, 3);
    float Y2 = X * worldToCam(1, 0) + Y * worldToCam(1, 1) + Z * worldToCam(1, 2) + worldToCam(1, 3);
    float Z2 = X * worldToCam(2, 0) + Y * worldToCam(2, 1) + Z * worldToCam(2, 2) + worldToCam(2, 3);

    // derivatives
    float dX2_dx = dX_dx * worldToCam(0, 0);
    float dX2_dy = dY_dy * worldToCam(0, 1);
    float dX2_dz = dZ_dz * worldToCam(0, 2);
    float dX2_droll = dX_droll * worldToCam(0, 0) + dY_droll * worldToCam(0, 1) + dZ_droll * worldToCam(0, 2);
    float dX2_dpitch = dX_dpitch * worldToCam(0, 0) + dY_dpitch * worldToCam(0, 1) + dZ_dpitch * worldToCam(0, 2);
    float dX2_dyaw = dX_dyaw * worldToCam(0, 0) + dY_dyaw * worldToCam(0, 1) + dZ_dyaw * worldToCam(0, 2);

    float dY2_dx = dX_dx * worldToCam(1, 0);
    float dY2_dy = dY_dy * worldToCam(1, 1);
    float dY2_dz = dZ_dz * worldToCam(1, 2);
    float dY2_droll = dX_droll * worldToCam(1, 0) + dY_droll * worldToCam(1, 1) + dZ_droll * worldToCam(1, 2);
    float dY2_dpitch = dX_dpitch * worldToCam(1, 0) + dY_dpitch * worldToCam(1, 1) + dZ_dpitch * worldToCam(1, 2);
    float dY2_dyaw = dX_dyaw * worldToCam(1, 0) + dY_dyaw * worldToCam(1, 1) + dZ_dyaw * worldToCam(1, 2);

    float dZ2_dx = dX_dx * worldToCam(2, 0);
    float dZ2_dy = dY_dy * worldToCam(2, 1);
    float dZ2_dz = dZ_dz * worldToCam(2, 2);
    float dZ2_droll = dX_droll * worldToCam(2, 0) + dY_droll * worldToCam(2, 1) + dZ_droll * worldToCam(2, 2);
    float dZ2_dpitch = dX_dpitch * worldToCam(2, 0) + dY_dpitch * worldToCam(2, 1) + dZ_dpitch * worldToCam(2, 2);
    float dZ2_dyaw = dX_dyaw * worldToCam(2, 0) + dY_dyaw * worldToCam(2, 1) + dZ_dyaw * worldToCam(2, 2);

    // perform back-projection
    float u = fx * (X2 / Z2) + cx;
    float v = fy * (Y2 / Z2) + cy;

    // derivatives
    float du_dx = fx * (dX2_dx * Z2 - dZ2_dx * X2) / pow(Z2, 2.0f);
    float du_dy = fx * (dX2_dy * Z2 - dZ2_dy * X2) / pow(Z2, 2.0f);
    float du_dz = fx * (dX2_dz * Z2 - dZ2_dz * X2) / pow(Z2, 2.0f);
    float du_droll = fx * (dX2_droll * Z2 - dZ2_droll * X2) / pow(Z2, 2.0f);
    float du_dpitch = fx * (dX2_dpitch * Z2 - dZ2_dpitch * X2) / pow(Z2, 2.0f);
    float du_dyaw = fx * (dX2_dyaw * Z2 - dZ2_dyaw * X2) / pow(Z2, 2.0f);

    float dv_dx = fy * (dY2_dx * Z2 - dZ2_dx * Y2) / pow(Z2, 2.0f);
    float dv_dy = fy * (dY2_dy * Z2 - dZ2_dy * Y2) / pow(Z2, 2.0f);
    float dv_dz = fy * (dY2_dz * Z2 - dZ2_dz * Y2) / pow(Z2, 2.0f);
    float dv_droll = fy * (dY2_droll * Z2 - dZ2_droll * Y2) / pow(Z2, 2.0f);
    float dv_dpitch = fy * (dY2_dpitch * Z2 - dZ2_dpitch * Y2) / pow(Z2, 2.0f);
    float dv_dyaw = fy * (dY2_dyaw * Z2 - dZ2_dyaw * Y2) / pow(Z2, 2.0f);

    measurementPredZ.at<float>(0) = u;
    measurementPredZ.at<float>(1) = v;

    measurementMatrixH.at<float>(0, 0) = du_dx;
    measurementMatrixH.at<float>(0, 1) = du_dy;
    measurementMatrixH.at<float>(0, 2) = du_dz;

    measurementMatrixH.at<float>(0, 6) = du_droll;
    measurementMatrixH.at<float>(0, 7) = du_dpitch;
    measurementMatrixH.at<float>(0, 8) = du_dyaw;

    measurementMatrixH.at<float>(1, 0) = dv_dx;
    measurementMatrixH.at<float>(1, 1) = dv_dy;
    measurementMatrixH.at<float>(1, 2) = dv_dz;

    measurementMatrixH.at<float>(1, 6) = dv_droll;
    measurementMatrixH.at<float>(1, 7) = dv_dpitch;
    measurementMatrixH.at<float>(1, 8) = dv_dyaw;
}

bool ScaatEKF::ComputeAssignment(
    _In_ std::vector<cv::Point2f> pointsSorted,
    _In_ std::vector<cv::Point2f> lastPoints,
    _Out_ std::vector<int>& assignment)
{
    AssignmentProblemSolver solver;

    const size_t N = lastPoints.size();	// # points in the model
    const size_t M = pointsSorted.size(); // # points in the target

    assignment = std::vector<int>(lastPoints.size(), -1);
    std::vector<float> cost_matrix(lastPoints.size() * pointsSorted.size());
    bool hasAssignment = false;

    // assign pointsSorted to lastPoints
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < M; j++)
        {
            float diff = CalculateDistance(pointsSorted[j], lastPoints[i]);
            cost_matrix[i + j * N] = diff;
        }
    }

    solver.Solve(cost_matrix, lastPoints.size(), pointsSorted.size(), assignment);

    for (int i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] != -1)
        {
            if (cost_matrix[i + assignment[i] * N] > 35)
            {
                assignment[i] = -1;
            }
            else
            {
                hasAssignment = true;
            }
        }
    }
    return hasAssignment;
}

float ScaatEKF::CalculateDistance(
    const cv::Point2f p1,
    const cv::Point2f p2)
{
    float diffY = p1.y - p2.y;
    float diffX = p1.x - p2.x;
    return sqrt((diffY * diffY) + (diffX * diffX));
}
