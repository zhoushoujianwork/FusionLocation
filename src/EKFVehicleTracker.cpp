#include "EKFVehicleTracker.h"

// 常量定义
#define EARTH_RADIUS 6371000.0f
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f

EKFVehicleTracker::EKFVehicleTracker(IIMUProvider* imu, double initLat, double initLng) 
    : imuProvider(imu), gpsProvider(nullptr), magProvider(nullptr),
      originLat(initLat), originLng(initLng) {
    
    hasOrigin = (initLat != 0 || initLng != 0);
}

void EKFVehicleTracker::setGPSProvider(IGPSProvider* gps) {
    gpsProvider = gps;
}

void EKFVehicleTracker::setMagProvider(IMagProvider* mag) {
    magProvider = mag;
}

void EKFVehicleTracker::setEKFConfig(const EKFConfig& cfg) {
    config = cfg;
}

void EKFVehicleTracker::setVehicleModel(const VehicleModel& model) {
    vehicleModel = model;
}

void EKFVehicleTracker::setInitialPosition(double lat, double lng) {
    originLat = lat;
    originLng = lng;
    hasOrigin = true;
}

void EKFVehicleTracker::begin() {
    if (!hasOrigin) return;
    
    // 初始化状态向量 [x, y, vx, vy, heading, heading_rate]
    memset(ekfState.x, 0, sizeof(ekfState.x));
    
    // 初始化协方差矩阵
    memset(ekfState.P, 0, sizeof(ekfState.P));
    ekfState.P[MATRIX_INDEX(0, 0, EKF_STATE_DIM)] = config.initialPosVariance;
    ekfState.P[MATRIX_INDEX(1, 1, EKF_STATE_DIM)] = config.initialPosVariance;
    ekfState.P[MATRIX_INDEX(2, 2, EKF_STATE_DIM)] = config.initialVelVariance;
    ekfState.P[MATRIX_INDEX(3, 3, EKF_STATE_DIM)] = config.initialVelVariance;
    ekfState.P[MATRIX_INDEX(4, 4, EKF_STATE_DIM)] = config.initialHeadingVariance;
    ekfState.P[MATRIX_INDEX(5, 5, EKF_STATE_DIM)] = config.initialHeadingVariance;
    
    ekfState.initialized = true;
    ekfState.lastUpdate = millis();
    
    // 初始化当前位置
    currentPosition.lat = originLat;
    currentPosition.lng = originLng;
    currentPosition.valid = true;
    currentPosition.timestamp = millis();
}

void EKFVehicleTracker::update() {
    if (!ekfState.initialized) return;
    
    uint32_t currentTime = millis();
    float deltaTime = (currentTime - ekfState.lastUpdate) / 1000.0f;
    
    if (deltaTime > 0.001f && deltaTime < 1.0f) {
        // 预测步骤
        predict(deltaTime);
        
        // 更新步骤
        if (imuProvider && imuProvider->isAvailable()) {
            IMUData imuData;
            if (imuProvider->getData(imuData)) {
                updateWithIMU(imuData);
                currentPosition.sources.hasIMU = true;
            }
        }
        
        if (gpsProvider && gpsProvider->isAvailable()) {
            GPSData gpsData;
            if (gpsProvider->getData(gpsData)) {
                updateWithGPS(gpsData);
                currentPosition.sources.hasGPS = true;
            }
        }
        
        if (magProvider && magProvider->isAvailable()) {
            MagData magData;
            if (magProvider->getData(magData)) {
                updateWithMag(magData);
                currentPosition.sources.hasMag = true;
            }
        }
        
        // 更新位置信息
        updatePositionFromState();
    }
    
    ekfState.lastUpdate = currentTime;
}

void EKFVehicleTracker::predict(float deltaTime) {
    // 状态转移：车辆运动模型
    // x(k+1) = x(k) + vx(k) * dt
    // y(k+1) = y(k) + vy(k) * dt
    // vx(k+1) = vx(k)  (假设速度在短时间内恒定)
    // vy(k+1) = vy(k)
    // heading(k+1) = heading(k) + heading_rate(k) * dt
    // heading_rate(k+1) = heading_rate(k)
    
    float newState[EKF_STATE_DIM];
    newState[0] = ekfState.x[0] + ekfState.x[2] * deltaTime; // x
    newState[1] = ekfState.x[1] + ekfState.x[3] * deltaTime; // y
    newState[2] = ekfState.x[2]; // vx
    newState[3] = ekfState.x[3]; // vy
    newState[4] = normalizeAngle(ekfState.x[4] + ekfState.x[5] * deltaTime); // heading
    newState[5] = ekfState.x[5]; // heading_rate
    
    // 状态转移矩阵 F
    float F[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
    F[MATRIX_INDEX(0, 0, EKF_STATE_DIM)] = 1.0f;
    F[MATRIX_INDEX(0, 2, EKF_STATE_DIM)] = deltaTime;
    F[MATRIX_INDEX(1, 1, EKF_STATE_DIM)] = 1.0f;
    F[MATRIX_INDEX(1, 3, EKF_STATE_DIM)] = deltaTime;
    F[MATRIX_INDEX(2, 2, EKF_STATE_DIM)] = 1.0f;
    F[MATRIX_INDEX(3, 3, EKF_STATE_DIM)] = 1.0f;
    F[MATRIX_INDEX(4, 4, EKF_STATE_DIM)] = 1.0f;
    F[MATRIX_INDEX(4, 5, EKF_STATE_DIM)] = deltaTime;
    F[MATRIX_INDEX(5, 5, EKF_STATE_DIM)] = 1.0f;
    
    // 过程噪声协方差矩阵 Q
    float Q[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
    Q[MATRIX_INDEX(0, 0, EKF_STATE_DIM)] = config.processNoisePos * deltaTime * deltaTime;
    Q[MATRIX_INDEX(1, 1, EKF_STATE_DIM)] = config.processNoisePos * deltaTime * deltaTime;
    Q[MATRIX_INDEX(2, 2, EKF_STATE_DIM)] = config.processNoiseVel * deltaTime;
    Q[MATRIX_INDEX(3, 3, EKF_STATE_DIM)] = config.processNoiseVel * deltaTime;
    Q[MATRIX_INDEX(4, 4, EKF_STATE_DIM)] = config.processNoiseHeading * deltaTime;
    Q[MATRIX_INDEX(5, 5, EKF_STATE_DIM)] = config.processNoiseHeadingRate * deltaTime;
    
    // 更新状态
    memcpy(ekfState.x, newState, sizeof(newState));
    
    // 更新协方差: P = F * P * F^T + Q
    float FP[EKF_STATE_DIM * EKF_STATE_DIM];
    float FT[EKF_STATE_DIM * EKF_STATE_DIM];
    float FPFT[EKF_STATE_DIM * EKF_STATE_DIM];
    
    matrixMultiply(F, ekfState.P, FP, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);
    matrixTranspose(F, FT, EKF_STATE_DIM, EKF_STATE_DIM);
    matrixMultiply(FP, FT, FPFT, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);
    matrixAdd(FPFT, Q, ekfState.P, EKF_STATE_DIM, EKF_STATE_DIM);
}

void EKFVehicleTracker::updateWithGPS(const GPSData& gpsData) {
    if (!gpsData.valid) return;
    
    // 将GPS经纬度转换为本地坐标
    float measX, measY;
    latLngToXY(gpsData.lat, gpsData.lng, measX, measY);
    
    // 测量向量 z = [x, y]
    float z[EKF_MEASUREMENT_GPS_DIM] = {measX, measY};
    
    // 观测矩阵 H (GPS直接观测位置)
    float H[EKF_MEASUREMENT_GPS_DIM * EKF_STATE_DIM] = {0};
    H[MATRIX_INDEX(0, 0, EKF_STATE_DIM)] = 1.0f; // 观测x
    H[MATRIX_INDEX(1, 1, EKF_STATE_DIM)] = 1.0f; // 观测y
    
    // 测量噪声协方差矩阵 R
    float R[EKF_MEASUREMENT_GPS_DIM * EKF_MEASUREMENT_GPS_DIM] = {0};
    float gpsVariance = gpsData.accuracy * gpsData.accuracy;
    if (gpsVariance < config.gpsNoisePos) gpsVariance = config.gpsNoisePos;
    R[MATRIX_INDEX(0, 0, EKF_MEASUREMENT_GPS_DIM)] = gpsVariance;
    R[MATRIX_INDEX(1, 1, EKF_MEASUREMENT_GPS_DIM)] = gpsVariance;
    
    // 预测测量值 h(x) = [x, y]
    float h[EKF_MEASUREMENT_GPS_DIM] = {ekfState.x[0], ekfState.x[1]};
    
    // 创新 y = z - h(x)
    float innovation[EKF_MEASUREMENT_GPS_DIM];
    matrixSubtract(z, h, innovation, EKF_MEASUREMENT_GPS_DIM, 1);
    
    // 创新协方差 S = H * P * H^T + R
    float HP[EKF_MEASUREMENT_GPS_DIM * EKF_STATE_DIM];
    float HT[EKF_STATE_DIM * EKF_MEASUREMENT_GPS_DIM];
    float HPHT[EKF_MEASUREMENT_GPS_DIM * EKF_MEASUREMENT_GPS_DIM];
    float S[EKF_MEASUREMENT_GPS_DIM * EKF_MEASUREMENT_GPS_DIM];
    
    matrixMultiply(H, ekfState.P, HP, EKF_MEASUREMENT_GPS_DIM, EKF_STATE_DIM, EKF_STATE_DIM);
    matrixTranspose(H, HT, EKF_MEASUREMENT_GPS_DIM, EKF_STATE_DIM);
    matrixMultiply(HP, HT, HPHT, EKF_MEASUREMENT_GPS_DIM, EKF_STATE_DIM, EKF_MEASUREMENT_GPS_DIM);
    matrixAdd(HPHT, R, S, EKF_MEASUREMENT_GPS_DIM, EKF_MEASUREMENT_GPS_DIM);
    
    // 卡尔曼增益 K = P * H^T * S^(-1)
    float Sinv[EKF_MEASUREMENT_GPS_DIM * EKF_MEASUREMENT_GPS_DIM];
    if (matrixInvert(S, Sinv, EKF_MEASUREMENT_GPS_DIM)) {
        float PHT[EKF_STATE_DIM * EKF_MEASUREMENT_GPS_DIM];
        float K[EKF_STATE_DIM * EKF_MEASUREMENT_GPS_DIM];
        
        matrixMultiply(ekfState.P, HT, PHT, EKF_STATE_DIM, EKF_STATE_DIM, EKF_MEASUREMENT_GPS_DIM);
        matrixMultiply(PHT, Sinv, K, EKF_STATE_DIM, EKF_MEASUREMENT_GPS_DIM, EKF_MEASUREMENT_GPS_DIM);
        
        // 状态更新 x = x + K * innovation
        float Ky[EKF_STATE_DIM];
        matrixMultiply(K, innovation, Ky, EKF_STATE_DIM, EKF_MEASUREMENT_GPS_DIM, 1);
        matrixAdd(ekfState.x, Ky, ekfState.x, EKF_STATE_DIM, 1);
        
        // 协方差更新 P = (I - K * H) * P
        float KH[EKF_STATE_DIM * EKF_STATE_DIM];
        float I[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
        float IKH[EKF_STATE_DIM * EKF_STATE_DIM];
        
        // 单位矩阵
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            I[MATRIX_INDEX(i, i, EKF_STATE_DIM)] = 1.0f;
        }
        
        matrixMultiply(K, H, KH, EKF_STATE_DIM, EKF_MEASUREMENT_GPS_DIM, EKF_STATE_DIM);
        matrixSubtract(I, KH, IKH, EKF_STATE_DIM, EKF_STATE_DIM);
        
        float newP[EKF_STATE_DIM * EKF_STATE_DIM];
        matrixMultiply(IKH, ekfState.P, newP, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);
        memcpy(ekfState.P, newP, sizeof(newP));
        
        // 归一化航向角
        ekfState.x[4] = normalizeAngle(ekfState.x[4]);
    }
}

void EKFVehicleTracker::updateWithIMU(const IMUData& imuData) {
    if (!imuData.valid) return;
    
    // IMU提供加速度和角速度测量
    // 这里简化处理，主要用角速度更新航向角速度
    float measuredHeadingRate = imuData.gyro[2]; // Z轴角速度
    
    // 测量向量 z = [heading_rate]
    float z[1] = {measuredHeadingRate};
    
    // 观测矩阵 H (观测航向角速度)
    float H[1 * EKF_STATE_DIM] = {0};
    H[MATRIX_INDEX(0, 5, EKF_STATE_DIM)] = 1.0f; // 观测heading_rate
    
    // 测量噪声协方差
    float R[1] = {config.imuNoiseGyro};
    
    // 预测测量值
    float h[1] = {ekfState.x[5]};
    
    // 创新
    float innovation[1] = {z[0] - h[0]};
    
    // 简化的1D更新
    float HP = H[5] * ekfState.P[MATRIX_INDEX(5, 5, EKF_STATE_DIM)];
    float S = HP * H[5] + R[0];
    
    if (S > 0.001f) {
        float K[EKF_STATE_DIM];
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            K[i] = ekfState.P[MATRIX_INDEX(i, 5, EKF_STATE_DIM)] * H[5] / S;
        }
        
        // 状态更新
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            ekfState.x[i] += K[i] * innovation[0];
        }
        
        // 协方差更新 (简化)
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            for (int j = 0; j < EKF_STATE_DIM; j++) {
                ekfState.P[MATRIX_INDEX(i, j, EKF_STATE_DIM)] -= 
                    K[i] * H[5] * ekfState.P[MATRIX_INDEX(5, j, EKF_STATE_DIM)];
            }
        }
        
        ekfState.x[4] = normalizeAngle(ekfState.x[4]);
    }
}

void EKFVehicleTracker::updateWithMag(const MagData& magData) {
    if (!magData.valid) return;
    
    // 计算磁北航向
    float magHeading = atan2(magData.mag[1], magData.mag[0]);
    magHeading = normalizeAngle(magHeading);
    
    // 与当前航向的差值
    float headingDiff = magHeading - ekfState.x[4];
    headingDiff = normalizeAngle(headingDiff);
    
    // 如果差值较小，用于校正航向
    if (fabs(headingDiff) < 0.5f) { // 约30度以内
        float weight = 0.1f; // 地磁计权重较小
        ekfState.x[4] += weight * headingDiff;
        ekfState.x[4] = normalizeAngle(ekfState.x[4]);
    }
}

// 基础矩阵运算函数
void EKFVehicleTracker::matrixMultiply(const float* A, const float* B, float* C, 
                                       int rowsA, int colsA, int colsB) {
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            C[MATRIX_INDEX(i, j, colsB)] = 0;
            for (int k = 0; k < colsA; k++) {
                C[MATRIX_INDEX(i, j, colsB)] += 
                    A[MATRIX_INDEX(i, k, colsA)] * B[MATRIX_INDEX(k, j, colsB)];
            }
        }
    }
}

void EKFVehicleTracker::matrixTranspose(const float* A, float* AT, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            AT[MATRIX_INDEX(j, i, rows)] = A[MATRIX_INDEX(i, j, cols)];
        }
    }
}

void EKFVehicleTracker::matrixAdd(const float* A, const float* B, float* C, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        C[i] = A[i] + B[i];
    }
}

void EKFVehicleTracker::matrixSubtract(const float* A, const float* B, float* C, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        C[i] = A[i] - B[i];
    }
}

bool EKFVehicleTracker::matrixInvert(const float* A, float* Ainv, int n) {
    // 简化的2x2矩阵求逆 (主要用于GPS更新)
    if (n == 2) {
        float det = A[0] * A[3] - A[1] * A[2];
        if (fabs(det) < 1e-6f) return false;
        
        Ainv[0] = A[3] / det;
        Ainv[1] = -A[1] / det;
        Ainv[2] = -A[2] / det;
        Ainv[3] = A[0] / det;
        return true;
    }
    return false; // 更高维度的矩阵求逆需要更复杂的算法
}

void EKFVehicleTracker::latLngToXY(double lat, double lng, float& x, float& y) {
    if (!hasOrigin) {
        x = y = 0;
        return;
    }
    
    // 简化的平面投影 (适用于小范围)
    double deltaLat = (lat - originLat) * DEG_TO_RAD;
    double deltaLng = (lng - originLng) * DEG_TO_RAD;
    
    y = deltaLat * EARTH_RADIUS;
    x = deltaLng * EARTH_RADIUS * cos(originLat * DEG_TO_RAD);
}

void EKFVehicleTracker::xyToLatLng(float x, float y, double& lat, double& lng) {
    if (!hasOrigin) {
        lat = lng = 0;
        return;
    }
    
    double deltaLat = y / EARTH_RADIUS;
    double deltaLng = x / (EARTH_RADIUS * cos(originLat * DEG_TO_RAD));
    
    lat = originLat + deltaLat * RAD_TO_DEG;
    lng = originLng + deltaLng * RAD_TO_DEG;
}

float EKFVehicleTracker::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void EKFVehicleTracker::updatePositionFromState() {
    // 从EKF状态更新Position结构
    xyToLatLng(ekfState.x[0], ekfState.x[1], currentPosition.lat, currentPosition.lng);
    
    currentPosition.heading = ekfState.x[4] * RAD_TO_DEG;
    if (currentPosition.heading < 0) currentPosition.heading += 360.0f;
    
    currentPosition.speed = sqrt(ekfState.x[2] * ekfState.x[2] + ekfState.x[3] * ekfState.x[3]);
    
    // 位置精度估计 (基于协方差矩阵)
    float posVariance = ekfState.P[MATRIX_INDEX(0, 0, EKF_STATE_DIM)] + 
                       ekfState.P[MATRIX_INDEX(1, 1, EKF_STATE_DIM)];
    currentPosition.accuracy = sqrt(posVariance);
    
    currentPosition.timestamp = millis();
    currentPosition.valid = ekfState.initialized;
}

// 公共接口实现
Position EKFVehicleTracker::getPosition() {
    return currentPosition;
}

bool EKFVehicleTracker::isInitialized() {
    return ekfState.initialized;
}

float EKFVehicleTracker::getPositionAccuracy() {
    return currentPosition.accuracy;
}

float EKFVehicleTracker::getVelocity() {
    return currentPosition.speed;
}

float EKFVehicleTracker::getHeading() {
    return currentPosition.heading;
}

float EKFVehicleTracker::getHeadingRate() {
    return ekfState.x[5] * RAD_TO_DEG; // 转换为度/秒
}

void EKFVehicleTracker::getStateVector(float state[EKF_STATE_DIM]) {
    memcpy(state, ekfState.x, sizeof(ekfState.x));
}

void EKFVehicleTracker::getCovarianceMatrix(float covariance[EKF_STATE_DIM * EKF_STATE_DIM]) {
    memcpy(covariance, ekfState.P, sizeof(ekfState.P));
}
