#include "EKFVehicleTracker.h"

EKFVehicleTracker::EKFVehicleTracker(IIMUProvider* imu, double initLat, double initLng) 
    : imuProvider(imu), gpsProvider(nullptr), magProvider(nullptr),
      originLat(initLat), originLng(initLng), debugEnabled(false),
      displacementOriginLat(initLat), displacementOriginLng(initLng), 
      hasDisplacementOrigin(false) {
    
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
    
    // 设置位移起始点
    displacementOriginLat = originLat;
    displacementOriginLng = originLng;
    hasDisplacementOrigin = true;
    
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
    
    // 调试输出GPS数据
    if (debugEnabled) {
        Serial.printf("[EKFVehicleTracker] 📡 GPS数据更新\n");
        Serial.printf("[EKFVehicleTracker] GPS位置: %.6f, %.6f (精度: %.1fm)\n", 
                     gpsData.lat, gpsData.lng, gpsData.accuracy);
    }
    
    // 将GPS经纬度转换为本地坐标
    float measX, measY;
    latLngToXY(gpsData.lat, gpsData.lng, measX, measY);
    
    // 调试输出坐标转换结果
    if (debugEnabled) {
        Serial.printf("[EKFVehicleTracker] 本地坐标: X=%.3f, Y=%.3f\n", measX, measY);
        Serial.printf("[EKFVehicleTracker] 当前预测位置: X=%.3f, Y=%.3f\n", 
                     ekfState.x[0], ekfState.x[1]);
    }
    
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
    
    // 调试输出创新值
    if (debugEnabled) {
        Serial.printf("[EKFVehicleTracker] GPS创新值: X=%.3f, Y=%.3f\n", 
                     innovation[0], innovation[1]);
        float innovationDistance = sqrt(innovation[0]*innovation[0] + innovation[1]*innovation[1]);
        Serial.printf("[EKFVehicleTracker] 创新距离: %.3fm\n", innovationDistance);
    }
    
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
    
    // 调试输出原始IMU数据
    if (debugEnabled) {
        Serial.printf("[EKFVehicleTracker] 🔄 IMU数据更新\n");
        Serial.printf("[EKFVehicleTracker] 加速度: X=%.3f, Y=%.3f, Z=%.3f m/s²\n", 
                     imuData.accel[0], imuData.accel[1], imuData.accel[2]);
        Serial.printf("[EKFVehicleTracker] 陀螺仪: X=%.3f, Y=%.3f, Z=%.3f rad/s\n", 
                     imuData.gyro[0], imuData.gyro[1], imuData.gyro[2]);
    }
    
    // ========== 1. 零速检测 (ZUPT) ==========
    float totalAccel = sqrt(imuData.accel[0] * imuData.accel[0] + 
                           imuData.accel[1] * imuData.accel[1] + 
                           imuData.accel[2] * imuData.accel[2]);
    float totalGyro = sqrt(imuData.gyro[0] * imuData.gyro[0] + 
                          imuData.gyro[1] * imuData.gyro[1] + 
                          imuData.gyro[2] * imuData.gyro[2]);
    
    // 静止检测条件
    bool isStationary = (abs(totalAccel - 9.8f) < 0.3f) && (totalGyro < 0.05f);
    
    if (isStationary) {
        // 零速更新：强制设置速度为0
        ekfState.x[2] = 0.0f;  // vx = 0
        ekfState.x[3] = 0.0f;  // vy = 0
        ekfState.x[5] = 0.0f;  // heading_rate = 0
        
        // 重置速度相关的协方差（提高置信度）
        ekfState.P[MATRIX_INDEX(2, 2, EKF_STATE_DIM)] = 0.01f;  // 速度方差很小
        ekfState.P[MATRIX_INDEX(3, 3, EKF_STATE_DIM)] = 0.01f;
        ekfState.P[MATRIX_INDEX(5, 5, EKF_STATE_DIM)] = 0.001f; // 角速度方差很小
        
        if (debugEnabled) {
            Serial.printf("[EKFVehicleTracker] 🛑 ZUPT: 检测到静止状态，速度重置为0\n");
        }
        return;
    }
    
    // ========== 2. GPS状态检测和自适应噪声 ==========
    bool hasGPS = currentPosition.sources.hasGPS;
    float noiseMultiplier = hasGPS ? 1.0f : 5.0f;  // 无GPS时增加5倍噪声
    
    // ========== 3. 车辆运动约束 ==========
    float forwardAccel = imuData.accel[0];  // 前进方向加速度
    float lateralAccel = imuData.accel[1];  // 侧向加速度
    
    // 摩托车物理约束
    float maxForwardAccel = 8.0f;   // 最大前进加速度 8m/s²
    float maxLateralAccel = 6.0f;   // 最大侧向加速度 6m/s²（转弯）
    float maxBrakeAccel = 12.0f;    // 最大制动减速度 12m/s²
    
    // 限制加速度在物理可能范围内
    if (forwardAccel > maxForwardAccel) forwardAccel = maxForwardAccel;
    if (forwardAccel < -maxBrakeAccel) forwardAccel = -maxBrakeAccel;
    if (abs(lateralAccel) > maxLateralAccel) {
        lateralAccel = (lateralAccel > 0) ? maxLateralAccel : -maxLateralAccel;
    }
    
    // ========== 4. EKF更新过程 ==========
    float measuredHeadingRate = imuData.gyro[2]; // Z轴角速度
    
    // 限制角速度在合理范围内
    float maxHeadingRate = 3.0f;  // 最大角速度 3 rad/s
    if (abs(measuredHeadingRate) > maxHeadingRate) {
        measuredHeadingRate = (measuredHeadingRate > 0) ? maxHeadingRate : -maxHeadingRate;
    }
    
    // 测量向量 z = [heading_rate]
    float z[1] = {measuredHeadingRate};
    
    // 观测矩阵 H (观测航向角速度)
    float H[1 * EKF_STATE_DIM] = {0};
    H[MATRIX_INDEX(0, 5, EKF_STATE_DIM)] = 1.0f; // 观测heading_rate
    
    // 自适应测量噪声协方差
    float R[1] = {config.imuNoiseGyro * noiseMultiplier};
    
    // 预测测量值
    float h[1] = {ekfState.x[5]};
    
    // 创新
    float innovation[1] = {z[0] - h[0]};
    
    // 调试输出EKF计算过程
    if (debugEnabled) {
        Serial.printf("[EKFVehicleTracker] 测量航向角速度: %.3f rad/s\n", measuredHeadingRate);
        Serial.printf("[EKFVehicleTracker] 预测航向角速度: %.3f rad/s\n", h[0]);
        Serial.printf("[EKFVehicleTracker] 创新值: %.3f rad/s\n", innovation[0]);
        Serial.printf("[EKFVehicleTracker] GPS状态: %s, 噪声倍数: %.1f\n", 
                     hasGPS ? "有效" : "无效", noiseMultiplier);
        Serial.printf("[EKFVehicleTracker] 加速度约束: 前进%.2f, 侧向%.2f m/s²\n", 
                     forwardAccel, lateralAccel);
    }
    
    // 简化的1D更新
    float HP = H[5] * ekfState.P[MATRIX_INDEX(5, 5, EKF_STATE_DIM)];
    float S = HP * H[5] + R[0];
    
    if (S > 0.001f) {
        float K[EKF_STATE_DIM];
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            K[i] = ekfState.P[MATRIX_INDEX(i, 5, EKF_STATE_DIM)] * H[5] / S;
        }
        
        // 保存旧状态用于调试输出
        float oldState[EKF_STATE_DIM];
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            oldState[i] = ekfState.x[i];
        }
        
        // 状态更新
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            ekfState.x[i] += K[i] * innovation[0];
        }
        
        // ========== 5. 状态约束（防止发散） ==========
        
        // 位置约束 (无GPS时防止过度漂移)
        if (!hasGPS) {
            float maxDrift = 5000.0f;  // 最大漂移5km
            if (abs(ekfState.x[0]) > maxDrift) {
                ekfState.x[0] *= 0.8f; // 位置衰减
                if (debugEnabled) Serial.printf("[EKFVehicleTracker] ⚠️ X位置漂移过大，衰减处理\n");
            }
            if (abs(ekfState.x[1]) > maxDrift) {
                ekfState.x[1] *= 0.8f;
                if (debugEnabled) Serial.printf("[EKFVehicleTracker] ⚠️ Y位置漂移过大，衰减处理\n");
            }
        }
        
        // 速度约束 (摩托车物理限制)
        float maxSpeed = 83.0f;  // 300km/h
        float currentSpeed = sqrt(ekfState.x[2] * ekfState.x[2] + ekfState.x[3] * ekfState.x[3]);
        if (currentSpeed > maxSpeed) {
            float scale = maxSpeed / currentSpeed;
            ekfState.x[2] *= scale;
            ekfState.x[3] *= scale;
            if (debugEnabled) {
                Serial.printf("[EKFVehicleTracker] ⚠️ 速度超限: %.1f km/h -> %.1f km/h\n", 
                             currentSpeed * 3.6f, maxSpeed * 3.6f);
            }
        }
        
        // 车辆非完整性约束：限制侧向速度
        if (!hasGPS) {
            float maxLateralSpeed = 5.0f;  // 最大侧向速度 5m/s
            if (abs(ekfState.x[3]) > maxLateralSpeed) {
                ekfState.x[3] *= 0.5f;  // 强制减少侧向速度
                if (debugEnabled) Serial.printf("[EKFVehicleTracker] ⚠️ 侧向速度过大，约束处理\n");
            }
        }
        
        // 航向约束
        ekfState.x[4] = normalizeAngle(ekfState.x[4]);
        
        // 角速度约束
        if (abs(ekfState.x[5]) > maxHeadingRate) {
            ekfState.x[5] = ekfState.x[5] > 0 ? maxHeadingRate : -maxHeadingRate;
        }
        
        // 调试输出状态更新结果
        if (debugEnabled) {
            Serial.printf("[EKFVehicleTracker] 状态更新: 位置(%.1f,%.1f) -> (%.1f,%.1f)\n", 
                         oldState[0], oldState[1], ekfState.x[0], ekfState.x[1]);
            Serial.printf("[EKFVehicleTracker] 速度更新: (%.2f,%.2f) -> (%.2f,%.2f) m/s\n", 
                         oldState[2], oldState[3], ekfState.x[2], ekfState.x[3]);
            Serial.printf("[EKFVehicleTracker] 航向更新: %.1f° -> %.1f°\n", 
                         oldState[4] * 180.0f / M_PI, ekfState.x[4] * 180.0f / M_PI);
            
            float finalSpeed = sqrt(ekfState.x[2] * ekfState.x[2] + ekfState.x[3] * ekfState.x[3]);
            Serial.printf("[EKFVehicleTracker] 当前速度: %.1f m/s (%.1f km/h)\n", 
                         finalSpeed, finalSpeed * 3.6f);
        }
        
        // 协方差更新 (简化)
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            for (int j = 0; j < EKF_STATE_DIM; j++) {
                ekfState.P[MATRIX_INDEX(i, j, EKF_STATE_DIM)] -= 
                    K[i] * H[5] * ekfState.P[MATRIX_INDEX(5, j, EKF_STATE_DIM)];
            }
        }
        
        // ========== 6. 协方差约束（防止过度收敛或发散） ==========
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            float minVar = 0.001f;  // 最小方差
            float maxVar = hasGPS ? 100.0f : 10000.0f;  // 最大方差（无GPS时更大）
            
            if (ekfState.P[MATRIX_INDEX(i, i, EKF_STATE_DIM)] < minVar) {
                ekfState.P[MATRIX_INDEX(i, i, EKF_STATE_DIM)] = minVar;
            }
            if (ekfState.P[MATRIX_INDEX(i, i, EKF_STATE_DIM)] > maxVar) {
                ekfState.P[MATRIX_INDEX(i, i, EKF_STATE_DIM)] = maxVar;
            }
        }
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

void EKFVehicleTracker::calculateRelativeDisplacement() {
    if (!hasDisplacementOrigin || !ekfState.initialized) {
        currentPosition.displacement.x = 0;
        currentPosition.displacement.y = 0;
        currentPosition.displacement.distance = 0;
        currentPosition.displacement.bearing = 0;
        return;
    }
    
    // 计算经纬度差值
    double deltaLat = currentPosition.lat - displacementOriginLat;
    double deltaLng = currentPosition.lng - displacementOriginLng;
    
    // 转换为米制坐标 (简化的平面投影)
    // Y轴：南北方向 (+北/-南)
    float deltaY = deltaLat * 6371000.0 * M_PI / 180.0;
    
    // X轴：东西方向 (+东/-西)
    float deltaX = deltaLng * 6371000.0 * cos(displacementOriginLat * M_PI / 180.0) * M_PI / 180.0;
    
    currentPosition.displacement.x = deltaX;
    currentPosition.displacement.y = deltaY;
    
    // 计算直线距离
    currentPosition.displacement.distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    
    // 计算方位角 (从起始点指向当前点的角度)
    if (currentPosition.displacement.distance > 0.1f) { // 距离大于10cm才计算方位角
        float bearing = atan2(deltaX, deltaY) * 180.0 / M_PI;
        if (bearing < 0) bearing += 360.0f;
        currentPosition.displacement.bearing = bearing;
    } else {
        currentPosition.displacement.bearing = 0;
    }
    
    if (debugEnabled) {
        Serial.printf("[EKFVehicleTracker] 📍 相对位移: X=%.2fm(东西), Y=%.2fm(南北), 距离=%.2fm, 方位=%.1f°\n", 
                     currentPosition.displacement.x, 
                     currentPosition.displacement.y,
                     currentPosition.displacement.distance,
                     currentPosition.displacement.bearing);
    }
}

void EKFVehicleTracker::resetOrigin() {
    if (ekfState.initialized) {
        displacementOriginLat = currentPosition.lat;
        displacementOriginLng = currentPosition.lng;
        hasDisplacementOrigin = true;
        
        if (debugEnabled) {
            Serial.printf("[EKFVehicleTracker] 🔄 起始点重置为: (%.6f, %.6f)\n", 
                         displacementOriginLat, displacementOriginLng);
        }
    }
}

void EKFVehicleTracker::setOrigin(double lat, double lng) {
    displacementOriginLat = lat;
    displacementOriginLng = lng;
    hasDisplacementOrigin = true;
    
    if (debugEnabled) {
        Serial.printf("[EKFVehicleTracker] 🔄 起始点设置为: (%.6f, %.6f)\n", 
                     displacementOriginLat, displacementOriginLng);
    }
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
    
    // 计算相对位移
    calculateRelativeDisplacement();
    
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
