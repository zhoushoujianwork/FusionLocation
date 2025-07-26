#include "FusionLocation.h"
#include <math.h>

FusionLocation::FusionLocation(IIMUProvider* imu, double initLat, double initLng) 
    : imuProvider(imu), gpsProvider(nullptr), magProvider(nullptr),
      initialLat(initLat), initialLng(initLng), debugEnabled(false),
      originLat(initLat), originLng(initLng), hasOrigin(false) {
    
    hasInitialPosition = (initLat != 0 || initLng != 0);
}

void FusionLocation::setGPSProvider(IGPSProvider* gps) {
    gpsProvider = gps;
}

void FusionLocation::setMagProvider(IMagProvider* mag) {
    magProvider = mag;
}

void FusionLocation::setInitialPosition(double lat, double lng) {
    initialLat = lat;
    initialLng = lng;
    hasInitialPosition = true;
    
    // 如果滤波器还未初始化，设置初始位置
    if (!filterState.initialized && hasInitialPosition) {
        filterState.lat = initialLat;
        filterState.lng = initialLng;
        filterState.initialized = true;
        filterState.lastUpdate = millis();
    }
}

void FusionLocation::begin() {
    // 初始化滤波器状态
    if (hasInitialPosition) {
        filterState.lat = initialLat;
        filterState.lng = initialLng;
        filterState.velX = 0.0f;  // 初始化速度为0
        filterState.velY = 0.0f;
        filterState.heading = 0.0f;
        filterState.initialized = true;
        filterState.lastUpdate = millis();
        
        // 设置起始点用于相对位移计算
        originLat = initialLat;
        originLng = initialLng;
        hasOrigin = true;
    }
    
    // 初始化当前位置
    currentPosition.lat = initialLat;
    currentPosition.lng = initialLng;
    currentPosition.speed = 0.0f;  // 初始速度为0
    currentPosition.heading = 0.0f;
    currentPosition.timestamp = millis();
    currentPosition.valid = hasInitialPosition;
}

void FusionLocation::update() {
    uint32_t currentTime = millis();
    
    // 检查IMU数据 (必需)
    if (imuProvider && imuProvider->isAvailable()) {
        IMUData imuData;
        if (imuProvider->getData(imuData)) {
            updateWithIMU(imuData);
            currentPosition.sources.hasIMU = true;
        }
    }
    
    // 检查GPS数据 (可选)
    if (gpsProvider && gpsProvider->isAvailable()) {
        GPSData gpsData;
        if (gpsProvider->getData(gpsData)) {
            updateWithGPS(gpsData);
            currentPosition.sources.hasGPS = true;
        }
    }
    
    // 检查地磁计数据 (可选)
    if (magProvider && magProvider->isAvailable()) {
        MagData magData;
        if (magProvider->getData(magData)) {
            updateWithMag(magData);
            currentPosition.sources.hasMag = true;
        }
    }
    
    // 更新位置估计
    updatePositionEstimate();
}

void FusionLocation::updateWithIMU(const IMUData& imuData) {
    if (!filterState.initialized) return;
    
    uint32_t currentTime = millis();
    float deltaTime = (currentTime - filterState.lastUpdate) / 1000.0f;
    
    if (deltaTime > 0.001f && deltaTime < 1.0f) { // 合理的时间间隔
        // 计算总加速度幅值
        float totalAccel = sqrt(imuData.accel[0] * imuData.accel[0] + 
                               imuData.accel[1] * imuData.accel[1] + 
                               imuData.accel[2] * imuData.accel[2]);
        
        // 检测是否接近静止状态 (总加速度接近重力9.8m/s²)
        bool isNearStatic = (abs(totalAccel - 9.8f) < 1.5f);
        
        // 重力补偿：估算重力在各轴的分量
        float gravityX = 0, gravityY = 0, gravityZ = 9.8f;
        
        if (isNearStatic) {
            // 静止时，当前加速度就是重力分量
            gravityX = imuData.accel[0];
            gravityY = imuData.accel[1];
            gravityZ = imuData.accel[2];
        } else {
            // 运动时，使用归一化的重力向量
            float norm = totalAccel;
            if (norm > 0.1f) {
                gravityX = (imuData.accel[0] / norm) * 9.8f;
                gravityY = (imuData.accel[1] / norm) * 9.8f;
                gravityZ = (imuData.accel[2] / norm) * 9.8f;
            }
        }
        
        // 去除重力分量，得到真实的线性加速度
        float linearAccelX = imuData.accel[0] - gravityX;
        float linearAccelY = imuData.accel[1] - gravityY;
        float linearAccelZ = imuData.accel[2] - gravityZ;
        
        // 调试输出
        if (debugEnabled) {
            Serial.printf("[FusionLocation] 原始加速度: X=%.3f, Y=%.3f, Z=%.3f m/s² (总=%.3f)\n", 
                         imuData.accel[0], imuData.accel[1], imuData.accel[2], totalAccel);
            Serial.printf("[FusionLocation] 重力分量: X=%.3f, Y=%.3f, Z=%.3f m/s²\n", 
                         gravityX, gravityY, gravityZ);
            Serial.printf("[FusionLocation] 线性加速度: X=%.3f, Y=%.3f, Z=%.3f m/s²\n", 
                         linearAccelX, linearAccelY, linearAccelZ);
            Serial.printf("[FusionLocation] 静止检测: %s\n", isNearStatic ? "是" : "否");
        }
        
        // 使用线性加速度进行运动检测
        float accelX = linearAccelX;
        float accelY = linearAccelY;
        
        // 如果线性加速度很小，认为是噪声
        if (abs(accelX) < 0.2f) accelX = 0;
        if (abs(accelY) < 0.2f) accelY = 0;
        
        // 限制加速度在合理范围内
        float maxAccel = 2.0f;  // 进一步降低最大加速度
        if (abs(accelX) > maxAccel) accelX = (accelX > 0) ? maxAccel : -maxAccel;
        if (abs(accelY) > maxAccel) accelY = (accelY > 0) ? maxAccel : -maxAccel;
        
        // 调试输出处理后的加速度
        if (debugEnabled) {
            Serial.printf("[FusionLocation] 处理后加速度: X=%.3f, Y=%.3f m/s²\n", accelX, accelY);
        }
        
        // 记录更新前的速度
        float oldVelX = filterState.velX;
        float oldVelY = filterState.velY;
        
        // 积分更新速度
        filterState.velX += accelX * deltaTime;
        filterState.velY += accelY * deltaTime;
        
        // 调试输出速度积分过程
        if (debugEnabled) {
            Serial.printf("[FusionLocation] 速度积分: dt=%.3fs, 加速度(%.3f,%.3f) -> 速度变化(%.3f,%.3f)\n", 
                         deltaTime, accelX, accelY, 
                         accelX * deltaTime, accelY * deltaTime);
            Serial.printf("[FusionLocation] 速度更新: (%.3f,%.3f) -> (%.3f,%.3f)\n", 
                         oldVelX, oldVelY, filterState.velX, filterState.velY);
        }
        
        // 限制速度在合理范围内 (现代摩托车最大速度约83m/s = 300km/h)
        float maxSpeed = 83.0f;  // 300km/h，现代摩托车的实际性能
        float currentSpeed = sqrt(filterState.velX * filterState.velX + filterState.velY * filterState.velY);
        if (currentSpeed > maxSpeed) {
            float scale = maxSpeed / currentSpeed;
            filterState.velX *= scale;
            filterState.velY *= scale;
            if (debugEnabled) {
                Serial.printf("[FusionLocation] 速度限制: %.3f m/s (%.0f km/h) -> %.3f m/s (%.0f km/h)\n", 
                             currentSpeed, currentSpeed * 3.6f, maxSpeed, maxSpeed * 3.6f);
            }
        }
        
        // 使用加速度数据进行位置预测
        predictPosition(deltaTime);
        
        // 更新航向 (基于陀螺仪)
        float oldHeading = filterState.heading;
        filterState.heading += imuData.gyro[2] * deltaTime * RAD_TO_DEG;
        filterState.heading = constrainAngle(filterState.heading);
        
        // 调试输出航向变化
        if (debugEnabled && abs(imuData.gyro[2]) > 0.01f) {
            Serial.printf("[FusionLocation] 航向更新: %.1f° -> %.1f° (陀螺仪Z=%.3f rad/s)\n", 
                         oldHeading, filterState.heading, imuData.gyro[2]);
        }
    }
    
    filterState.lastUpdate = currentTime;
}

void FusionLocation::updateWithGPS(const GPSData& gpsData) {
    if (!gpsData.valid) return;
    
    if (!filterState.initialized) {
        // 使用GPS数据初始化滤波器
        filterState.lat = gpsData.lat;
        filterState.lng = gpsData.lng;
        filterState.velX = 0.0f;  // 初始化速度为0
        filterState.velY = 0.0f;
        filterState.initialized = true;
        filterState.lastUpdate = millis();
        return;
    }
    
    // 检查GPS位置跳跃是否过大
    double latDiff = abs(gpsData.lat - filterState.lat);
    double lngDiff = abs(gpsData.lng - filterState.lng);
    
    // 如果GPS位置跳跃过大，可能是信号异常，重置速度
    if (latDiff > 0.001 || lngDiff > 0.001) { // 约100米的跳跃
        filterState.velX = 0.0f;
        filterState.velY = 0.0f;
    }
    
    // GPS数据融合 - 简单的加权平均
    float gpsWeight = 0.3f; // GPS权重
    if (gpsData.accuracy < 10.0f) {
        gpsWeight = 0.7f; // 高精度GPS给更高权重
    }
    
    filterState.lat = filterState.lat * (1 - gpsWeight) + gpsData.lat * gpsWeight;
    filterState.lng = filterState.lng * (1 - gpsWeight) + gpsData.lng * gpsWeight;
    
    // 更新高度
    currentPosition.altitude = gpsData.altitude;
    currentPosition.accuracy = gpsData.accuracy;
}

void FusionLocation::updateWithMag(const MagData& magData) {
    if (!magData.valid) return;
    
    // 计算磁北航向
    float magHeading = atan2(magData.mag[1], magData.mag[0]) * RAD_TO_DEG;
    magHeading = constrainAngle(magHeading);
    
    // 与IMU航向融合
    if (filterState.initialized) {
        float magWeight = 0.1f; // 地磁计权重较小，主要用于校正漂移
        filterState.heading = filterState.heading * (1 - magWeight) + magHeading * magWeight;
        filterState.heading = constrainAngle(filterState.heading);
    }
}

void FusionLocation::predictPosition(float deltaTime) {
    // 检查速度是否异常
    float currentSpeed = sqrt(filterState.velX * filterState.velX + filterState.velY * filterState.velY);
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 位置预测开始: 当前速度=%.3f m/s (%.3f,%.3f)\n", 
                     currentSpeed, filterState.velX, filterState.velY);
    }
    
    // 如果速度异常大，重置速度 (超过150m/s = 540km/h才认为异常)
    if (currentSpeed > 150.0f) {
        Serial.printf("[FusionLocation] ❌ 异常速度检测: %.2f m/s (%.0f km/h), 重置为0\n", 
                     currentSpeed, currentSpeed * 3.6f);
        filterState.velX = 0;
        filterState.velY = 0;
        return;
    }
    
    // 如果没有GPS数据，适度减少位置更新（不要太激进）
    bool hasRecentGPS = currentPosition.sources.hasGPS;
    float positionUpdateFactor = hasRecentGPS ? 1.0f : 0.5f; // 无GPS时用50%而不是10%
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] GPS状态: %s, 位置更新系数: %.1f\n", 
                     hasRecentGPS ? "有效" : "无效", positionUpdateFactor);
    }
    
    // 简单的位置预测 - 基于当前速度
    float distanceX = filterState.velX * deltaTime * positionUpdateFactor;
    float distanceY = filterState.velY * deltaTime * positionUpdateFactor;
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 位移计算: dt=%.3fs, 距离(%.3f,%.3f)m\n", 
                     deltaTime, distanceX, distanceY);
    }
    
    // 转换为经纬度偏移
    double latOffset = metersToLatitude(distanceY);
    double lngOffset = metersToLongitude(distanceX, filterState.lat);
    
    // 记录位置更新前的坐标
    double oldLat = filterState.lat;
    double oldLng = filterState.lng;
    
    filterState.lat += latOffset;
    filterState.lng += lngOffset;
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 位置更新: (%.6f,%.6f) -> (%.6f,%.6f)\n", 
                     oldLat, oldLng, filterState.lat, filterState.lng);
        Serial.printf("[FusionLocation] 经纬度偏移: lat+%.8f, lng+%.8f\n", 
                     latOffset, lngOffset);
    }
    
    // 更温和的速度衰减
    float decayFactor = hasRecentGPS ? 0.95f : 0.90f; // 减少衰减强度
    float oldVelX = filterState.velX;
    float oldVelY = filterState.velY;
    
    filterState.velX *= decayFactor;
    filterState.velY *= decayFactor;
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 速度衰减: 系数=%.2f, (%.3f,%.3f) -> (%.3f,%.3f)\n", 
                     decayFactor, oldVelX, oldVelY, filterState.velX, filterState.velY);
    }
    
    // 如果速度很小，直接设为0
    if (abs(filterState.velX) < 0.05f) filterState.velX = 0;  // 降低阈值
    if (abs(filterState.velY) < 0.05f) filterState.velY = 0;
    
    float finalSpeed = sqrt(filterState.velX * filterState.velX + filterState.velY * filterState.velY);
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 最终速度: %.3f m/s\n", finalSpeed);
        Serial.println("----------------------------------------");
    }
}

void FusionLocation::updatePositionEstimate() {
    if (!filterState.initialized) return;
    
    // 更新当前位置
    currentPosition.lat = filterState.lat;
    currentPosition.lng = filterState.lng;
    currentPosition.heading = filterState.heading;
    currentPosition.speed = sqrt(filterState.velX * filterState.velX + 
                                filterState.velY * filterState.velY);
    currentPosition.timestamp = millis();
    currentPosition.valid = true;
    
    // 计算相对位移
    calculateRelativeDisplacement();
    
    // 调试输出最终位置和速度信息
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 🎯 最终输出: 位置(%.6f,%.6f), 速度=%.3f m/s, 航向=%.1f°\n", 
                     currentPosition.lat, currentPosition.lng, 
                     currentPosition.speed, currentPosition.heading);
        Serial.printf("[FusionLocation] 🎯 速度分量: Vx=%.3f, Vy=%.3f m/s\n", 
                     filterState.velX, filterState.velY);
    }
    
    // 估算精度 - 基于数据源
    float baseAccuracy = 50.0f; // 基础精度50米
    if (currentPosition.sources.hasGPS) {
        baseAccuracy = min(baseAccuracy, currentPosition.accuracy);
    }
    if (currentPosition.sources.hasIMU) {
        baseAccuracy *= 0.8f; // IMU可以提高精度
    }
    if (currentPosition.sources.hasMag) {
        baseAccuracy *= 0.9f; // 地磁计可以稍微提高精度
    }
    
    currentPosition.accuracy = baseAccuracy;
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 🎯 数据源状态: GPS=%s, IMU=%s, MAG=%s, 精度=%.1fm\n", 
                     currentPosition.sources.hasGPS ? "✓" : "✗",
                     currentPosition.sources.hasIMU ? "✓" : "✗", 
                     currentPosition.sources.hasMag ? "✓" : "✗",
                     currentPosition.accuracy);
        Serial.println("========================================");
    }
}

Position FusionLocation::getPosition() {
    return currentPosition;
}

bool FusionLocation::isInitialized() {
    return filterState.initialized;
}

float FusionLocation::getPositionAccuracy() {
    return currentPosition.accuracy;
}

float FusionLocation::getHeading() {
    return currentPosition.heading;
}

float FusionLocation::getSpeed() {
    return currentPosition.speed;
}

uint32_t FusionLocation::getLastUpdateTime() {
    return filterState.lastUpdate;
}

void FusionLocation::resetOrigin() {
    if (filterState.initialized) {
        originLat = filterState.lat;
        originLng = filterState.lng;
        hasOrigin = true;
        
        if (debugEnabled) {
            Serial.printf("[FusionLocation] 🔄 起始点重置为: (%.6f, %.6f)\n", originLat, originLng);
        }
    }
}

void FusionLocation::setOrigin(double lat, double lng) {
    originLat = lat;
    originLng = lng;
    hasOrigin = true;
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 🔄 起始点设置为: (%.6f, %.6f)\n", originLat, originLng);
    }
}

// 工具函数实现
float FusionLocation::constrainAngle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

double FusionLocation::metersToLatitude(float meters) {
    return meters / EARTH_RADIUS * RAD_TO_DEG;
}

double FusionLocation::metersToLongitude(float meters, double lat) {
    return meters / (EARTH_RADIUS * cos(lat * DEG_TO_RAD)) * RAD_TO_DEG;
}

void FusionLocation::calculateRelativeDisplacement() {
    if (!hasOrigin || !filterState.initialized) {
        currentPosition.displacement.x = 0;
        currentPosition.displacement.y = 0;
        currentPosition.displacement.distance = 0;
        currentPosition.displacement.bearing = 0;
        return;
    }
    
    // 计算经纬度差值
    double deltaLat = filterState.lat - originLat;
    double deltaLng = filterState.lng - originLng;
    
    // 转换为米制坐标 (简化的平面投影)
    // Y轴：南北方向 (+北/-南)
    float deltaY = deltaLat * EARTH_RADIUS * DEG_TO_RAD;
    
    // X轴：东西方向 (+东/-西)
    float deltaX = deltaLng * EARTH_RADIUS * cos(originLat * DEG_TO_RAD) * DEG_TO_RAD;
    
    currentPosition.displacement.x = deltaX;
    currentPosition.displacement.y = deltaY;
    
    // 计算直线距离
    currentPosition.displacement.distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    
    // 计算方位角 (从起始点指向当前点的角度)
    if (currentPosition.displacement.distance > 0.1f) { // 距离大于10cm才计算方位角
        float bearing = atan2(deltaX, deltaY) * RAD_TO_DEG;
        if (bearing < 0) bearing += 360.0f;
        currentPosition.displacement.bearing = bearing;
    } else {
        currentPosition.displacement.bearing = 0;
    }
    
    if (debugEnabled) {
        Serial.printf("[FusionLocation] 📍 相对位移: X=%.2fm(东西), Y=%.2fm(南北), 距离=%.2fm, 方位=%.1f°\n", 
                     currentPosition.displacement.x, 
                     currentPosition.displacement.y,
                     currentPosition.displacement.distance,
                     currentPosition.displacement.bearing);
        
        // 添加方向指示
        String direction = "";
        if (currentPosition.displacement.distance > 0.1f) {
            if (currentPosition.displacement.y > 0.1f) direction += "北";
            else if (currentPosition.displacement.y < -0.1f) direction += "南";
            
            if (currentPosition.displacement.x > 0.1f) direction += "东";
            else if (currentPosition.displacement.x < -0.1f) direction += "西";
            
            if (direction.length() == 0) direction = "原点";
        } else {
            direction = "原点";
        }
        
        Serial.printf("[FusionLocation] 📍 移动方向: %s\n", direction.c_str());
    }
}
