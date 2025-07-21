#include "FusionLocation.h"
#include <math.h>

FusionLocation::FusionLocation(IIMUProvider* imu, double initLat, double initLng) 
    : imuProvider(imu), gpsProvider(nullptr), magProvider(nullptr),
      initialLat(initLat), initialLng(initLng) {
    
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
        filterState.initialized = true;
        filterState.lastUpdate = millis();
    }
    
    // 初始化当前位置
    currentPosition.lat = initialLat;
    currentPosition.lng = initialLng;
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
        // 使用加速度数据进行位置预测
        predictPosition(deltaTime);
        
        // 更新航向 (基于陀螺仪)
        filterState.heading += imuData.gyro[2] * deltaTime * RAD_TO_DEG;
        filterState.heading = constrainAngle(filterState.heading);
    }
    
    filterState.lastUpdate = currentTime;
}

void FusionLocation::updateWithGPS(const GPSData& gpsData) {
    if (!gpsData.valid) return;
    
    if (!filterState.initialized) {
        // 使用GPS数据初始化滤波器
        filterState.lat = gpsData.lat;
        filterState.lng = gpsData.lng;
        filterState.initialized = true;
        filterState.lastUpdate = millis();
        return;
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
    // 简单的位置预测 - 基于当前速度
    float distanceX = filterState.velX * deltaTime;
    float distanceY = filterState.velY * deltaTime;
    
    // 转换为经纬度偏移
    double latOffset = metersToLatitude(distanceY);
    double lngOffset = metersToLongitude(distanceX, filterState.lat);
    
    filterState.lat += latOffset;
    filterState.lng += lngOffset;
    
    // 简单的速度衰减 (模拟摩擦)
    filterState.velX *= 0.95f;
    filterState.velY *= 0.95f;
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
