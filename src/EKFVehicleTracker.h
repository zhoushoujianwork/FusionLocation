#ifndef EKF_VEHICLE_TRACKER_H
#define EKF_VEHICLE_TRACKER_H

#include "FusionLocation.h"
#include <math.h>

// EKF状态向量维度 (x, y, vx, vy, heading, heading_rate)
#define EKF_STATE_DIM 6
#define EKF_MEASUREMENT_GPS_DIM 2    // GPS测量 (x, y)
#define EKF_MEASUREMENT_IMU_DIM 3    // IMU测量 (ax, ay, gz)

// 矩阵操作宏定义
#define MATRIX_INDEX(row, col, cols) ((row) * (cols) + (col))

// EKF车辆追踪器配置
struct EKFConfig {
    // 过程噪声协方差
    float processNoisePos;      // 位置过程噪声 (m²)
    float processNoiseVel;      // 速度过程噪声 (m²/s²)
    float processNoiseHeading;  // 航向过程噪声 (rad²)
    float processNoiseHeadingRate; // 航向角速度过程噪声 (rad²/s²)
    
    // 测量噪声协方差
    float gpsNoisePos;          // GPS位置测量噪声 (m²)
    float imuNoiseAccel;        // IMU加速度测量噪声 (m²/s⁴)
    float imuNoiseGyro;         // IMU角速度测量噪声 (rad²/s²)
    
    // 初始状态协方差
    float initialPosVariance;   // 初始位置方差 (m²)
    float initialVelVariance;   // 初始速度方差 (m²/s²)
    float initialHeadingVariance; // 初始航向方差 (rad²)
    
    // 默认配置
    EKFConfig() {
        processNoisePos = 0.1f;
        processNoiseVel = 0.5f;
        processNoiseHeading = 0.01f;
        processNoiseHeadingRate = 0.1f;
        
        gpsNoisePos = 25.0f;        // GPS精度约5m
        imuNoiseAccel = 0.1f;
        imuNoiseGyro = 0.01f;
        
        initialPosVariance = 100.0f;
        initialVelVariance = 10.0f;
        initialHeadingVariance = 0.1f;
    }
};

// EKF状态结构
struct EKFState {
    float x[EKF_STATE_DIM];     // 状态向量 [x, y, vx, vy, heading, heading_rate]
    float P[EKF_STATE_DIM * EKF_STATE_DIM]; // 状态协方差矩阵
    uint32_t lastUpdate;        // 上次更新时间
    bool initialized;           // 是否已初始化
    
    EKFState() {
        memset(x, 0, sizeof(x));
        memset(P, 0, sizeof(P));
        lastUpdate = 0;
        initialized = false;
    }
};

// 车辆运动模型参数
struct VehicleModel {
    float wheelbase;            // 轴距 (m)
    float maxAcceleration;      // 最大加速度 (m/s²)
    float maxDeceleration;      // 最大减速度 (m/s²)
    float maxSteeringAngle;     // 最大转向角 (rad)
    
    VehicleModel() {
        wheelbase = 2.7f;           // 典型轿车轴距
        maxAcceleration = 3.0f;
        maxDeceleration = 8.0f;
        maxSteeringAngle = 0.6f;    // 约35度
    }
};

class EKFVehicleTracker {
public:
    EKFVehicleTracker(IIMUProvider* imu, double initLat = 0, double initLng = 0);
    
    // 配置设置
    void setGPSProvider(IGPSProvider* gps);
    void setMagProvider(IMagProvider* mag);
    void setEKFConfig(const EKFConfig& config);
    void setVehicleModel(const VehicleModel& model);
    void setInitialPosition(double lat, double lng);
    
    // 初始化和更新
    void begin();
    void update();
    
    // 获取结果
    Position getPosition();
    
    // 状态查询
    bool isInitialized();
    float getPositionAccuracy();
    float getVelocity();
    float getHeading();
    float getHeadingRate();
    
    // 调试信息
    void getStateVector(float state[EKF_STATE_DIM]);
    void getCovarianceMatrix(float covariance[EKF_STATE_DIM * EKF_STATE_DIM]);

private:
    // 传感器提供者
    IIMUProvider* imuProvider;
    IGPSProvider* gpsProvider;
    IMagProvider* magProvider;
    
    // 配置和模型
    EKFConfig config;
    VehicleModel vehicleModel;
    
    // EKF状态
    EKFState ekfState;
    Position currentPosition;
    
    // 坐标转换
    double originLat, originLng;
    bool hasOrigin;
    
    // EKF核心算法
    void predict(float deltaTime);
    void updateWithGPS(const GPSData& gpsData);
    void updateWithIMU(const IMUData& imuData);
    void updateWithMag(const MagData& magData);
    
    // 矩阵运算
    void matrixMultiply(const float* A, const float* B, float* C, int rowsA, int colsA, int colsB);
    void matrixTranspose(const float* A, float* AT, int rows, int cols);
    void matrixAdd(const float* A, const float* B, float* C, int rows, int cols);
    void matrixSubtract(const float* A, const float* B, float* C, int rows, int cols);
    bool matrixInvert(const float* A, float* Ainv, int n);
    
    // 坐标转换
    void latLngToXY(double lat, double lng, float& x, float& y);
    void xyToLatLng(float x, float y, double& lat, double& lng);
    
    // 工具函数
    float normalizeAngle(float angle);
    void updatePositionFromState();
};

#endif // EKF_VEHICLE_TRACKER_H
