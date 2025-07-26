#ifndef FUSION_LOCATION_H
#define FUSION_LOCATION_H

#include <Arduino.h>

// 通用常量定义
#ifndef FUSION_CONSTANTS_DEFINED
#define FUSION_CONSTANTS_DEFINED
#define EARTH_RADIUS 6371000.0f  // 地球半径 (米)
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232f
#endif
#endif

// 标准数据结构定义
struct GPSData {
    double lat, lng;
    float altitude;
    float accuracy;
    uint32_t timestamp;
    bool valid;
    
    GPSData() : lat(0), lng(0), altitude(0), accuracy(0), timestamp(0), valid(false) {}
};

struct IMUData {
    float accel[3];    // 加速度 x,y,z (m/s²)
    float gyro[3];     // 角速度 x,y,z (rad/s)
    uint32_t timestamp;
    bool valid;
    
    IMUData() : timestamp(0), valid(false) {
        for(int i = 0; i < 3; i++) {
            accel[i] = 0;
            gyro[i] = 0;
        }
    }
};

struct MagData {
    float mag[3];      // 磁场强度 x,y,z (μT)
    uint32_t timestamp;
    bool valid;
    
    MagData() : timestamp(0), valid(false) {
        for(int i = 0; i < 3; i++) {
            mag[i] = 0;
        }
    }
};

struct Position {
    double lat, lng;           // 融合后的经纬度
    float altitude;            // 高度
    float accuracy;            // 位置精度估计 (米)
    float heading;             // 航向角 (度, 0-360)
    float speed;               // 速度估计 (m/s)
    uint32_t timestamp;        // 时间戳
    bool valid;                // 数据有效性
    
    // 相对位移 (相对于起始点的2D位移，单位：米)
    struct {
        float x;               // 东西方向位移 (+东/-西)
        float y;               // 南北方向位移 (+北/-南)
        float distance;        // 距离起始点的直线距离
        float bearing;         // 相对于起始点的方位角 (度, 0-360)
    } displacement;
    
    // 数据来源标识
    struct {
        bool hasGPS;
        bool hasIMU; 
        bool hasMag;
    } sources;
    
    Position() : lat(0), lng(0), altitude(0), accuracy(0), heading(0), 
                 speed(0), timestamp(0), valid(false) {
        displacement.x = 0;
        displacement.y = 0;
        displacement.distance = 0;
        displacement.bearing = 0;
        sources.hasGPS = false;
        sources.hasIMU = false;
        sources.hasMag = false;
    }
};

// 抽象接口定义
class IGPSProvider {
public:
    virtual ~IGPSProvider() {}
    virtual bool getData(GPSData& data) = 0;
    virtual bool isAvailable() = 0;
};

class IIMUProvider {
public:
    virtual ~IIMUProvider() {}
    virtual bool getData(IMUData& data) = 0;
    virtual bool isAvailable() = 0;
};

class IMagProvider {
public:
    virtual ~IMagProvider() {}
    virtual bool getData(MagData& data) = 0;
    virtual bool isAvailable() = 0;
};

// 简单的卡尔曼滤波器状态
struct FilterState {
    double lat, lng;           // 位置状态
    float velX, velY;          // 速度状态 (m/s)
    float heading;             // 航向状态
    uint32_t lastUpdate;       // 上次更新时间
    bool initialized;          // 是否已初始化
    
    FilterState() : lat(0), lng(0), velX(0), velY(0), heading(0), 
                   lastUpdate(0), initialized(false) {}
};

class FusionLocation {
public:
    // 构造函数 - IMU是必需的
    FusionLocation(IIMUProvider* imu, double initLat = 0, double initLng = 0);
    
    // 设置可选传感器
    void setGPSProvider(IGPSProvider* gps);
    void setMagProvider(IMagProvider* mag);
    void setInitialPosition(double lat, double lng);
    
    // 初始化
    void begin();
    
    // 融合更新 - 在主循环中调用
    void update();
    
    // 获取融合后的位置
    Position getPosition();
    
    // 调试控制
    void setDebug(bool enable) { debugEnabled = enable; }
    
    // 相对位移控制
    void resetOrigin();                    // 重置起始点为当前位置
    void setOrigin(double lat, double lng); // 设置指定位置为起始点
    
    // 状态查询
    bool isInitialized();
    float getPositionAccuracy();
    float getHeading();
    float getSpeed();
    uint32_t getLastUpdateTime();

private:
    // 传感器提供者指针
    IIMUProvider* imuProvider;
    IGPSProvider* gpsProvider;
    IMagProvider* magProvider;
    
    // 初始位置
    double initialLat, initialLng;
    bool hasInitialPosition;
    
    // 起始点记录 (用于计算相对位移)
    double originLat, originLng;
    bool hasOrigin;
    
    // 滤波器状态
    FilterState filterState;
    Position currentPosition;
    
    // 调试控制
    bool debugEnabled;
    
    // 内部方法
    void updateWithIMU(const IMUData& imuData);
    void updateWithGPS(const GPSData& gpsData);
    void updateWithMag(const MagData& magData);
    void predictPosition(float deltaTime);
    float calculateHeading(const MagData& magData, const IMUData& imuData);
    void updatePositionEstimate();
    
    // 工具函数
    float constrainAngle(float angle);
    double metersToLatitude(float meters);
    double metersToLongitude(float meters, double lat);
    void calculateRelativeDisplacement();
};

#endif // FUSION_LOCATION_H
