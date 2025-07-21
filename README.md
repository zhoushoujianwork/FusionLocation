# FusionLocation

一个用于 Arduino 平台的多源融合定位库，支持 GPS、陀螺仪、地磁计等多种定位方式的数据融合，输出更稳定、可靠的实时位置信息。

## 设计特点

### 🔧 通用性设计
- **抽象接口**：通过标准接口支持任意品牌的GPS、IMU、地磁计设备
- **渐进式配置**：从最简单的IMU+初始位置开始，可逐步添加更多传感器
- **容错性**：某个传感器失效时，系统仍能基于其他传感器工作

### 📊 支持的传感器组合
- ✅ **IMU + 初始位置**（最基本配置）
- ✅ **IMU + GPS + 初始位置**
- ✅ **IMU + GPS + 地磁计 + 初始位置**（完整配置）
- ✅ **IMU + 地磁计 + 初始位置**（无GPS场景）

### 🎯 融合算法
- 简化的卡尔曼滤波器
- GPS数据校正位置漂移
- IMU数据提供高频位置更新
- 地磁计数据校正航向漂移

## 快速开始

### 1. 实现传感器接口

```cpp
#include <FusionLocation.h>

// 实现IMU接口（必需）
class MyIMUProvider : public IIMUProvider {
public:
    bool getData(IMUData& data) override {
        // 从你的IMU设备读取数据
        data.accel[0] = myIMU.getAccelX();
        data.accel[1] = myIMU.getAccelY();
        data.accel[2] = myIMU.getAccelZ();
        data.gyro[0] = myIMU.getGyroX();
        data.gyro[1] = myIMU.getGyroY();
        data.gyro[2] = myIMU.getGyroZ();
        data.timestamp = millis();
        data.valid = true;
        return true;
    }
    
    bool isAvailable() override {
        return myIMU.isReady();
    }
};

// 实现GPS接口（可选）
class MyGPSProvider : public IGPSProvider {
public:
    bool getData(GPSData& data) override {
        if (gps.location.isValid()) {
            data.lat = gps.location.lat();
            data.lng = gps.location.lng();
            data.altitude = gps.altitude.meters();
            data.accuracy = gps.hdop.hdop();
            data.timestamp = millis();
            data.valid = true;
            return true;
        }
        return false;
    }
    
    bool isAvailable() override {
        return gps.location.isValid();
    }
};
```

### 2. 创建融合定位对象

```cpp
MyIMUProvider imuProvider;
MyGPSProvider gpsProvider;

// 创建融合定位对象（北京坐标作为初始位置）
FusionLocation fusion(&imuProvider, 39.9042, 116.4074);

void setup() {
    Serial.begin(115200);
    
    // 设置可选传感器
    fusion.setGPSProvider(&gpsProvider);
    
    // 初始化
    fusion.begin();
}

void loop() {
    // 更新融合计算
    fusion.update();
    
    // 获取融合后的位置
    Position pos = fusion.getPosition();
    
    if (pos.valid) {
        Serial.printf("位置: %.6f, %.6f, 精度: %.1fm\n", 
                     pos.lat, pos.lng, pos.accuracy);
        Serial.printf("航向: %.1f°, 速度: %.1fm/s\n", 
                     pos.heading, pos.speed);
    }
    
    delay(100);
}
```

## API 参考

### 数据结构

#### Position（位置信息）
```cpp
struct Position {
    double lat, lng;           // 融合后的经纬度
    float altitude;            // 高度
    float accuracy;            // 位置精度估计 (米)
    float heading;             // 航向角 (度, 0-360)
    float speed;               // 速度估计 (m/s)
    uint32_t timestamp;        // 时间戳
    bool valid;                // 数据有效性
    
    struct {
        bool hasGPS;           // 是否有GPS数据
        bool hasIMU;           // 是否有IMU数据
        bool hasMag;           // 是否有地磁计数据
    } sources;
};
```

### 主要方法

#### 构造函数
```cpp
FusionLocation(IIMUProvider* imu, double initLat = 0, double initLng = 0);
```

#### 传感器设置
```cpp
void setGPSProvider(IGPSProvider* gps);      // 设置GPS提供者
void setMagProvider(IMagProvider* mag);      // 设置地磁计提供者
void setInitialPosition(double lat, double lng); // 设置初始位置
```

#### 核心方法
```cpp
void begin();                    // 初始化
void update();                   // 更新融合计算（在主循环中调用）
Position getPosition();          // 获取当前融合位置
```

#### 状态查询
```cpp
bool isInitialized();           // 是否已初始化
float getPositionAccuracy();    // 获取位置精度
float getHeading();             // 获取航向
float getSpeed();               // 获取速度
uint32_t getLastUpdateTime();   // 获取最后更新时间
```

## 使用场景

### 🚗 车载导航
```cpp
// 车辆启动时有GPS，进入隧道后GPS失效，依靠IMU继续定位
FusionLocation carNav(&carIMU, lastKnownLat, lastKnownLng);
carNav.setGPSProvider(&carGPS);
```

### 🤖 室内机器人
```cpp
// 室内无GPS，使用IMU+地磁计+已知起始位置
FusionLocation robotNav(&robotIMU, roomStartLat, roomStartLng);
robotNav.setMagProvider(&robotMag);
```

### 📱 物联网设备
```cpp
// 低功耗场景，GPS间歇工作，IMU持续工作
FusionLocation iotTracker(&lowPowerIMU, deviceLat, deviceLng);
iotTracker.setGPSProvider(&periodicGPS);
```

## 依赖要求

- Arduino 核心库
- 用户提供的传感器库（如 TinyGPS++、MPU6050、QMC5883L 等）

## 许可证

MIT License

## 贡献

欢迎提交 Issue 和 Pull Request！
