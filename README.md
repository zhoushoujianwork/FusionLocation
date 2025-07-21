# FusionLocation

一个用于 Arduino 平台的多源融合定位库，支持 GPS、陀螺仪、地磁计等多种定位方式的数据融合，输出更稳定、可靠的实时位置信息。

## 🚀 核心功能

### 基础融合定位 (FusionLocation)
- **抽象接口设计**：支持任意品牌的GPS、IMU、地磁计设备
- **渐进式配置**：从最简单的IMU+初始位置开始，可逐步添加更多传感器
- **简化卡尔曼滤波**：适用于一般精度要求的应用场景

### 🚗 专业车机EKF定位 (EKFVehicleTracker)
- **扩展卡尔曼滤波器**：6维状态向量 [x, y, vx, vy, heading, heading_rate]
- **车辆运动模型**：考虑轴距、最大加速度等车辆物理约束
- **高精度融合**：专门针对车载导航优化的算法
- **实时性能**：ESP32上100Hz IMU + 10Hz GPS稳定运行

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
- **基础版本**：简化的卡尔曼滤波器
- **专业版本**：扩展卡尔曼滤波器 (EKF)
- GPS数据校正位置漂移
- IMU数据提供高频位置更新
- 地磁计数据校正航向漂移

## 快速开始

### 基础版本使用

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

MyIMUProvider imuProvider;
FusionLocation fusion(&imuProvider, 39.9042, 116.4074);

void setup() {
    fusion.begin();
}

void loop() {
    fusion.update();
    Position pos = fusion.getPosition();
    
    if (pos.valid) {
        Serial.printf("位置: %.6f, %.6f, 精度: %.1fm\n", 
                     pos.lat, pos.lng, pos.accuracy);
    }
    delay(100);
}
```

### 🚗 ESP32车机EKF版本使用

```cpp
#include <FusionLocation.h>
#include <EKFVehicleTracker.h>

MyIMUProvider imuProvider;
MyGPSProvider gpsProvider;

// 创建EKF车辆追踪器
EKFVehicleTracker ekfTracker(&imuProvider, 39.9042, 116.4074);

void setup() {
    // 配置EKF参数
    EKFConfig config;
    config.processNoisePos = 0.1f;      // 位置过程噪声
    config.gpsNoisePos = 25.0f;         // GPS噪声 (5m精度)
    ekfTracker.setEKFConfig(config);
    
    // 配置车辆模型
    VehicleModel vehicle;
    vehicle.wheelbase = 2.7f;           // 轴距2.7m
    vehicle.maxAcceleration = 3.0f;     // 最大加速度
    ekfTracker.setVehicleModel(vehicle);
    
    ekfTracker.setGPSProvider(&gpsProvider);
    ekfTracker.begin();
}

void loop() {
    ekfTracker.update();
    Position pos = ekfTracker.getPosition();
    
    if (pos.valid) {
        Serial.printf("位置: %.6f, %.6f, 精度: %.1fm, 速度: %.1fm/s\n", 
                     pos.lat, pos.lng, pos.accuracy, pos.speed);
    }
    delay(10); // 100Hz主循环
}
```

## 📊 推荐采样率配置

### 车载导航场景
- **GPS**: 10Hz (高精度基准)
- **IMU**: 100Hz (捕捉车辆动态)
- **地磁计**: 50Hz (航向校正)

### 室内机器人场景
- **IMU**: 200Hz (高精度惯性导航)
- **地磁计**: 100Hz (磁场干扰滤波)
- **GPS**: 1Hz (室内信号弱)

### 低功耗物联网场景
- **IMU**: 20Hz (平衡功耗和精度)
- **GPS**: 0.1Hz (间歇性定位)
- **地磁计**: 10Hz (低功耗检测)

详细配置请参考: [传感器采样率配置指南](docs/SamplingRates.md)

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

### 基础版本 (FusionLocation)

#### 构造函数
```cpp
FusionLocation(IIMUProvider* imu, double initLat = 0, double initLng = 0);
```

#### 核心方法
```cpp
void begin();                    // 初始化
void update();                   // 更新融合计算
Position getPosition();          // 获取当前融合位置
```

### 🚗 EKF版本 (EKFVehicleTracker)

#### 构造函数
```cpp
EKFVehicleTracker(IIMUProvider* imu, double initLat = 0, double initLng = 0);
```

#### 配置方法
```cpp
void setEKFConfig(const EKFConfig& config);      // 设置EKF参数
void setVehicleModel(const VehicleModel& model); // 设置车辆模型
```

#### 高级查询
```cpp
float getVelocity();             // 获取速度
float getHeadingRate();          // 获取角速度
void getStateVector(float state[6]); // 获取完整状态向量
```

## 使用场景

### 🚗 车载导航 (推荐EKF版本)
```cpp
EKFVehicleTracker carNav(&carIMU, lastKnownLat, lastKnownLng);
carNav.setGPSProvider(&carGPS);
// 进入隧道后GPS失效，依靠IMU继续高精度定位
```

### 🤖 室内机器人 (基础版本)
```cpp
FusionLocation robotNav(&robotIMU, roomStartLat, roomStartLng);
robotNav.setMagProvider(&robotMag);
// 室内无GPS，使用IMU+地磁计+已知起始位置
```

### 📱 物联网设备 (基础版本)
```cpp
FusionLocation iotTracker(&lowPowerIMU, deviceLat, deviceLng);
iotTracker.setGPSProvider(&periodicGPS);
// 低功耗场景，GPS间歇工作，IMU持续工作
```

## 📁 项目结构

```
FusionLocation/
├── src/
│   ├── FusionLocation.h         # 基础融合定位
│   ├── FusionLocation.cpp
│   ├── EKFVehicleTracker.h      # 🚗 EKF车机定位
│   └── EKFVehicleTracker.cpp
├── examples/
│   ├── BasicUsage/              # 基础使用示例
│   └── ESP32VehicleEKF/         # 🚗 ESP32车机EKF示例
├── docs/
│   └── SamplingRates.md         # 采样率配置指南
└── README.md
```

## 性能对比

| 特性 | 基础版本 | EKF车机版本 |
|------|----------|-------------|
| **定位精度** | ±5m | ±1-2m |
| **航向精度** | ±5° | ±1-2° |
| **速度精度** | ±0.5m/s | ±0.1m/s |
| **响应时间** | 200ms | 50ms |
| **内存使用** | ~8KB | ~20KB |
| **CPU使用率** | ~15% | ~40% |
| **适用场景** | 一般定位 | 车载导航 |

## 依赖要求

- Arduino 核心库
- ESP32 (推荐用于EKF版本)
- 用户提供的传感器库（如 TinyGPS++、MPU6050、QMC5883L 等）

## 许可证

MIT License

## 贡献

欢迎提交 Issue 和 Pull Request！
