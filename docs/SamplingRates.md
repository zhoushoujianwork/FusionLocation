# 传感器采样率配置指南

## 🎯 推荐采样率配置

### 车载导航场景

| 传感器类型 | 推荐采样率 | 最低采样率 | 最高采样率 | 说明 |
|-----------|-----------|-----------|-----------|------|
| **GPS** | 10Hz | 1Hz | 20Hz | 高频GPS提供稳定位置基准 |
| **IMU** | 100Hz | 50Hz | 200Hz | 高频IMU捕捉车辆动态 |
| **地磁计** | 50Hz | 10Hz | 100Hz | 中频地磁计校正航向漂移 |

### 室内机器人场景

| 传感器类型 | 推荐采样率 | 最低采样率 | 最高采样率 | 说明 |
|-----------|-----------|-----------|-----------|------|
| **IMU** | 200Hz | 100Hz | 500Hz | 室内需要更高精度的惯性导航 |
| **地磁计** | 100Hz | 50Hz | 200Hz | 室内磁场干扰大，需要高频滤波 |
| **GPS** | 1Hz | 0.1Hz | 5Hz | 室内GPS信号弱，低频即可 |

### 低功耗物联网场景

| 传感器类型 | 推荐采样率 | 最低采样率 | 最高采样率 | 说明 |
|-----------|-----------|-----------|-----------|------|
| **IMU** | 20Hz | 10Hz | 50Hz | 平衡功耗和精度 |
| **GPS** | 0.1Hz | 0.02Hz | 1Hz | 间歇性GPS定位 |
| **地磁计** | 10Hz | 5Hz | 20Hz | 低功耗磁场检测 |

## ⚙️ ESP32性能优化配置

### CPU频率设置
```cpp
// 高性能模式 (车载导航)
setCpuFrequencyMhz(240); // 240MHz

// 平衡模式 (一般应用)
setCpuFrequencyMhz(160); // 160MHz

// 低功耗模式 (物联网设备)
setCpuFrequencyMhz(80);  // 80MHz
```

### 任务调度配置
```cpp
// 创建高优先级EKF任务
xTaskCreatePinnedToCore(
    ekfUpdateTask,      // 任务函数
    "EKF_Update",       // 任务名称
    4096,               // 栈大小
    NULL,               // 参数
    2,                  // 优先级 (高)
    &ekfTaskHandle,     // 任务句柄
    1                   // CPU核心1
);

// 创建传感器读取任务
xTaskCreatePinnedToCore(
    sensorReadTask,     // 任务函数
    "Sensor_Read",      // 任务名称
    2048,               // 栈大小
    NULL,               // 参数
    1,                  // 优先级 (中)
    &sensorTaskHandle,  // 任务句柄
    0                   // CPU核心0
);
```

## 📊 采样率对性能的影响

### 内存使用

| 配置 | IMU频率 | GPS频率 | 内存使用 | CPU使用率 |
|------|---------|---------|----------|-----------|
| 低配 | 50Hz | 1Hz | ~8KB | ~15% |
| 标准 | 100Hz | 10Hz | ~12KB | ~25% |
| 高配 | 200Hz | 20Hz | ~20KB | ~40% |

### 定位精度

| 配置 | 位置精度 | 航向精度 | 速度精度 | 响应时间 |
|------|----------|----------|----------|----------|
| 低配 | ±5m | ±5° | ±0.5m/s | 200ms |
| 标准 | ±2m | ±2° | ±0.2m/s | 100ms |
| 高配 | ±1m | ±1° | ±0.1m/s | 50ms |

## 🔧 代码配置示例

### 高性能车载配置
```cpp
// GPS配置 - 10Hz
class HighPerformanceGPS : public IGPSProvider {
    bool getData(GPSData& data) override {
        if (millis() - lastUpdate < 100) return false; // 10Hz
        // ... GPS读取代码
    }
};

// IMU配置 - 100Hz
class HighPerformanceIMU : public IIMUProvider {
    bool getData(IMUData& data) override {
        if (millis() - lastUpdate < 10) return false; // 100Hz
        // ... IMU读取代码
    }
};

// EKF配置
EKFConfig config;
config.processNoisePos = 0.05f;     // 低过程噪声
config.gpsNoisePos = 9.0f;          // GPS精度3m
config.imuNoiseGyro = 0.005f;       // 高精度IMU
```

### 低功耗物联网配置
```cpp
// GPS配置 - 0.1Hz (10秒一次)
class LowPowerGPS : public IGPSProvider {
    bool getData(GPSData& data) override {
        if (millis() - lastUpdate < 10000) return false; // 0.1Hz
        // ... GPS读取代码
    }
};

// IMU配置 - 20Hz
class LowPowerIMU : public IIMUProvider {
    bool getData(IMUData& data) override {
        if (millis() - lastUpdate < 50) return false; // 20Hz
        // ... IMU读取代码
    }
};

// EKF配置
EKFConfig config;
config.processNoisePos = 0.5f;      // 较高过程噪声
config.gpsNoisePos = 100.0f;        // GPS精度10m
config.imuNoiseGyro = 0.02f;        // 中等精度IMU
```

## 📱 实际硬件推荐

### GPS模块
- **高精度**: u-blox NEO-9M (10Hz, 2.5m精度)
- **标准**: u-blox NEO-8M (5Hz, 3m精度)  
- **低功耗**: u-blox NEO-6M (1Hz, 5m精度)

### IMU模块
- **高性能**: ICM-20948 (1kHz, 9轴)
- **标准**: MPU6050 (1kHz, 6轴)
- **低功耗**: LSM6DS3 (6.6kHz, 6轴, 低功耗)

### 地磁计模块
- **高精度**: HMC5883L (160Hz, 1-2°精度)
- **标准**: QMC5883L (200Hz, 2-3°精度)
- **集成**: ICM-20948内置 (70Hz, 3-5°精度)

## ⚡ 功耗优化建议

### 动态采样率调整
```cpp
void adjustSamplingRate() {
    float speed = ekfTracker.getVelocity();
    
    if (speed < 1.0f) {
        // 静止状态 - 降低采样率
        imuSamplingRate = 20;  // 20Hz
        gpsSamplingRate = 1;   // 1Hz
    } else if (speed < 10.0f) {
        // 低速状态 - 标准采样率
        imuSamplingRate = 50;  // 50Hz
        gpsSamplingRate = 5;   // 5Hz
    } else {
        // 高速状态 - 高采样率
        imuSamplingRate = 100; // 100Hz
        gpsSamplingRate = 10;  // 10Hz
    }
}
```

### 睡眠模式管理
```cpp
void enterSleepMode() {
    if (vehicleStationary() && batteryLow()) {
        // 进入深度睡眠
        esp_sleep_enable_timer_wakeup(30 * 1000000); // 30秒后唤醒
        esp_deep_sleep_start();
    }
}
```

## 🎛️ 调试和监控

### 性能监控代码
```cpp
void printPerformanceStats() {
    Serial.printf("EKF更新频率: %.1fHz\n", ekfUpdateRate);
    Serial.printf("GPS数据率: %.1fHz\n", gpsDataRate);
    Serial.printf("IMU数据率: %.1fHz\n", imuDataRate);
    Serial.printf("CPU使用率: %.1f%%\n", cpuUsage);
    Serial.printf("内存使用: %dKB\n", memoryUsage / 1024);
}
```

选择合适的采样率配置可以在性能、精度和功耗之间找到最佳平衡点。
