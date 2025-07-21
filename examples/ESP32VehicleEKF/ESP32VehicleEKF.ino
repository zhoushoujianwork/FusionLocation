#include <FusionLocation.h>
#include <EKFVehicleTracker.h>

// ESP32车机EKF定位融合示例
// 推荐硬件配置：
// - GPS: NEO-8M/NEO-9M (10Hz)
// - IMU: MPU6050/ICM20948 (100Hz)
// - 地磁计: QMC5883L/HMC5883L (50Hz)

// 模拟车辆IMU数据提供者
class VehicleIMUProvider : public IIMUProvider {
private:
    uint32_t lastUpdate;
    float simTime;
    float currentSpeed;
    float currentHeading;
    
public:
    VehicleIMUProvider() : lastUpdate(0), simTime(0), currentSpeed(0), currentHeading(0) {}
    
    bool getData(IMUData& data) override {
        uint32_t now = millis();
        if (now - lastUpdate < 10) return false; // 100Hz采样率
        
        lastUpdate = now;
        simTime += 0.01f;
        
        // 模拟车辆运动：加速、转弯、制动
        float phase = simTime * 0.1f;
        
        // 模拟加速度 (车辆前进方向)
        float acceleration = sin(phase) * 2.0f; // ±2m/s²
        currentSpeed += acceleration * 0.01f;
        currentSpeed = constrain(currentSpeed, 0, 20.0f); // 最大20m/s
        
        // 模拟转向 (Z轴角速度)
        float steeringInput = sin(phase * 0.3f) * 0.2f; // ±0.2 rad/s
        currentHeading += steeringInput * 0.01f;
        
        // 转换到车体坐标系
        data.accel[0] = acceleration; // 前进方向加速度
        data.accel[1] = currentSpeed * steeringInput; // 侧向加速度
        data.accel[2] = 9.8f + random(-50, 50) / 1000.0f; // Z轴重力+噪声
        
        data.gyro[0] = random(-10, 10) / 1000.0f; // X轴角速度噪声
        data.gyro[1] = random(-10, 10) / 1000.0f; // Y轴角速度噪声
        data.gyro[2] = steeringInput + random(-20, 20) / 1000.0f; // Z轴转向角速度
        
        data.timestamp = now;
        data.valid = true;
        return true;
    }
    
    bool isAvailable() override {
        return true;
    }
    
    float getCurrentSpeed() { return currentSpeed; }
    float getCurrentHeading() { return currentHeading; }
};

// 模拟车载GPS提供者
class VehicleGPSProvider : public IGPSProvider {
private:
    uint32_t lastUpdate;
    double currentLat, currentLng;
    float simTime;
    VehicleIMUProvider* imuRef;
    
public:
    VehicleGPSProvider(double startLat, double startLng, VehicleIMUProvider* imu) 
        : lastUpdate(0), currentLat(startLat), currentLng(startLng), simTime(0), imuRef(imu) {}
    
    bool getData(GPSData& data) override {
        uint32_t now = millis();
        if (now - lastUpdate < 100) return false; // 10Hz采样率
        
        lastUpdate = now;
        simTime += 0.1f;
        
        // 基于IMU速度和航向更新GPS位置
        if (imuRef) {
            float speed = imuRef->getCurrentSpeed();
            float heading = imuRef->getCurrentHeading();
            
            // 简单的位置积分
            float deltaTime = 0.1f;
            float deltaX = speed * cos(heading) * deltaTime;
            float deltaY = speed * sin(heading) * deltaTime;
            
            // 转换为经纬度偏移 (粗略)
            currentLat += deltaY / 111000.0; // 约111km/度
            currentLng += deltaX / (111000.0 * cos(currentLat * 0.017453));
        }
        
        // 添加GPS噪声
        data.lat = currentLat + random(-50, 50) / 1000000.0; // ±5米精度
        data.lng = currentLng + random(-50, 50) / 1000000.0;
        data.altitude = 100.0f + random(-20, 20) / 10.0f;
        
        // 模拟GPS精度变化
        data.accuracy = 3.0f + random(0, 70) / 10.0f; // 3-10米精度
        
        data.timestamp = now;
        data.valid = millis() > 3000; // 3秒后GPS可用
        return data.valid;
    }
    
    bool isAvailable() override {
        return millis() > 3000 && random(0, 100) > 5; // 95%可用率
    }
};

// 模拟车载地磁计提供者
class VehicleMagProvider : public IMagProvider {
private:
    uint32_t lastUpdate;
    VehicleIMUProvider* imuRef;
    
public:
    VehicleMagProvider(VehicleIMUProvider* imu) : lastUpdate(0), imuRef(imu) {}
    
    bool getData(MagData& data) override {
        uint32_t now = millis();
        if (now - lastUpdate < 20) return false; // 50Hz采样率
        
        lastUpdate = now;
        
        // 基于当前航向计算磁场
        float heading = imuRef ? imuRef->getCurrentHeading() : 0;
        float magneticDeclination = 0.1f; // 磁偏角
        float trueHeading = heading + magneticDeclination;
        
        // 地磁场强度约50μT
        data.mag[0] = cos(trueHeading) * 50.0f + random(-100, 100) / 100.0f;
        data.mag[1] = sin(trueHeading) * 50.0f + random(-100, 100) / 100.0f;
        data.mag[2] = -30.0f + random(-50, 50) / 100.0f; // 垂直分量
        
        data.timestamp = now;
        data.valid = true;
        return true;
    }
    
    bool isAvailable() override {
        return true;
    }
};

// 全局对象
VehicleIMUProvider imuProvider;
VehicleGPSProvider gpsProvider(39.9042, 116.4074, &imuProvider); // 北京坐标
VehicleMagProvider magProvider(&imuProvider);

// 创建EKF车辆追踪器
EKFVehicleTracker ekfTracker(&imuProvider, 39.9042, 116.4074);

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 车机EKF定位融合示例");
    Serial.println("========================================");
    
    // 配置EKF参数
    EKFConfig config;
    config.processNoisePos = 0.1f;      // 位置过程噪声
    config.processNoiseVel = 0.5f;      // 速度过程噪声
    config.gpsNoisePos = 25.0f;         // GPS噪声 (5m精度)
    config.imuNoiseGyro = 0.01f;        // IMU角速度噪声
    ekfTracker.setEKFConfig(config);
    
    // 配置车辆模型
    VehicleModel vehicle;
    vehicle.wheelbase = 2.7f;           // 轴距2.7m
    vehicle.maxAcceleration = 3.0f;     // 最大加速度
    vehicle.maxDeceleration = 8.0f;     // 最大减速度
    ekfTracker.setVehicleModel(vehicle);
    
    // 设置传感器
    ekfTracker.setGPSProvider(&gpsProvider);
    ekfTracker.setMagProvider(&magProvider);
    
    // 初始化
    ekfTracker.begin();
    
    Serial.println("初始化完成");
    Serial.println("时间 | 纬度      | 经度       | 精度  | 航向  | 速度   | 角速度 | 数据源");
    Serial.println("-----|-----------|------------|-------|-------|--------|--------|-------");
}

void loop() {
    // 更新EKF
    ekfTracker.update();
    
    // 获取融合位置
    Position pos = ekfTracker.getPosition();
    
    // 每500ms输出一次结果
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        
        if (pos.valid) {
            Serial.printf("%4lu | %9.6f | %10.6f | %5.1fm | %5.1f° | %5.1fm/s | %6.1f°/s | ",
                         millis()/1000, pos.lat, pos.lng, pos.accuracy, 
                         pos.heading, pos.speed, ekfTracker.getHeadingRate());
            
            // 显示数据源
            if (pos.sources.hasGPS) Serial.print("G");
            if (pos.sources.hasIMU) Serial.print("I");
            if (pos.sources.hasMag) Serial.print("M");
            Serial.println();
        } else {
            Serial.println("EKF未初始化或数据无效");
        }
        
        // 显示调试信息
        if (millis() % 5000 < 500) { // 每5秒显示一次详细信息
            Serial.println("--- EKF状态向量 ---");
            float state[6];
            ekfTracker.getStateVector(state);
            Serial.printf("位置: (%.2f, %.2f)m, 速度: (%.2f, %.2f)m/s, 航向: %.2f°, 角速度: %.2f°/s\n",
                         state[0], state[1], state[2], state[3], 
                         state[4] * 57.3f, state[5] * 57.3f);
        }
    }
    
    delay(5); // 200Hz主循环
}
