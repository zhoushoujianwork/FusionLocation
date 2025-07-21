#include <FusionLocation.h>

// 示例IMU提供者实现
class MockIMUProvider : public IIMUProvider {
private:
    uint32_t lastUpdate;
    float simTime;
    
public:
    MockIMUProvider() : lastUpdate(0), simTime(0) {}
    
    bool getData(IMUData& data) override {
        uint32_t now = millis();
        if (now - lastUpdate < 50) return false; // 20Hz更新率
        
        lastUpdate = now;
        simTime += 0.05f;
        
        // 模拟IMU数据
        data.accel[0] = sin(simTime * 0.1f) * 0.5f;  // 模拟x轴加速度
        data.accel[1] = cos(simTime * 0.1f) * 0.5f;  // 模拟y轴加速度
        data.accel[2] = 9.8f;                        // z轴重力
        
        data.gyro[0] = 0;
        data.gyro[1] = 0;
        data.gyro[2] = sin(simTime * 0.05f) * 0.1f;  // 模拟转向
        
        data.timestamp = now;
        data.valid = true;
        return true;
    }
    
    bool isAvailable() override {
        return true;
    }
};

// 示例GPS提供者实现
class MockGPSProvider : public IGPSProvider {
private:
    uint32_t lastUpdate;
    double baseLat, baseLng;
    float simTime;
    
public:
    MockGPSProvider(double lat, double lng) : lastUpdate(0), baseLat(lat), baseLng(lng), simTime(0) {}
    
    bool getData(GPSData& data) override {
        uint32_t now = millis();
        if (now - lastUpdate < 1000) return false; // 1Hz更新率
        
        lastUpdate = now;
        simTime += 1.0f;
        
        // 模拟GPS数据 - 围绕基点做小范围移动
        data.lat = baseLat + sin(simTime * 0.01f) * 0.0001f;  // 约10米范围
        data.lng = baseLng + cos(simTime * 0.01f) * 0.0001f;
        data.altitude = 100.0f;
        data.accuracy = 5.0f + random(0, 50) / 10.0f;  // 5-10米精度
        data.timestamp = now;
        data.valid = true;
        return true;
    }
    
    bool isAvailable() override {
        return millis() > 5000; // 5秒后GPS可用
    }
};

// 示例地磁计提供者实现
class MockMagProvider : public IMagProvider {
private:
    uint32_t lastUpdate;
    float simTime;
    
public:
    MockMagProvider() : lastUpdate(0), simTime(0) {}
    
    bool getData(MagData& data) override {
        uint32_t now = millis();
        if (now - lastUpdate < 100) return false; // 10Hz更新率
        
        lastUpdate = now;
        simTime += 0.1f;
        
        // 模拟地磁计数据 - 指向磁北
        float heading = simTime * 0.02f; // 缓慢旋转
        data.mag[0] = cos(heading) * 50.0f;  // 磁场强度约50μT
        data.mag[1] = sin(heading) * 50.0f;
        data.mag[2] = -30.0f;                // 垂直分量
        
        data.timestamp = now;
        data.valid = true;
        return true;
    }
    
    bool isAvailable() override {
        return true;
    }
};

// 全局对象
MockIMUProvider imuProvider;
MockGPSProvider gpsProvider(39.9042, 116.4074); // 北京坐标
MockMagProvider magProvider;

// 创建融合定位对象
FusionLocation fusion(&imuProvider, 39.9042, 116.4074);

void setup() {
    Serial.begin(115200);
    Serial.println("FusionLocation 示例开始");
    
    // 设置可选传感器
    fusion.setGPSProvider(&gpsProvider);
    fusion.setMagProvider(&magProvider);
    
    // 初始化
    fusion.begin();
    
    Serial.println("初始化完成");
    Serial.println("格式: 时间 | 纬度 | 经度 | 精度 | 航向 | 速度 | 数据源");
}

void loop() {
    // 更新融合定位
    fusion.update();
    
    // 获取当前位置
    Position pos = fusion.getPosition();
    
    // 每秒输出一次结果
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        
        if (pos.valid) {
            Serial.printf("%6lu | %9.6f | %10.6f | %5.1fm | %5.1f° | %4.1fm/s | ",
                         millis()/1000, pos.lat, pos.lng, pos.accuracy, 
                         pos.heading, pos.speed);
            
            // 显示数据源
            if (pos.sources.hasGPS) Serial.print("G");
            if (pos.sources.hasIMU) Serial.print("I");
            if (pos.sources.hasMag) Serial.print("M");
            Serial.println();
        } else {
            Serial.println("位置数据无效");
        }
        
        // 显示系统状态
        if (!fusion.isInitialized()) {
            Serial.println("警告: 系统未初始化");
        }
    }
    
    delay(10); // 100Hz主循环
}
