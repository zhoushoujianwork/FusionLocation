# FusionLocation

一个用于 Arduino 平台的多源融合定位库，支持 GPS、陀螺仪、地磁计、LBS、WiFi 等多种定位方式的数据融合，输出更稳定、可靠的实时位置信息。

## 特性
- 支持多种传感器数据输入（GPS、IMU、LBS、WiFi）
- 融合算法（如卡尔曼滤波）实现高精度定位
- 适用于车载、物联网等多种场景

## 用法示例

```cpp
#include <FusionLocation.h>

FusionLocation fusionLoc;

void setup() {
  fusionLoc.begin();
}

void loop() {
  fusionLoc.update();
  auto pos = fusionLoc.getPosition();
  // pos.lat, pos.lng, pos.accuracy
}
```

## 依赖
- Arduino
- 相关传感器库（如 TinyGPS++、MPU6050、QMC5883L、LBS/WiFi定位API等）

## 许可证
MIT 