#ifndef FUSION_LOCATION_H
#define FUSION_LOCATION_H

#include <Arduino.h>

// 融合定位主类，支持GPS、IMU、LBS、WiFi等多源数据融合
class FusionLocation {
public:
    // 初始化各类传感器
    void begin();
    // 更新各类传感器数据并进行融合计算
    void update();
    // 获取当前融合定位结果
    struct Position {
        double lat;
        double lng;
        float accuracy;
    };
    Position getPosition();
};

#endif // FUSION_LOCATION_H 