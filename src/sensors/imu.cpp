#include "imu.h"
#include <Wire.h>
#include <SensorQMI8658.hpp>
#include "config.h"

namespace {

SensorQMI8658 g_qmi;

} // namespace

bool imu_begin()
{
    Wire.begin(IIC_SDA, IIC_SCL, 400000);
    if (!g_qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        return false;
    }
    g_qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,
        SensorQMI8658::ACC_ODR_250Hz,
        SensorQMI8658::LPF_MODE_0);
    g_qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_512DPS,
        SensorQMI8658::GYR_ODR_112_1Hz,
        SensorQMI8658::LPF_MODE_0);
    g_qmi.enableAccelerometer();
    g_qmi.enableGyroscope();
    return true;
}

void imu_read(float *ax, float *ay, float *az,
              float *gx, float *gy, float *gz)
{
    if (!g_qmi.getDataReady()) {
        *ax = *ay = *az = 0;
        *gx = *gy = *gz = 0;
        return;
    }
    IMUdata acc{};
    g_qmi.getAccelerometer(acc.x, acc.y, acc.z);
    IMUdata gyr{};
    g_qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
    *ax = acc.x; *ay = acc.y; *az = acc.z;
    *gx = gyr.x; *gy = gyr.y; *gz = gyr.z;
}
