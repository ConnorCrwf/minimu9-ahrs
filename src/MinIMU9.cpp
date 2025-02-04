#include "vector.h"
#include "MinIMU9.h"
#include "exceptions.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <wordexp.h>

MinIMU9::MinIMU9(const char * i2cDeviceName) :
  #ifdef IMU_V5
  gyro_accel(i2cDeviceName), mag(i2cDeviceName)
  #else
  compass(i2cDeviceName), gyro(i2cDeviceName)
  #endif
{
}

void MinIMU9::enable()
{
    #ifdef IMU_V5
    gyro_accel.enable();
    mag.enable();
    #else
    compass.enable();
    gyro.enable();
    #endif
}

void MinIMU9::loadCalibration()
{
    wordexp_t expansion_result;
    wordexp("~/.minimu9-ahrs-cal", &expansion_result, 0);

    std::ifstream file(expansion_result.we_wordv[0]);
    if (file.fail())
    {
        throw posix_error("Failed to open calibration file ~/.minimu9-ahrs-cal.");
    }
    
    file >> mag_min(0) >> mag_max(0) >> mag_min(1) >> mag_max(1) >> mag_min(2) >> mag_max(2);
    if (file.fail() || file.bad())
    {
        throw std::runtime_error("Failed to parse calibration file ~/.minimu9-ahrs-cal.");
    }
    
}

void MinIMU9::measureOffsets()
{
    // LSM303 accelerometer: 8 g sensitivity.  3.8 mg/digit; 1 g = 256.
    // TODO: unify this with the other place in the code where we scale accelerometer readings.
    gyro_offset = vector::Zero();
    const int sampleCount = 32;
    for(int i = 0; i < sampleCount; i++)
    {
        #ifdef IMU_V5
        gyro_accel.gyroRead();
        gyro_offset += vector_from_ints(&gyro_accel.g);
        #else
        gyro.read();
        gyro_offset += vector_from_ints(&gyro.g);
        #endif
        usleep(20*1000);
    }
    gyro_offset /= sampleCount;
}

vector MinIMU9::readMag()
{
    #ifdef IMU_V5
    mag.read();
    IMU::raw_m = int_vector_from_ints(&mag.m);
    vector v;
    v(0) = (float)(mag.m[0] - mag_min(0)) / (mag_max(0) - mag_min(0)) * 2 - 1;
    v(1) = (float)(mag.m[1] - mag_min(1)) / (mag_max(1) - mag_min(1)) * 2 - 1;
    v(2) = (float)(mag.m[2] - mag_min(2)) / (mag_max(2) - mag_min(2)) * 2 - 1;
    #else
    compass.readMag();
    IMU::raw_m = int_vector_from_ints(&compass.m);
    vector v;
    v(0) = (float)(compass.m[0] - mag_min(0)) / (mag_max(0) - mag_min(0)) * 2 - 1;
    v(1) = (float)(compass.m[1] - mag_min(1)) / (mag_max(1) - mag_min(1)) * 2 - 1;
    v(2) = (float)(compass.m[2] - mag_min(2)) / (mag_max(2) - mag_min(2)) * 2 - 1;
    #endif

    return v;
}

vector MinIMU9::readAcc()
{
    // Info about linear acceleration sensitivity from datasheets:
    // LSM303DLM: at FS = 8 g, 3.9 mg/digit (12-bit reading)
    // LSM303DLHC: at FS = 8 g, 4 mg/digit (12-bit reading probably an approximation)
    // LSM303DLH: at FS = 8 g, 3.9 mg/digit (12-bit reading)
    // LSM303D: at FS = 8 g, 0.244 mg/LSB (16-bit reading)
    const float accel_scale = 0.000244;

    #ifdef IMU_V5
    gyro_accel.accelRead();
    IMU::raw_a = int_vector_from_ints(&gyro_accel.a);
    return vector_from_ints(&gyro_accel.a) * accel_scale;
    #else
    compass.readAcc();
    IMU::raw_a = int_vector_from_ints(&compass.a);
    return vector_from_ints(&compass.a) * accel_scale;
    #endif
}

vector MinIMU9::readGyro()
{
    // Info about sensitivity from datasheets:
    // L3G4200D: at FS = 2000 dps, 70 mdps/digit
    // L3GD20: at FS = 2000 dps, 70 mdps/digit
    // L3GD20H: at FS = 2000 dps, 70 mdps/digit
    const float gyro_scale = 0.07 * 3.14159265 / 180;
    #ifdef IMU_V5
    gyro_accel.gyroRead();
    IMU::raw_g = int_vector_from_ints(&gyro_accel.g);
    return ( vector_from_ints(&gyro_accel.g) - gyro_offset ) * gyro_scale;
    #else
    gyro.read();
    IMU::raw_g = int_vector_from_ints(&gyro.g);
    return ( vector_from_ints(&gyro.g) - gyro_offset ) * gyro_scale;
    #endif
}
