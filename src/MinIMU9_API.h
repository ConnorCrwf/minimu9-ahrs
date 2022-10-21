#include "vector.h"
#include "exceptions.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <wordexp.h>
#include <vector>

/**@todo more general APIs.*/
vector gyro_offset;
int_vector mag_min, mag_max;

int_vector raw_m, raw_a, raw_g;

// gyro angular velocity readings
int g[3];

int a[3];  // accelerometer readings
int m[3];  // magnetometer readings

struct container {int reading[3];};

void loadCalibration()
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

void measureOffsets(std::vector<container> gyro_readings)
{
    // LSM303 accelerometer: 8 g sensitivity.  3.8 mg/digit; 1 g = 256.
    // TODO: unify this with the other place in the code where we scale accelerometer readings.
    gyro_offset = vector::Zero();
    for(auto gyro_readings_it = gyro_readings.begin(); gyro_readings_it != gyro_readings.end();
      gyro_readings_it++)
    {
        gyro_offset += vector_from_ints(&(gyro_readings_it->reading));
        usleep(20*1000);
    }
    gyro_offset /= gyro_readings.size();
}

vector readMag(container compass_reading)
{
    raw_m = int_vector_from_ints(&compass_reading.reading);
    
    vector v;
    v(0) = (float)(compass_reading.reading[0] - mag_min(0)) / (mag_max(0) - mag_min(0)) * 2 - 1;
    v(1) = (float)(compass_reading.reading[1] - mag_min(1)) / (mag_max(1) - mag_min(1)) * 2 - 1;
    v(2) = (float)(compass_reading.reading[2] - mag_min(2)) / (mag_max(2) - mag_min(2)) * 2 - 1;
    return v;
}

vector readAcc(container compass_reading)
{
    // Info about linear acceleration sensitivity from datasheets:
    // LSM303DLM: at FS = 8 g, 3.9 mg/digit (12-bit reading)
    // LSM303DLHC: at FS = 8 g, 4 mg/digit (12-bit reading probably an approximation)
    // LSM303DLH: at FS = 8 g, 3.9 mg/digit (12-bit reading)
    // LSM303D: at FS = 8 g, 0.244 mg/LSB (16-bit reading)
    const float accel_scale = 0.000244;

    raw_a = int_vector_from_ints(&compass_reading.reading);
    return vector_from_ints(&compass_reading.reading) * accel_scale;
}

vector readGyro(container gyro_reading)
{
    // Info about sensitivity from datasheets:
    // L3G4200D: at FS = 2000 dps, 70 mdps/digit
    // L3GD20: at FS = 2000 dps, 70 mdps/digit
    // L3GD20H: at FS = 2000 dps, 70 mdps/digit
    const float gyro_scale = 0.07 * 3.14159265 / 180;

    raw_g = int_vector_from_ints(&gyro_reading.reading);
    return ( vector_from_ints(&gyro_reading.reading) - gyro_offset ) * gyro_scale;
}
