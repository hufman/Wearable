/*
  Orientation module

*/

// Use Seeed nrf52 platform 1.1.1 or:
// Add the Seeed LSM6DS3 library
// and then edit ~/Arduino/libraries/Seeed_Arduino_LSM6DS3/LSM6DS3.cpp to force-enable the #define Wire Wire1
// https://forum.seeedstudio.com/t/how-to-access-wire1-with-bluefruit-library/266295/9
#include <LSM6DS3.h>
#include <MadgwickAHRS.h>

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
int imuSuccess = 1;

// Madgwick orientation
Madgwick madgwickFilter;
long madgwickPrevious = 0;
int madgwickInterval = 0;

void imuInit() {

  //Call .begin() to configure the IMUs
  // myIMU.settings.gyroEnabled = 0;  // investigate dynamic toggling
  imuSuccess = myIMU.begin();
  if (imuSuccess != 0) {
      Serial.println("Device error");
  } else {
      Serial.println("Device OK!");
  }

  madgwickPrevious = micros();
  madgwickInterval = (int)(1000000 / 20);
  madgwickFilter.begin(20);
}

Madgwick madgwickGet() {
    return madgwickFilter;
}

class GyroCalibration {
public:
    GyroCalibration();
    void update(float ax, float ay, float az, float gx, float gy, float gz);
    void adjust(float* gx, float* gy, float* gz);
private:
    float stillAccelerometer = 0.005f;
    int index = 0;
    float histAx[16] = {0.0f};
    float histAy[16] = {0.0f};
    float histAz[16] = {0.0f};
    float histGx[16] = {0.0f};
    float histGy[16] = {0.0f};
    float histGz[16] = {0.0f};
    float calibGx = 0.0f; // the resting gyroscope values, to be subtracted from new readings
    float calibGy = 0.0f;
    float calibGz = 0.0f;
    bool isStill();
    float average(float data[16]);
    float maxDeviation(float middle, float data[16]);
};
GyroCalibration::GyroCalibration() {
}
void GyroCalibration::update(float ax, float ay, float az, float gx, float gy, float gz) {
    histAx[index] = ax;
    histAy[index] = ay;
    histAz[index] = az;
    histGx[index] = gx;
    histGy[index] = gy;
    histGz[index] = gz;
    index += 1;
    if (index > 15) index = 0;
    if (index == 0 && isStill()) {
        calibGx = average(histGx);
        calibGy = average(histGy);
        calibGz = average(histGz);
    }
}
bool GyroCalibration::isStill() {
    float aveAx = average(histAx);
    float aveAy = average(histAy);
    float aveAz = average(histAz);
    return maxDeviation(aveAx, histAx) < stillAccelerometer &&
           maxDeviation(aveAy, histAy) < stillAccelerometer &&
           maxDeviation(aveAz, histAz) < stillAccelerometer;
}
float GyroCalibration::average(float data[16]) {
    double sum = 0.0;
    for (int i = 0; i < 16; i++) {
        sum += (double)data[i];
    }
    return (float)(sum / 16);
}
float GyroCalibration::maxDeviation(float mean, float data[16]) {
    float deviation = 0.0f;
    for (int i = 0; i < 16; i++) {
       deviation = max(deviation, abs(mean - data[i]));
    }
    return deviation;
}
void GyroCalibration::adjust(float* gx, float* gy, float* gz) {
    *gx -= calibGx;
    *gy -= calibGy;
    *gz -= calibGz;
}

GyroCalibration gyroCalibration;

float copysign(float base, float mod1, float mod2) {
  if (mod1 < 0) base *= -1;
  if (mod2 < 0) base *= -1;
  return base;
}

void reportIMU() {
  
  if (imuSuccess != 0) {
      Serial.print("Device error: ");
      Serial.println(imuSuccess);
  } else {
      Serial.println("Device OK!");
  }

  // https://how2electronics.com/using-imu-microphone-on-xiao-ble-nrf52840-sense/
  float x_val = myIMU.readFloatAccelX();
  float y_val = myIMU.readFloatAccelY();
  float z_val = myIMU.readFloatAccelZ();
  //Accelerometer
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X1 = ");
  Serial.println(x_val, 4);
  Serial.print(" Y1 = ");
  Serial.println(y_val, 4);
  Serial.print(" Z1 = ");
  Serial.println(z_val, 4);

  // https://www.hobbytronics.co.uk/accelerometer-info
  float x2 = x_val * x_val;
  float y2 = y_val * y_val;
  float z2 = z_val * z_val;

  float result;
  //X Axis
  result=sqrt(y2+z2);
//  result=copysign(result, y_val, z_val);
//  result=x_val/result;
  float accel_rad_x = atan2(x_val, result);
  float accel_angle_x = accel_rad_x * 180/PI;

  //Y Axis
  result=sqrt(x2+z2);
  result=copysign(result, x_val, z_val);
//  result=y_val/result;
//  float accel_rad_y = atan2(y_val, result);
  float accel_rad_y = atan2(y_val, z_val);
  float accel_angle_y = accel_rad_y * 180/PI;
  
  //Z Axis
  result=sqrt(x2+y2);
//  result=copysign(result, x_val, y_val);
//  result=z_val/result;
  float accel_rad_z = atan2(z_val, result);
  float accel_angle_z = accel_rad_z * 180/PI;
  
  Serial.print(" Roll = ");
  Serial.println(accel_angle_y, 4);
  Serial.print(" Pitch = ");
  Serial.println(accel_angle_x, 4);
  Serial.print(" Yaw = ");
  Serial.println(accel_angle_z, 4);
  
  //Gyroscope
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();
  gyroCalibration.update(x_val, y_val, z_val, gx, gy, gz);
  gyroCalibration.adjust(&gx, &gy, &gz);

  Serial.print("\nGyroscope:\n");
  Serial.print(" X1 = ");
  Serial.println(gx, 4);
  Serial.print(" Y1 = ");
  Serial.println(gy, 4);
  Serial.print(" Z1 = ");
  Serial.println(gz, 4);

  Serial.print("Orientation: ");
  Serial.print(madgwickFilter.getYaw());
  Serial.print(" ");
  Serial.print(madgwickFilter.getPitch());
  Serial.print(" ");
  Serial.println(madgwickFilter.getRoll());

  //Thermometer
  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C1 = ");
  Serial.println(myIMU.readTempC(), 4);
  Serial.print(" Degrees F1 = ");
  Serial.println(myIMU.readTempF(), 4);
}

void imuUpdate() {
  if (madgwickPrevious + madgwickInterval < micros()) {
    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    float gx = myIMU.readFloatGyroX();
    float gy = myIMU.readFloatGyroY();
    float gz = myIMU.readFloatGyroZ();
    gyroCalibration.adjust(&gx, &gy, &gz);
    madgwickFilter.updateIMU(gx, gy, gz, ax, ay, az);

    madgwickPrevious += madgwickInterval;
  }
  if (madgwickPrevious > micros()) madgwickPrevious = micros();
};
