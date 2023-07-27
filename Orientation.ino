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

class AccelerationTracker {
public:
    AccelerationTracker(Madgwick* adjustment);
    void update(float ax, float ay, float az);
    void enableReporting();
    void reset();
    long lastUpdate = 0;
    long lastReport = 0;
    long lastReportEnabled = 0;
    float ax0 = 0;
    float ay0 = 0;
    float az0 = 0;
    float vx = 0;
    float vy = 0;
    float vz = 0;
    float vx0 = 0;
    float vy0 = 0;
    float vz0 = 0;
    float x = 0;
    float y = 0;
    float z = 0;
private:
    Madgwick* madgwick;
};

AccelerationTracker::AccelerationTracker(Madgwick* adjustment) {
    madgwick = adjustment;
}
void AccelerationTracker::update(float ax, float ay, float az) {
    /** Transform the given acceleration values to translations */
    float tx = cos(madgwick->getPitchRadians()) * ax + sin(madgwick->getPitchRadians()) * az;
    float ty = -cos(madgwick->getRollRadians()) * ay + sin(madgwick->getRollRadians()) * az;
    float ty2 = cos(-madgwick->getRollRadians()) * ay + sin(madgwick->getRollRadians()) * az;
    float distance = sqrt(madgwick->getRollRadians()*madgwick->getRollRadians() + madgwick->getPitchRadians()*madgwick->getPitchRadians());
    float tz = sin(madgwick->getRollRadians()) * ay - sin(madgwick->getPitchRadians()) * ax
               + cos(distance) * az;
    float tz2 = tz - 1.02;
    if (lastReport + 1000000 < micros()) {
        Serial.print("Orientation: ");
        Serial.print(madgwick->getYaw());
        Serial.print(" ");
        Serial.print(madgwick->getPitch());
        Serial.print(" ");
        Serial.println(madgwick->getRoll());

        Serial.print("\nCurrent Accelerometer:\n");
        Serial.print(" X = ");
        Serial.println(ax, 4);
        Serial.print(" Y = ");
        Serial.println(ay, 4);
        Serial.print(" Z = ");
        Serial.println(az, 4);
        
        Serial.print("\nAdjusted Accelerometer:\n");
        Serial.print(" X1 = ");
        Serial.println(tx, 4);
        Serial.print(" Y1 = ");
        Serial.println(ty, 4);
        Serial.print(" Y2 = ");
        Serial.println(ty2, 4);
        Serial.print(" Z1 = ");
        Serial.println(tz, 4);
        Serial.print(" Z2 = ");
        Serial.println(tz2, 4);
        Serial.print(" sin(roll) = ");
        Serial.println(sin(madgwick->getRollRadians()), 4);
        lastReport = micros();
    }
    
    if (lastUpdate > 0) {
        long dt = micros() - lastUpdate;
        // update resting accelerometer reading
        float a0Factor = 0.05;  // how weakly to update the a0
        ax0 = ax0 * (1-a0Factor) + tx * a0Factor;
        ay0 = ay0 * (1-a0Factor) + ty * a0Factor;
        az0 = az0 * (1-a0Factor) + tz2 * a0Factor;
        // update velocity
        float dampingFactor = 1;
        vx = (vx + tx - ax0) * dampingFactor;
        vy = (vy + ty - ay0) * dampingFactor;
        vz = (vz + tz2 - az0) * dampingFactor;
        // move position based on velocity
        x += vx * dt/100000;
        y += vy * dt/100000;
        z += vz * dt/100000;
        // gradually reset to the middle
        float gravity = 0.99;
        x *= gravity;
        y *= gravity;
        z *= gravity;
    }
    lastUpdate = micros();

    if (lastReportEnabled > 0 && lastReportEnabled + 5000 > millis()) {
        // JSON output
        Serial.print("{");
        Serial.print("\"p\":");
        Serial.print(madgwick->getPitch(), 4);
        Serial.print(",\"r\":");
        Serial.print(madgwick->getRoll(), 4);
        Serial.print(",\"ax\":");
        Serial.print(tx, 4);
        Serial.print(",\"ay\":");
        Serial.print(ty, 4);
        Serial.print(",\"az\":");
        Serial.print(tz2, 4);
        Serial.print(",\"ax0\":");
        Serial.print(ax0, 4);
        Serial.print(",\"ay0\":");
        Serial.print(ay0, 4);
        Serial.print(",\"az0\":");
        Serial.print(az0, 4);
        Serial.print(",\"vx\":");
        Serial.print(vx, 4);
        Serial.print(",\"vy\":");
        Serial.print(vy, 4);
        Serial.print(",\"vz\":");
        Serial.print(vz, 4);
        Serial.print(",\"x\":");
        Serial.print(x, 4);
        Serial.print(",\"y\":");
        Serial.print(y, 4);
        Serial.print(",\"z\":");
        Serial.print(z, 4);
        Serial.println("}");
    }
}

void AccelerationTracker::enableReporting() {
  lastReportEnabled = millis();
}
void AccelerationTracker::reset() {
    Serial.print("\nVelocity:\n");
    Serial.print(" X=");
    Serial.print(vx, 4);
    Serial.print(" Y=");
    Serial.print(vy, 4);
    Serial.print(" Z=");
    Serial.println(vz, 4);
    
    Serial.print("\nPosition:\n");
    Serial.print(" X=");
    Serial.print(x, 4);
    Serial.print(" Y=");
    Serial.print(y, 4);
    Serial.print(" Z=");
    Serial.println(z, 4);
    /*
    x = 0;
    y = 0;
    z = 0;
    */
}

AccelerationTracker tracker = AccelerationTracker(&madgwickFilter);

void enableOrientationReporting() {
  tracker.enableReporting();
}


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
  
  tracker.reset();
  return;
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
    if (madgwickPrevious + madgwickInterval * 4 < micros()) {
      gyroCalibration.update(ax, ay, az, gx, gy, gz);
    }
    gyroCalibration.adjust(&gx, &gy, &gz);
    madgwickFilter.updateIMU(gx, gy, gz, ax, ay, az);
    tracker.update(ax, ay, az);

    madgwickPrevious += madgwickInterval;
  }
  if (madgwickPrevious > micros()) madgwickPrevious = micros();
};
