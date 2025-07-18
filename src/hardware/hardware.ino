#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include "SparkFun_MMC5983MA_Arduino_Library.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define TXD2 17
#define RXD2 16

SFE_MMC5983MA mag;
uint32_t rawX, rawY, rawZ;
double magX, magY, magZ;

SparkFun_ISM330DHCX imu;
sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

void setup() {
	Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); 
  Wire.begin(SDA_PIN, SCL_PIN);
	
	// Initialize ISM330DHCX
  while (!imu.begin()) {
    Serial.println("Failed to detect ISM330DHCX.");
    delay(500);
  }
  Serial.println("ISM330DHCX initialized.");

  imu.setDeviceConfig();
  imu.setBlockDataUpdate();

  // Accelerometer settings
  imu.setAccelFullScale(ISM_2g);
  imu.setAccelFilterLP2();
  imu.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
  imu.setAccelDataRate(ISM_XL_ODR_104Hz);

  // Gyroscope settings
  imu.setGyroFullScale(ISM_250dps);
  imu.setGyroFilterLP1();
  imu.setGyroLP1Bandwidth(ISM_MEDIUM);
  imu.setGyroDataRate(ISM_GY_ODR_104Hz);

	// Initialize MMC5983MA
  while (!mag.begin()) {
    Serial.println("Failed to detect MMC5983MA.");
    delay(500);
  }
  Serial.println("MMC5983MA initialized.");

  mag.softReset();
  mag.setContinuousModeFrequency(100);  // 100 Hz
  mag.enableContinuousMode();           // Begin continuous measurement
}

void loop() {
  // while (Serial2.available()) {
  //   char c = Serial2.read();
  //   Serial.print(c); 
  // }

  // --- Read Magnetometer ---
  mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

  // raw to uT
  magX = (((double)rawX - 131072.0) / 131072.0) * 800;
  magY = (((double)rawY - 131072.0) / 131072.0) * 800;
  magZ = -(((double)rawZ - 131072.0) / 131072.0) * 800;

  // Serial2.println("Hello from ESP32!");
  Serial2.flush();

  Serial2.print("(");
  Serial2.print(magX, 5); Serial2.print(",");
  Serial2.print(magY, 5); Serial2.print(",");
  Serial2.print(magZ, 5); Serial2.print(") ");

  // --- Read Accelerometer & Gyroscope ---
  if (imu.checkStatus()) {
    imu.getAccel(&accelData); // Raw data
    imu.getGyro(&gyroData);   // Raw data

    // convert raw to g and dps
    float ax_g = accelData.xData / 1000;
    float ay_g = accelData.yData / 1000; 
    float az_g = accelData.zData / 1000;

    float gx_dps = gyroData.xData / 1000;
    float gy_dps = gyroData.yData / 1000;
    float gz_dps = gyroData.zData / 1000;

    Serial2.print("(");
    Serial2.print(ax_g, 5); Serial2.print(",");
    Serial2.print(ay_g, 5); Serial2.print(",");
    Serial2.print(az_g, 5); Serial2.print(") ");

    Serial2.print("(");
    Serial2.print(gx_dps, 5); Serial2.print(",");
    Serial2.print(gy_dps, 5); Serial2.print(",");
    Serial2.print(gz_dps, 5); Serial2.println(")");

  }

  delay(100); // ~10Hz
}
