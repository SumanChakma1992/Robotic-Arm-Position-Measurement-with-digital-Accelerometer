#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ArduinoJson.h>

MPU6050 mpu1(0x68); // Default I2C address
MPU6050 mpu2(0x69); // Alternate I2C address
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

const int numReadings = 10; // Number of readings for the moving average

// Arrays to store readings for moving average
float mpu1_axReadings[numReadings];
float mpu1_ayReadings[numReadings];
float mpu1_azReadings[numReadings];
float mpu1_gxReadings[numReadings];
float mpu1_gyReadings[numReadings];
float mpu1_gzReadings[numReadings];

float mpu2_axReadings[numReadings];
float mpu2_ayReadings[numReadings];
float mpu2_azReadings[numReadings];
float mpu2_gxReadings[numReadings];
float mpu2_gyReadings[numReadings];
float mpu2_gzReadings[numReadings];

float adxl_axReadings[numReadings];
float adxl_ayReadings[numReadings];
float adxl_azReadings[numReadings];

int readIndex = 0;
float total_mpu1_ax = 0;
float total_mpu1_ay = 0;
float total_mpu1_az = 0;
float total_mpu1_gx = 0;
float total_mpu1_gy = 0;
float total_mpu1_gz = 0;

float total_mpu2_ax = 0;
float total_mpu2_ay = 0;
float total_mpu2_az = 0;
float total_mpu2_gx = 0;
float total_mpu2_gy = 0;
float total_mpu2_gz = 0;

float total_adxl_ax = 0;
float total_adxl_ay = 0;
float total_adxl_az = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Wire.begin();

  // Adding a delay to ensure the serial connection is fully established
  delay(1000);

  // Initialize first MPU6050 sensor
  mpu1.initialize();
  if (mpu1.testConnection()) {
    sendInitStatus("MPU6050 #1 connection successful");
  } else {
    sendInitStatus("MPU6050 #1 connection failed");
  }

  // Initialize second MPU6050 sensor
  mpu2.initialize();
  if (mpu2.testConnection()) {
    sendInitStatus("MPU6050 #2 connection successful");
  } else {
    sendInitStatus("MPU6050 #2 connection failed");
  }

  // Initialize ADXL345 sensor
  if (adxl.begin()) {
    sendInitStatus("ADXL345 connection successful");
  } else {
    sendInitStatus("ADXL345 connection failed");
  }

  // Calibrate the first sensor
  if (mpu1.testConnection()) {
    mpu1.CalibrateAccel(6);
    mpu1.CalibrateGyro(6);
    sendInitStatus("Calibration completed for MPU6050 #1");
  }

  // Calibrate the second sensor
  if (mpu2.testConnection()) {
    mpu2.CalibrateAccel(6);
    mpu2.CalibrateGyro(6);
    sendInitStatus("Calibration completed for MPU6050 #2");
  }

  // Initialize readings arrays to zero
  for (int i = 0; i < numReadings; i++) {
    mpu1_axReadings[i] = 0;
    mpu1_ayReadings[i] = 0;
    mpu1_azReadings[i] = 0;
    mpu1_gxReadings[i] = 0;
    mpu1_gyReadings[i] = 0;
    mpu1_gzReadings[i] = 0;

    mpu2_axReadings[i] = 0;
    mpu2_ayReadings[i] = 0;
    mpu2_azReadings[i] = 0;
    mpu2_gxReadings[i] = 0;
    mpu2_gyReadings[i] = 0;
    mpu2_gzReadings[i] = 0;

    adxl_axReadings[i] = 0;
    adxl_ayReadings[i] = 0;
    adxl_azReadings[i] = 0;
  }
}

void sendInitStatus(const char* status) {
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  String output;
  serializeJson(doc, output);
  Serial.print("<JSON>");
  Serial.print(output);
  Serial.println("</JSON>");
}

void sendSensorData() {
  // Read accelerometer and gyroscope data from first MPU6050
  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  float g_ax1 = ax1 / 16384.0;
  float g_ay1 = ay1 / 16384.0;
  float g_az1 = az1 / 16384.0;
  float dps_gx1 = gx1 / 131.0;
  float dps_gy1 = gy1 / 131.0;
  float dps_gz1 = gz1 / 131.0;

  // Read accelerometer and gyroscope data from second MPU6050
  int16_t ax2, ay2, az2, gx2, gy2, gz2;
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  float g_ax2 = ax2 / 16384.0;
  float g_ay2 = ay2 / 16384.0;
  float g_az2 = az2 / 16384.0;
  float dps_gx2 = gx2 / 131.0;
  float dps_gy2 = gy2 / 131.0;
  float dps_gz2 = gz2 / 131.0;

  // Read sensor data from ADXL345
  sensors_event_t event;
  adxl.getEvent(&event);

  // Update moving average arrays
  total_mpu1_ax -= mpu1_axReadings[readIndex];
  total_mpu1_ay -= mpu1_ayReadings[readIndex];
  total_mpu1_az -= mpu1_azReadings[readIndex];
  total_mpu1_gx -= mpu1_gxReadings[readIndex];
  total_mpu1_gy -= mpu1_gyReadings[readIndex];
  total_mpu1_gz -= mpu1_gzReadings[readIndex];

  total_mpu2_ax -= mpu2_axReadings[readIndex];
  total_mpu2_ay -= mpu2_ayReadings[readIndex];
  total_mpu2_az -= mpu2_azReadings[readIndex];
  total_mpu2_gx -= mpu2_gxReadings[readIndex];
  total_mpu2_gy -= mpu2_gyReadings[readIndex];
  total_mpu2_gz -= mpu2_gzReadings[readIndex];

  total_adxl_ax -= adxl_axReadings[readIndex];
  total_adxl_ay -= adxl_ayReadings[readIndex];
  total_adxl_az -= adxl_azReadings[readIndex];

  mpu1_axReadings[readIndex] = g_ax1;
  mpu1_ayReadings[readIndex] = g_ay1;
  mpu1_azReadings[readIndex] = g_az1;
  mpu1_gxReadings[readIndex] = dps_gx1;
  mpu1_gyReadings[readIndex] = dps_gy1;
  mpu1_gzReadings[readIndex] = dps_gz1;

  mpu2_axReadings[readIndex] = g_ax2;
  mpu2_ayReadings[readIndex] = g_ay2;
  mpu2_azReadings[readIndex] = g_az2;
  mpu2_gxReadings[readIndex] = dps_gx2;
  mpu2_gyReadings[readIndex] = dps_gy2;
  mpu2_gzReadings[readIndex] = dps_gz2;

  adxl_axReadings[readIndex] = event.acceleration.x;
  adxl_ayReadings[readIndex] = event.acceleration.y;
  adxl_azReadings[readIndex] = event.acceleration.z;

  total_mpu1_ax += mpu1_axReadings[readIndex];
  total_mpu1_ay += mpu1_ayReadings[readIndex];
  total_mpu1_az += mpu1_azReadings[readIndex];
  total_mpu1_gx += mpu1_gxReadings[readIndex];
  total_mpu1_gy += mpu1_gyReadings[readIndex];
  total_mpu1_gz += mpu1_gzReadings[readIndex];

  total_mpu2_ax += mpu2_axReadings[readIndex];
  total_mpu2_ay += mpu2_ayReadings[readIndex];
  total_mpu2_az += mpu2_azReadings[readIndex];
  total_mpu2_gx += mpu2_gxReadings[readIndex];
  total_mpu2_gy += mpu2_gyReadings[readIndex];
  total_mpu2_gz += mpu2_gzReadings[readIndex];

  total_adxl_ax += adxl_axReadings[readIndex];
  total_adxl_ay += adxl_ayReadings[readIndex];
  total_adxl_az += adxl_azReadings[readIndex];

  readIndex++;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  float avg_mpu1_ax = total_mpu1_ax / numReadings;
  float avg_mpu1_ay = total_mpu1_ay / numReadings;
  float avg_mpu1_az = total_mpu1_az / numReadings;
  float avg_mpu1_gx = total_mpu1_gx / numReadings;
  float avg_mpu1_gy = total_mpu1_gy / numReadings;
  float avg_mpu1_gz = total_mpu1_gz / numReadings;

  float avg_mpu2_ax = total_mpu2_ax / numReadings;
  float avg_mpu2_ay = total_mpu2_ay / numReadings;
  float avg_mpu2_az = total_mpu2_az / numReadings;
  float avg_mpu2_gx = total_mpu2_gx / numReadings;
  float avg_mpu2_gy = total_mpu2_gy / numReadings;
  float avg_mpu2_gz = total_mpu2_gz / numReadings;

  float avg_adxl_ax = total_adxl_ax / numReadings;
  float avg_adxl_ay = total_adxl_ay / numReadings;
  float avg_adxl_az = total_adxl_az / numReadings;

  // Create a JSON document
  StaticJsonDocument<800> doc;
  doc["mpu1"]["ax"] = avg_mpu1_ax;
  doc["mpu1"]["ay"] = avg_mpu1_ay;
  doc["mpu1"]["az"] = avg_mpu1_az;
  doc["mpu1"]["gx"] = avg_mpu1_gx;
  doc["mpu1"]["gy"] = avg_mpu1_gy;
  doc["mpu1"]["gz"] = avg_mpu1_gz;

  doc["mpu2"]["ax"] = avg_mpu2_ax;
  doc["mpu2"]["ay"] = avg_mpu2_ay;
  doc["mpu2"]["az"] = avg_mpu2_az;
  doc["mpu2"]["gx"] = avg_mpu2_gx;
  doc["mpu2"]["gy"] = avg_mpu2_gy;
  doc["mpu2"]["gz"] = avg_mpu2_gz;

  doc["adxl"]["ax"] = avg_adxl_ax;
  doc["adxl"]["ay"] = avg_adxl_ay;
  doc["adxl"]["az"] = avg_adxl_az;

  // Serialize JSON to a String
  String data;
  serializeJson(doc, data);

  // Print JSON string to serial monitor
  Serial.print("<JSON>");
  Serial.print(data);
  Serial.println("</JSON>");
}

void loop() {
  // Continuously send sensor data
  sendSensorData();
  delay(100); // Adjust delay as needed
}

