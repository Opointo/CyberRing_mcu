#include <U8g2lib.h>

#include "Adafruit_AHRS_Mahony.h"
#include "Adafruit_AHRS_Madgwick.h"
#include <Adafruit_AHRS_NXPFusion.h>
#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>

#include <SPI.h>
#include <Wire.h>
#include <SD.h>

class MyBoschSensor : public BoschSensorClass {

public:
  MyBoschSensor(TwoWire& wire = Wire)
    : BoschSensorClass(wire){};

protected:
  virtual int8_t configure_sensor(struct bmi2_dev* dev) {
    int8_t rslt;
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    struct bmi2_int_pin_config int_pin_cfg;
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    return rslt;
  }
};

MyBoschSensor myIMU(Wire1);

#define PANIC(msg)         \
  {                        \
    Serial.println((msg)); \
    while (1)              \
      ;                    \
  }
#define RASSERT(cond, msg) \
  if (!(cond))            \
    PANIC((msg));

File myFile;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const int chipSelect = 10;

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3] = { -20.80F, 10.19F, -18.41 };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.004, -0.006, 0.003 },
                                    { -0.006, 0.984, -0.004 },
                                    { 0.003, -0.004, 1.013 } };

float mag_field_strength = 44.75F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3] = { 0.0F, 0.0F, 0.0F };


// Mahony is lighter weight as a filter and should be used
// on slower systems
Adafruit_Mahony filter;
//Adafruit_Madgwick filter;
//Adafruit_NXPSensorFusion filter;

float accelX, accelY, accelZ,          // units m/s/s i.e. accelZ if often 9.8 (gravity)
  gyroX, gyroY, gyroZ,                 // units dps (degrees per second)
  gyroDriftX, gyroDriftY, gyroDriftZ,  // units dps
  magX, magY, magZ, mag_x, mag_y;

long lastInterval, lastTime, a;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.print("\n等着 \n");
  myIMU.begin();
  RASSERT(SD.begin(chipSelect), "你SD卡死了");

  calibrateIMU(250, 250);
  lastTime = micros();

  filter.begin(25);

  // u8g2.setI2CAddress(0x78);
  // ASSERT(u8g2.begin(), "你屏幕烧了");
  // u8g2.setFont(u8g2_font_profont12_mf);
}

void loop()
{
  static unsigned int file_count = 0;
  static SDLib::File myFile;
  // millis() 每50天溢出一次
  if((millis() / (1000*60*30) + 1) != file_count) { // 1000毫秒=1秒，60秒=1分钟，30分钟=30分钟
    if(file_count!=0) {myFile.close();}
    file_count = millis() / (1000*60*30) + 1;
    char filename[8+3+1+1];// 8+3文件名，1位点，一个\0
    sprintf(filename, "data_%d.csv", file_count);
    if(SD.exists(filename)) {
      SD.remove(filename);
    }
    myFile = SD.open(filename, FILE_WRITE); // [文件不存在的话会自动创建](https://www.arduino.cc/reference/en/libraries/sd/open/)。
    RASSERT(myFile, "md打不开");
  }

readIMU();
  unsigned long currentTime = micros();
  lastInterval = currentTime - lastTime;  // expecting this to be ~104Hz +- 4%
  lastTime = currentTime;

  // Apply mag offset compensation (base values in uTesla)
  float x = magX - mag_offsets[0];
  float y = magY - mag_offsets[1];
  float z = magZ - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyroX - gyro_zero_offsets[0];
  float gy = gyroY - gyro_zero_offsets[1];
  float gz = gyroZ - gyro_zero_offsets[2];

  // Update the filter
  filter.update(gx * dpsToRad, gy * dpsToRad, gz * dpsToRad,
                accelX, accelY, accelZ,
                mx, my, mz);

  // Print the orientation filter output
  if (millis() - a >= 250) {
    a += 250;
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();
    
    myFile.print(accelX);
    myFile.print(",");
    myFile.print(accelY);
    myFile.print(",");
    myFile.print(accelZ);
    myFile.print(",");
    myFile.print(gyroX);
    myFile.print(",");
    myFile.print(gyroY);
    myFile.print(",");
    myFile.print(gyroZ);
    myFile.print(",");
    myFile.print(magX);
    myFile.print(",");
    myFile.print(magY);
    myFile.print(",");
    myFile.print(magZ);
    myFile.print(",");
    myFile.print(roll);
    myFile.print(",");
    myFile.print(pitch);
    myFile.print(",");
    myFile.println(heading);
    /*
    // Print the orientation filter output in quaternions.
    // This avoids the gimbal lock problem with Euler angles when you get
    // close to 180 degrees (causing the model to rotate or flip, etc.)
   float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    Serial.print(millis());
    Serial.print(" - Quat: ");
    Serial.print(qw);
    Serial.print(" ");
    Serial.print(qx);
    Serial.print(" ");
    Serial.print(qy);
    Serial.print(" ");
    Serial.println(qz);
 */
  }
  // u8g2.clearBuffer();
  // u8g2.drawStr(0, 10, "I AM FUCKING ALIVE");
  // u8g2.sendBuffer();
  if (Serial.available())
  {
    myFile.flush();
    Serial.read();
    Serial.println("flush");
  }
}

void readIMU() {
  myIMU.readAcceleration(accelX, accelY, accelZ);
  myIMU.readGyroscope(gyroX, gyroY, gyroZ);
  myIMU.readMagneticField(magX, magY, magZ);
}

void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis);  // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    readIMU();
    // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    calibrationCount++;
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}