#include <U8g2lib.h>

#include "Arduino_BMI270_BMM150.h"

#include <SPI.h>
#include <Wire.h>
#include <SD.h>

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

struct IMUData
{
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
};

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.print("\n等着 \n");
  RASSERT(IMU.begin(), "你IMU炸了");
  RASSERT(SD.begin(chipSelect), "你SD卡死了");

  // u8g2.setI2CAddress(0x78);
  // ASSERT(u8g2.begin(), "你屏幕烧了");
  // u8g2.setFont(u8g2_font_profont12_mf);
}

void loop()
{
  static IMUData imu_d = {0};
  static unsigned long last_report_time = 0; // 上次输出时间
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
  if (readIMU(&imu_d))
  {
    Serial.print("Accel:");
    Serial.print(imu_d.accel_x);
    Serial.print(",");
    Serial.print(imu_d.accel_y);
    Serial.print(",");
    Serial.print(imu_d.accel_z);
    Serial.print("  Gyro:");
    Serial.print(imu_d.gyro_x);
    Serial.print(",");
    Serial.print(imu_d.gyro_y);
    Serial.print(",");
    Serial.print(imu_d.gyro_z);
    Serial.print("  Mag:");
    Serial.print(imu_d.mag_x);
    Serial.print(",");
    Serial.print(imu_d.mag_y);
    Serial.print(",");
    Serial.print(imu_d.mag_z);

    myFile.print(imu_d.accel_x);
    myFile.print(",");
    myFile.print(imu_d.accel_y);
    myFile.print(",");
    myFile.print(imu_d.accel_z);
    myFile.print(",");
    myFile.print(imu_d.gyro_x);
    myFile.print(",");
    myFile.print(imu_d.gyro_y);
    myFile.print(",");
    myFile.print(imu_d.gyro_z);
    myFile.print(",");
    myFile.print(imu_d.mag_x);
    myFile.print(",");
    myFile.print(imu_d.mag_y);
    myFile.print(",");
    myFile.print(imu_d.mag_z);
    unsigned long interval; // 本次输出所用时间
    // micros() 最长记录70分钟，超过会溢出
    // 此处最好能暂停 micros(), 不然可能遇到奇怪的边界问题
    if (micros() > last_report_time)
    {
      interval = micros() - last_report_time;
    }
    else
    {
      interval = micros() + (0xffffffff - last_report_time);
    }
    last_report_time = micros();
    Serial.print(",");
    Serial.println(interval);
    myFile.print(",");
    myFile.println(interval);
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

int readIMU(IMUData *d)
{
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable())
  {
    IMU.readAcceleration(d->accel_x, d->accel_y, d->accel_z);
    IMU.readGyroscope(d->gyro_x, d->gyro_y, d->gyro_z);
    if (IMU.magneticFieldAvailable())
    {
      IMU.readMagneticField(d->mag_x, d->mag_y, d->mag_z);
    }
    return 1;
  }
  return 0;
}