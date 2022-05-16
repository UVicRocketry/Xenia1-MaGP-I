// global variablex and libraries declarations 
#include "PetitFS.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
//object initialization
Adafruit_BMP3XX bmp;
Adafruit_MPU6050 mpu;
File dataGathered;
int pinCS=29; //pin 29 on SPI 

void setup() {
  start_bmp390();
  start_mpu();
  start_sdcard();
}

void loop() {

}

//functions to control BMP sensor

void start_bmp390(){
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}


//functions for the MPU6050 sensor
void start_mpu(){
Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void run_both(){
 if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }  
   /* Get new sensor events with the readings for mpu */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.print(" *C");
  Serial.print("\t");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
  Serial.print("\t");


  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.print("\t");

  delay(2000);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
   Serial.print("\t");
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
   Serial.print("\t");
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  Serial.print("\t");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
   Serial.print("\t");
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print("\t");
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.print("\t");
  Serial.println(" rad/s");
  Serial.print("\t");

  delay(2000);

}

void  start_sdcard(){
  Serial.begin(115200);
  pinMode(pinCS,OUTPUT);

  //Start SD Card
  if (SD.begin()){
    Serial.println("SD Card initialization was successful");
  } else{
    Serial.println("SD Card could not be initialized");
    return;
  }

  //start and open the SD card
   dataGathered= SD.open("dataGathered.csv", FILE_WRITE);
   //Write on file
   if(dataGathered){
    run_both();   
  } else{
    Serial.println("an error ocurred");
  }
  delay(3000);
}
