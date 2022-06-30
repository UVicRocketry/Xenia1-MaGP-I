//libraries for BMP sensor
#include <Wire.h>
#include <SPI.h>
//libraries for BMP
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

//libraries for MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//libraries for SD card logging
#include <SPI.h>
#include <SD.h>

//libraries used for GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>


//variables used for BMP
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

//variables used for Hall effect sensor
 volatile byte half_revolutions;
 unsigned int rpm;
 unsigned long timeold;

 //variables used for gps
 float lat ,lon ; 


#define SEALEVELPRESSURE_HPA (1013.25)
 


//BMP object 
Adafruit_BMP3XX bmp;
//MPU object 
Adafruit_MPU6050 mpu;
//File object
File dataLog;
//GPS object
TinyGPS gps;
SoftwareSerial ss(6, 5); //Serial connection pins



//Set-up for the sensors to be run in setup() code 
void set_SD(){
  Serial.print("checkpoint 1");
   Wire.begin();
   SD.begin(115200);
    Serial.print("checkpoint 1");
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    
   Serial.println("initialization failed for the SD CARD!");
  while (1);
      Serial.print("checkpoint 2");

  }
   Serial.print("checkpoint 3");

  Serial.println("initialization done."); 
}


void set_bmp(){
  
  //set up-bmp to connect through serial
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP390 initialization");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  //filtering unecessary data
   bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); 
}

void set_mpu(){
  Serial.begin(115200);
  while (!Serial)
    delay(10); 


  // Initialize
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 was found");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
  
}

void set_GPS(){
  Serial.begin(115200);
  ss.begin(9600);
  Serial.print("GPS succesfully initialized");
  
}

//functions for initialization and magnet detection in Hall Effect sensor
void set_hallEffect(){
   Serial.begin(115200);
   half_revolutions = 0;
   rpm = 0;
   timeold = 0;
 }

 void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
   half_revolutions++;
   Serial.println("detect");
  
}

void detect_Hall(){
    if (half_revolutions >= 20) { 
     rpm = 30*1000/(millis() - timeold)*half_revolutions;
     timeold = millis();
     half_revolutions = 0;
     //Serial.println(rpm,DEC);
   }
  
}


//Code functionality to be ran in the loop() 
void save_input(){
  dataLog= SD.open("data.csv", FILE_WRITE);
  dataLog.print(millis());
  dataLog.print("\t");

  //bmp sensor input
  
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  dataLog.print("Temperature = ");
  dataLog.print(bmp.temperature);
  dataLog.println(" *C");

  dataLog.print("Pressure = ");
  dataLog.print(bmp.pressure / 100.0);
  dataLog.println(" hPa");

  dataLog.print("Approx. Altitude = ");
  dataLog.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  dataLog.println(" m");

  //mpu data sensors readings

    if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    dataLog.print("MOTION DETECTION DATA");
    dataLog.print("\t");
    dataLog.print("AccelX:");
    dataLog.print(a.acceleration.x);
    dataLog.print(",");
    dataLog.print("AccelY:");
    dataLog.print(a.acceleration.y);
    dataLog.print(",");
    dataLog.print("AccelZ:");
    dataLog.print(a.acceleration.z);
    dataLog.print(", ");
    dataLog.print("GyroX:");
    dataLog.print(g.gyro.x);
    dataLog.print(",");
    dataLog.print("GyroY:");
    dataLog.print(g.gyro.y);
    dataLog.print(",");
    dataLog.print("GyroZ:");
    dataLog.print(g.gyro.z);
    dataLog.println("");
  }

  //hall effect readings

  
  //GPS readings
  while(ss.available()){ // check for gps data
    if(gps.encode(ss.read()))// encode 
    { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude
    dataLog.print("\t");
    dataLog.print("LATITUDE");
    dataLog.print(lat);
    dataLog.print("\t");
    dataLog.print("LONGITUDE");
    dataLog.print(lon);
    dataLog.println("");
    }
  }

  

  dataLog.close(); 
}



void setup() {
  Serial.println("Starting set up");
  set_SD();
  set_bmp();
  set_mpu();
  set_hallEffect();
  

}

void loop() {
  save_input();
  detect_Hall();

  
  }
