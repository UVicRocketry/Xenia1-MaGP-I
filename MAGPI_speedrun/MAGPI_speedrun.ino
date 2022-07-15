//libraries for BMP sensor
#include "Wire.h"

//libraries for BMP
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP3XX.h"

//libraries for MPU
#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"

//libraries for SD card logging
#include "SPI.h"
#include "SD.h"

//for motor
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

//libraries used for GPS
#include "SoftwareSerial.h"
#include <TinyGPS++.h>


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
double lat ,lon, lat_old, lon_old, current_time, speed, gps_alt, heading;
double elasped_time;
static const double TARGET_LAT = 12.123456, TARGET_LON = 12.123456;

double filtered_alt;

//for motor
#define EN 5 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //
int motor_position = -999; // to be zero by the the hall effect sensor  

//for algorithm
bool loop_valid = 1; //incase a reading is not valid
bool launched = 0;
bool deployed = 0;
double target_heading = 0.0;
double last_glide = 0.0;
const double delta_static = 1.0; ////////////change accordingly///////////
const double circumference = 1.0; ////////////change accordingly///////////
const double glide_interval = 1.0; ////////////change accordingly///////////
double turn_interval;

/*
struct Algorithm
{
  bool loop_valid; //incase a reading is not valid
  bool launched;
  bool deployed;
  double target_heading;
  double last_glide;
  const double delta_static; ////////////change accordingly///////////
  const double circumference;
  double turn_interval;
};
*/


#define filename "MAGPI.txt"

#define SEALEVELPRESSURE_HPA (1013.25)
#define delim "\t"
 


//BMP object 
Adafruit_BMP3XX bmp;
//MPU object 
Adafruit_MPU6050 mpu;
//File object
File myFile;
//GPS object
TinyGPSPlus myGPS;
//SoftwareSerial ss(34, 35); //Serial connection pins



//Set-up for the sensors to be run in setup() code 
void set_SD(){
  Serial.print("checkpoint 1");
  Wire.begin();
  Serial.print("checkpoint 1");
  Serial.print("Initializing SD card...");
  
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed for the SD CARD!");

  } else{
    Serial.println("SD Card Initializated");
  }
}

void set_bmp(){
  Serial.println("Adafruit BMP390 initialization");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    //while (1);
  }
  //filtering unecessary data
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); 
}

void set_mpu(){
  // Initialize
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  } else {
    Serial.println("MPU6050 was found");
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //set accleration range to 16G
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG); //set gyro range to 2000 deg/s

  Serial.print("Setting: Accel / Gyro:"); 
  Serial.print(mpu.getAccelerometerRange());
  Serial.print(delim);
  Serial.println(mpu.getGyroRange());
  
  //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); uncommmnet for final code with filter
  
  //setupt motion detection
  /* We will not be using, comment out if the code use int
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  */


}

void set_GPS(){
  Serial8.begin(9600);
  Serial.print("GPS succesfully initialized");
  
}

//functions for initialization and magnet detection in Hall Effect sensor
void set_hallEffect(){
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

void get_data(){
  while(Serial8.available()>0){ // check for gps data
    myGPS.encode(Serial8.read());// encode 
  }

  if (myGPS.location.isUpdated()){////////////need check, should perform .isUpdated() for all data?
    lat = myGPS.location.lat();
    lon = myGPS.location.lng();
    current_time = myGPS.time.value(); //UNIX time
    gps_alt = myGPS.altitude.meters();
    heading = myGPS.course.deg();
    speed = myGPS.speed.mps();

  } else{
    current_time = micros(); //////////need check//////////// should use gps time + micros
  }

  //mpu
  sensors_event_t a, w, temp;
  mpu.getEvent(&a, &w, &temp);

  //bmp
  //////check how to make the code better by class
  if (!bmp.performReading()) {
    Serial.println("BMP Failed"); 
  }
  myFile.print(bmp.temperature);
  

  myFile.print("Pressure = ");
  myFile.print(bmp.pressure / 100.0);
  myFile.println(" hPa");

  myFile.print("Approx. Altitude = ");
  myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  myFile.println(" m");

}


//Code functionality to be ran in the loop() 
void save_data(){
//read sensors
//GPS readings
  

  

  //mpu
  


  myFile= SD.open(filename, FILE_WRITE);
  myFile.print(millis());
  myFile.print(delim);


  //mpu data sensors readings
  
  myFile.print(a.acceleration.x); myFile.print(delim);
  myFile.print(a.acceleration.y); myFile.print(delim);
  myFile.print(a.acceleration.z); myFile.print(delim);
  myFile.print(w.gyro.x); myFile.print(delim);
  myFile.print(w.gyro.y); myFile.print(delim);
  myFile.print(w.gyro.z); myFile.print(delim);

  //add bmp



  myFile.close(); 

  //bmp sensor input
  /*
  if (!bmp.performReading()) {
    Serial.println("BMP Failed"); 
  }
  myFile.print("Temperature = ");
  myFile.print(bmp.temperature);
  myFile.println(" *C");

  myFile.print("Pressure = ");
  myFile.print(bmp.pressure / 100.0);
  myFile.println(" hPa");

  myFile.print("Approx. Altitude = ");
  myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  myFile.println(" m");
  */
  //hall effect readings
  
}

void algorithm(){
  int turn_no = 0;
  int dir = 0; // -1 == LEFT, 1== RIGHT
  double yaw_req = 0.0;
  target_heading = myGPS.courseTo(lat, lon, TARGET_LAT, TARGET_LON);
  yaw_req = target_heading - heading;
  
  //get principal angle
  if (yaw_req == 0.0){ 
    break;
  }
  if (yaw_req > 0.0){
    dir = 1;
    if (yaw_req > 180.0){
      yaw_req = -360.0 - yaw_req;
      dir = -1;
    }
  } else {
      dir = -1;
      if (yaw_req < -180.0){
        yaw_req = -360.0 + yaw_req;
        dir = 1;
      }
  }
  turn_no = (delta_static - (yaw_req/turn_interval /*should be an equation*/)/(1.0 /*should be an equation*/))/(circumference);//gear_ratio);/////update 

  //turn_dir = (target_heading < 0)? 0 : 1;
  //equations and constant, drop test needed + motor 

  //spin

  //wait for some time

  //glide

}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting set up");
  set_SD();
  set_bmp();
  set_mpu();
  set_hallEffect();
  //add one set the motor to high impendace mode to prevent spinning
  
  /*
  while(1){
        check lunch condition (use Jam Jar one);
        if(launched){ ////////////state///////////////
            break;
        }
    }

    while(launched && !deployed){ /////////////state/////////////
        get data;
        store data;
  */
 ///////////////change pin/////////////////
  Encoder motor(39,40); // delcared encoder after deployed to prevent triggering ISR 

}

void loop() {
  //save_input();
  //detect_Hall();
  //find required yaw
  if ((current_time - last_glide)> glide_interval){
    algorithm();
  }
  
  }
