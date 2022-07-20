/* Info
#####  Sensors/Input  #####
Data/Signal -> Sensor <Library> [protocol]

Numerical Data:
- GPS -> neo GPS <TinyGPS++> [Serial Port8]
- Accelration, Gyro -> MPU6050 <Adafruit_sensor.h> [I2C]
- Alt, Pressure, Temp -> BMP390 <Adafurit> [I2C]

Logic Signal:
- Gear Motor Position/Step -> Encoder <Encoder.h> [Interrupt/ISR]
- Pulley RPM -> Hall Effect Sensor [Interrupt/ISR] 
    Output HIGH when Magnetic Flux increase, therefore the ISR condition is RISING

Interrupts must be handled with caution as it will insert a new command after
the current command. Interrupts may not be ideal and should be detached/disable 
during certain operation, writing data to SD card is a good example. 


##### Actuator/Output #####
Gear Motor with L293D to control line deflection
L293D I/O
Channel 1_3A I
Channel 2_4A I
Channel EN   I
Channel 1_3Y O
Channel 2_4Y O

Logic Table:
A  EN  Y
H  H   H
L  H   L
X  L   Z (High Impedance)


##### Top Level Control Flow (without logic) #####
get data -> calculate -> run control algorithm -> store data 
*/

/*Gordon to do
- double check if there is any code of the algoritm used Radians -> use degrees
- check algorthim factor in overshoot angle that is >360 and <-360
- complete logic then add time_out routines
- Algorthim data writing: Encoder, P controller output, every step

*/

#include "Wire.h" //I2C
#include "Adafruit_Sensor.h" 
#include "Adafruit_BMP3XX.h"
#include "Adafruit_MPU6050.h"

#include "SPI.h"
#include "SD.h"

//for motor encoder
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

//GPS
#include <TinyGPS++.h>

//states
bool launched = 0;
bool deployed = 0;
bool test_status = 0;

//variables used for Hall effect sensor /////////////need checking///////////
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
Encoder motor(31, 32); 

//for algorithm
bool loop_valid = 1; //incase a reading is not valid
double target_heading = 0.0;
double last_glide = 0.0;
const double delta_static = 1.0; ////////////change accordingly///////////
const double turn_ratio = 1.0; ////////////change accordingly///////////
const double glide_interval = 1.0; ////////////change accordingly///////////
double turn_interval = 1.0; ////////////change accordingly///////////
double max_cord = 50; //50mm
int dir = 0;
const int control_timeout;
double yaw = 0.0;
const double yaw_control_cutoff;
unsigned int void_timestamp = 0;
double error = 0.0;
const double k_p = 5.0/180.0; // max length/180 degree 
unsigned int delta_time = 0;
unsigned int last_time = 0;

/*
struct Algorithm ///////////should use class OR split into multiple .ino file ///////////////
{
  bool loop_valid; //incase a reading is not valid
  bool launched;
  bool deployed;
  double target_heading;
  double last_glide;
  const double delta_static; ////////////change accordingly///////////
  const double turn_ratio;
  double turn_interval;
};
*/

#define filename "MAGPI.txt"
#define delim ","

#define SEALEVELPRESSURE_HPA (1013.25) ////////////double check the purpose of this definition

//BMP
Adafruit_BMP3XX bmp;
//MPU
Adafruit_MPU6050 mpu;
sensors_event_t a, w, temp;
//File object
File myFile;
//GPS object
TinyGPSPlus myGPS;


//Set-up for the sensors to be run in setup() code 
void set_SD(){
  Wire.begin();
  Serial.print("Initializing SD card...");
  
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed for the SD CARD!");

  } else{
    Serial.println("SD Card Initializated");
  }
}

void set_bmp(){
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("BMP not found");
  } else {
    Serial.println("BMP OK");
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
    Serial.println("MPU not found");
  } else {
    Serial.println("MPU OK");
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
  if(Serial8){
    Serial.print("GPS succesfully initialized");
  } else{
    Serial.print("Serial8 not open");
  }
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
  //need to implement logic to see if data is valid:
  //TinyGPS++ -> isUpdate on class
  //mpu -> bool getEvent() [Logic need update, should not give a state to the whole loop]
  //bmp -> bool performReading [Logic need update, should not give a state to the whole loop]
  
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
    current_time = micros(); //////////need check//////////// should use gps time + micros OR just treat it as another variable
  }

  loop_valid = (mpu.getEvent(&a, &w, &temp))? 1 : 0;
  loop_valid = (bmp.performReading())? 1 : 0;
}


//Code functionality to be ran in the loop() 
void save_data(){
  //Use Serial for DEBUG [add pause function]
  myFile = SD.open(filename, FILE_WRITE);
  myFile.print(millis());
  myFile.print(delim);

  //mpu data sensors readings
  
  myFile.print(a.acceleration.x); myFile.print(delim);
  myFile.print(a.acceleration.y); myFile.print(delim);
  myFile.print(a.acceleration.z); myFile.print(delim);
  myFile.print(w.gyro.x); myFile.print(delim);
  myFile.print(w.gyro.y); myFile.print(delim);
  myFile.print(w.gyro.z); myFile.print(delim);

  //bmp
  myFile.print(bmp.temperature); myFile.print(delim);
  myFile.print(bmp.pressure); myFile.print(delim);
  myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); myFile.print(delim);

  myFile.print(lat); myFile.print(delim);
  myFile.print(lon); myFile.println(""); 

  myFile.close(); 
  
}

void motor_left(){
  digitalWrite(EN, HIGH);
  digitalWrite(A1_3, HIGH);
  digitalWrite(A2_4, LOW);
}

void motor_right(){

}

void algorithm(){
  int turn_no = 0;
  double yaw_req = 0.0;
  target_heading = myGPS.courseTo(lat, lon, TARGET_LAT, TARGET_LON);
  yaw_req = (target_heading - heading);
  
  //get principal angle
  if (yaw_req > 0.0){
    dir = 1;
    if (yaw_req > 180.0){
      yaw_req = -360.0 - yaw_req;
      dir = -1;
    }
  } else if (yaw_req < 0.0){
      dir = -1;
      if (yaw_req < -180.0){
        yaw_req = -360.0 + yaw_req;
        dir = 1;
      }
  } else {}
  



// get yaw difference -> P controller
// P controller needs -> current yaw               /////need to set the map of the pulley////////// 

yaw = 0.0;
void_timestamp = micros();

while (!(((error = yaw_req - yaw)<= yaw_control_cutoff)&&(error >= -yaw_control_cutoff)) && ((micros()- void_timestamp) < control_timeout)) {
 
  last_time = micros(); //t_n-1
  turn_no = error*turn_ratio;//P controller = SetPoint - actual yaw      /////////need to check the ratio of turns to actuated distance
  while(turn_no != 0){ 
    //turn motor in the direction needed
    if(turn_no > 0){  
      digitalWrite(EN, HIGH);
      digitalWrite(A1_3, HIGH);
      digitalWrite(A2_4, LOW);
    } else if (turn_no < 0){
      digitalWrite(EN, HIGH);
      digitalWrite(A1_3, LOW);
      digitalWrite(A2_4, HIGH);
    }

    //getGyro
    
    turn_no -= motor.read(); // to check if motion is completed(check step difference)
  }
    get_data();
    delta_time = micros()- last_time; //t_n - t_n-1
    yaw += w.gyro.z * delta_time; //intergral ///yaw angle is accumulated  ////////change to actual axis + filtering + axis offset calibration 
    
    //get -360 < yaw < 360
    if (yaw >360.0) {
      yaw += -360.0;
    } else {
      yaw += 360.0;
    }
    

}
  //spin
  
  digitalWrite(EN, LOW); //disable motor when glide
  digitalWrite(A1_3, LOW);
  digitalWrite(A2_4, LOW);
  //wait for some time ///////////// add//////////////

  //glide
  last_glide = 0.0;//time now ////////use unix time or Micros()?

}

bool high_G(){
    /*
    Adafruit_MPU6050_Accelerometer::getSensor(sensor_t &a);
    if ((a.acceleration.x ^2 + a.acceleration.y ^2 + a.acceleration.z ^2 ) >= 25){
      return 1;
    } else {
      return 0;
    }
    */
  return 0;
  }

bool is_deploy(){
  return 0;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Set up");

  //pin setup for controlling motor
  pinMode(EN, OUTPUT);
  pinMode(A1_3, OUTPUT);
  pinMode(A2_4, OUTPUT);
  digitalWrite(EN, LOW); //High impendence mode
  digitalWrite(A1_3, LOW);
  digitalWrite(A2_4, LOW);

  set_SD();
  set_bmp();
  set_mpu();
  set_hallEffect();
  set_GPS();
  get_data();
  save_data();
  //add one set the motor to high impendace mode to prevent spinning
  //add set sensor to sleep OR not active
  
  while(!launched){
    void_timestamp = millis();
    while(high_G() && !launched){  ///////////check if this is in Gs
      launched = ((millis()- void_timestamp) > 200)? 1 : 0;
    }

  while(launched && !deployed){ /////////////state/////////////
    get_data();
    save_data();
    //add rasperry pi camera start
  }
  ///////////////change pin/////////////////
  }
  
}

void loop() {
  //save_input();
  //detect_Hall();
  //find required yaw
  if ((current_time - last_glide)> glide_interval){
    algorithm();
  }
  
}
