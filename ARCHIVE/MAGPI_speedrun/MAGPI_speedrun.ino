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
- Start up: Use serial port???
- complete logic then add time_out routines
- Algorthim data writing: Encoder, P controller output, every step
- check sensor sampling rate
- set sensor range

*/

/* Time Routine
///////// use two elaspe time?

bool reset = 0;
while(Serial8.available()>0){ // check for gps data
    myGPS.encode(Serial8.read());// encode 
    if(!reset){
      elaspsedMicros since_last_GPS; //reset time stamp when first GPS string package is recieved
      reset = 1;
    }
  }
the data file will look like (subject to change)

GPS time, time since_last_GPS
### GPS received ###
10:00:00, 0, data1, data2, .....
10:00:00, 10, data1, data2, .....
*/

#include "Wire.h" //I2C
#include "Adafruit_Sensor.h" 
#include "Adafruit_BMP3XX.h"
#include "Adafruit_MPU6050.h"

#include "SPI.h"
#include "SD.h"

#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

//GPS
#include <TinyGPS++.h>

//Algorthim
#include <math.h>

//states
bool launched = 0;
bool deployed = 0;
bool test_status = 0;

//variables used for Hall effect sensor /////////////need checking///////////
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

//variables used for GPS
double lat ,lon, lat_old, lon_old, current_time, speed, gps_alt, heading;
double elasped_time;
static const double TARGET_LAT = 12.123456, TARGET_LON = 12.123456;

double filtered_alt;

//motor
#define EN 5 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //
int motor_position = -999; // to be zero by the the hall effect sensor  
Encoder motor(31, 32); 

//algorithm
bool loop_valid = 1; //incase a reading is not valid
double target_heading = 0.0;
double last_glide = 0.0;
const double delta_static = 1.0; ////////////change accordingly///////////
//const double turn_ratio = 1.0; ////////////change accordingly///////////
const double glide_interval = 1.0; ////////////change accordingly///////////
double turn_interval = 1.0; ////////////change accordingly///////////
double max_cord = 50; //50mm
int dir = 0;
const int control_timeout;
double yaw = 0.0;
const double yaw_control_cutoff;
unsigned int void_timestamp = 0;
double error = 0.0;
const double k_p = 3.0/180.0; // max length/180 degree 
unsigned int delta_time = 0;
unsigned int last_time = 0;
const float alt_cutoff = 1.0; ////////change

#define filename "MAGPI.txt"
#define delim ","

#define SEALEVELPRESSURE_HPA (1013.25) ////////////double check the purpose of this definition

//BMP
Adafruit_BMP3XX bmp;
float alt_bmp = 0.0;
//MPU
Adafruit_MPU6050 mpu;
sensors_event_t a, w, temp;
//File object
File myFile;
//GPS object
TinyGPSPlus myGPS;


#define REAL

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

  #ifdef DEBUG
  if(Serial8){
    Serial.print("GPS succesfully initialized");
  } else{
    Serial.print("Serial8 not open");
  }
  #endif
}

void get_data(){
  //need to implement logic to see if data is valid:
  //TinyGPS++ -> isUpdate on class
  //mpu -> bool getEvent() [Logic need update, should not give a state to the whole loop]
  //bmp -> bool performReading [Logic need update, should not give a state to the whole loop]
  #ifdef REAL
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
  /*
  loop_valid = (mpu.getEvent(&a, &w, &temp))? 1 : 0;
  loop_valid = (bmp.performReading())? 1 : 0;
  */
  mpu.getEvent(&a, &w, &temp);
  bmp.performReading(); //write to public attributes temperature and pressure
  alt_bmp = bmp.readAltitude(SEALEVELPRESSURE_HPA); 
  #endif

  #ifdef DEBUG_SERIAL_INPUT

  #endif
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
  myFile.print(alt_bmp); myFile.print(delim);

  myFile.print(lat); myFile.print(delim);
  myFile.print(lon); myFile.println(""); 

  myFile.close(); 
  
}

double principal_angle(double angle){
    if (angle > 180.0){
      angle += -360.0;
    } else if (angle < -180.0){
      angle += 360.0;
    } else if (angle == 360.0 || angle == -360.0){
      angle = 0.0;
    }
  return angle;
}

void rotate_motor(int turns){
  while(turns != 0){ 
  //turn motor in the direction needed
  if(turns > 0){  
    digitalWrite(EN, HIGH); ////// check to see if have time to put a PID routine here //////
    digitalWrite(A1_3, HIGH);
    digitalWrite(A2_4, LOW);
  } else if (turns < 0){
    digitalWrite(EN, HIGH);
    digitalWrite(A1_3, LOW);
    digitalWrite(A2_4, HIGH);
  }
  /////////////////// get gyro??? OR write to file???
  turns -= motor.read(); 
  }
  //return turns;
}

void algorithm(){
  int turn_no = 0;
  int turn_reset = 0;
  double yaw_req = 0.0;
  target_heading = myGPS.courseTo(lat, lon, TARGET_LAT, TARGET_LON);
  yaw_req = (target_heading - heading);
  yaw_req = principal_angle(yaw_req);
  
// get yaw difference -> P controller
// P controller needs -> current yaw               /////need to set the map of the pulley////////// 

  yaw = 0.0;
  /*elapsedMicros*/  int algorithm_time; //////////change back to elasped micros   //reset algorithm_time to find time passed and for integration

  while (!(((error = yaw_req - yaw)<= yaw_control_cutoff)&&(error >= -yaw_control_cutoff)) && (algorthim_time < control_timeout)) {
    

    last_time = algorithm_time; //t_n-1
    turn_no = round(error*k_p);//P controller = SetPoint - actual yaw      /////////need to check the ratio of turns to actuated distance
    turn_reset += -turn_no; // find required reset turn 

    rotate_motor(turn_no); //////////it is now a single function, check to see how to write the encoder data

    get_data(); //mainly for gyro
    delta_time = micros()- last_time; //t_n - t_n-1
    yaw += w.gyro.z * delta_time; //intergral ///yaw angle is accumulated  ////////change to actual axis + filtering + axis offset calibration 
    
    //find yaw := remainder(mod 360)
    yaw = std::fmod(yaw, 360.0);
    yaw = principal_angle(yaw);

      
  }
  // reset control line length
  rotate_motor(turn_reset);
  
  digitalWrite(EN, LOW); //disable motor when glide
  digitalWrite(A1_3, LOW);
  digitalWrite(A2_4, LOW);
  //wait for some time ///////////// add//////////////

  //glide
  last_glide = 0.0;//time now ////////use unix time or Micros()?

}

bool high_G(){
  #ifdef REAL
    Adafruit_MPU6050_Accelerometer::getSensor(sensor_t &a);
    if ((a.acceleration.x ^2 + a.acceleration.y ^2 + a.acceleration.z ^2 ) >= 25){
      return 1;
    } else {
      return 0;
    }
  #endif

  #ifdef DEBUG_SERIAL_INPUT
    Serialif(Serial.available() > 0){
    status = Serial.read() - '0'; 
    Serial.print("status:");
    Serial.println(status);
    }

    if (status >= 1){
      return 1;
      
    } else {
      status = 0;
      return 0;
    }
  #endif
  
  #if(!defined DEBUG_SERIAL_INPUT && !defined REAL)
    return 0;
  #endif
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

  while(launched && !deployed){
    get_data();
    save_data();
    //add rasperry pi camera start
    //digitalWrite(pi_pin, HIGH);
  }
  ///////////////change pin/////////////////
  }
  
}

void loop() {
  get_data();

  if ((current_time - last_glide) > glide_interval && alt_bmp < alt_cutoff){
    algorithm();
  }

  save_input();
  
}
