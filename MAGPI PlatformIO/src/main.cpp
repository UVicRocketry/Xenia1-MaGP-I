/* Info
#####  Sensors/Input  #####
Data/Signal -> Sensor <Library> [protocol]

Numerical Data:
- GPS -> neo GPS <TinyGPS++> [Serial Port8]
- Accelration, Gyro -> MPU6050 <Adafruit_sensor.h> [I2C]
- Alt, Pressure, Temp -> BMP390 <Adafurit> [I2C]

Logic Signal:
- Gear Motor Position/Step -> Encoder <Encoder.h> [Interrupt/ISR]

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

/* Time Routine
the data file will look like (subject to change)

GPS time, time since_last_GPS
### GPS received ###
10:00:00, 0, data1, data2, .....
10:00:00, 10, data1, data2, .....
*/
#include <Arduino.h>
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
#include "cmath"
#include "SimpleKalmanFilter.h"

#include <cstdlib>

//states
bool launched = false;
bool deployed = false;
bool test_status = false;
float max_alt = 0.0;
#define PI_PIN 1


//variables used for GPS
double lat ,lon, lat_old, lon_old, speed, gps_alt, heading;
double elasped_time;
static const double TARGET_LAT = 46.085087, TARGET_LON = -80.401635; ////////////change GPS Location
unsigned long current_time, gps_time;
bool gps_updated = false;

double filtered_alt;

//motor
#define EN 5 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //L293d channel 2A 4A 
Encoder motor(31, 32); // Set up Encoder object and declare two interrupt pins used
 
//algorithm
double target_heading = 0.0;
double last_glide = 0.0;
const double glide_interval = 7.0; //5 sec glide time ////////check if I used micros or mills
double yaw = 0.0;
const double yaw_control_cutoff = 5.0; //5degree cut off (for the error)
unsigned int void_timestamp = 0;
const double turn_per_length = 4951.0/80.55; //turns/mm
const unsigned int control_timeout = 2000; // max 2 sec turn control ////////check if I used micros or mills
const double k_p = 50.0/180.0; // max length/180 degree 
double error = 0.0;
unsigned int delta_time = 0;
unsigned int last_time = 0;
const float alt_cutoff = 300.0; ////////change
const double rad_to_degree = 180.0/PI;
double cord_length = 0.0;
#define max_cord_lenth 50.0

SimpleKalmanFilter kalmanFilter(2,2,0.01); ///////////change constants   //SimpleKalmanFilter(e_mea, e_est, q); 

#define filename "MAGPI.txt"
#define delim ","

#define SEALEVELPRESSURE_HPA (1013.25)
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
//#define ASSY_REHERSAL
//#define DEBUG_SERIAL_INPUT

#ifdef DEBUG_SERIAL_INPUT
void serial_debug(String str){
  while(Serial.available() > 0){
    Serial.read();
  }
  Serial.print(str);
  while(Serial.available()==0){
  }
}
#endif

//Set-up for the sensors to be run in setup() code 
void set_SD(){
  Wire.begin();
  Serial.print("Initializing SD card...");

  //Write the heading of the txt file
  myFile = SD.open(filename, FILE_WRITE);
  myFile.println("time,ax,ay,az,wx,wy,wz,temp,pressure,alt,lat, lon, gps_alt, heading, speed");
  myFile.close();
  
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed for the SD CARD!");

  } else{
    Serial.println("SD Card OK");
  }
}

void set_bmp(){
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("BMP not found");
  } else {
    Serial.println("BMP OK");
  }
  //filtering unecessary data
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); //high res 19 bit/0.0006C
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X); //high res 19 bit/0.33pa
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ); 
}

void set_mpu(){
  // Initialize
  if (!mpu.begin()) {
    Serial.println("MPU not found");
  } else {
    Serial.println("MPU OK");
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //set accleration range to 16G
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG); //set gyro range to 1000 deg/s
  //set rate

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

  //#ifdef DEBUG
  if(Serial8){
    Serial.print("GPS succesfully initialized");
  } else{
    Serial.print("Serial8 not open");
  }
  //#endif
}

void get_data(){
  #ifdef REAL
  
  bool reset = false;
  gps_updated = false;
  unsigned long timestamp_GPS_package = 0;
  
  elapsedMicros since_last_GPS; //must be declared outside loop for access
  while(Serial8.available()>0){ // check for gps data
    if(!reset){
      timestamp_GPS_package = since_last_GPS;//time stamp for first GPS string package is recieved
      reset = true;
    }
    myGPS.encode(Serial8.read());// encode 
  }

  if (myGPS.location.isUpdated()){////////////need check, should perform .isUpdated() for all data?
    lat = myGPS.location.lat();
    lon = myGPS.location.lng();
    current_time = myGPS.time.value(); //Raw HHMMSSCC (u32)
    gps_alt = myGPS.altitude.meters();
    heading = myGPS.course.deg();
    speed = myGPS.speed.mps();
    gps_updated = true;

  } else{
    current_time = since_last_GPS - timestamp_GPS_package; //////////need check//////////// should use gps time + micros OR just treat it as another variable
  }

  mpu.getEvent(&a, &w, &temp);
  bmp.performReading(); //write to public attributes temperature and pressure
  alt_bmp = bmp.readAltitude(SEALEVELPRESSURE_HPA); 
  #endif

  #ifdef DEBUG_SERIAL_INPUT
    serial_debug("lat: ");
    lat = double(Serial.parseFloat());
    serial_debug("lon: ");
    lon = double(Serial.parseFloat());
    serial_debug("GPS Time (HHMMSSCC): ");
    current_time = Serial.parseInt(); //UNIX time
    //gps_alt = myGPS.altitude.meters();
    serial_debug("heading: ");
    heading = myGPS.course.deg();

    //w.gyro.x = 0.0;
    mpu.getEvent(&a, &w, &temp);
    bmp.performReading(); //write to public attributes temperature and pressure
    alt_bmp = bmp.readAltitude(SEALEVELPRESSURE_HPA); 

    //speed = myGPS.speed.mps();
  #endif
}

void save_data(){
  //Use Serial for DEBUG [add pause function]
  #ifdef REAL
  myFile = SD.open(filename, FILE_WRITE);
  myFile.println("*** N ***"); myFile.print(current_time,8); myFile.print(delim);

  //mpu data sensors readings
  myFile.print("a");
  myFile.print(a.acceleration.x,8); myFile.print(delim);
  myFile.print(a.acceleration.y,8); myFile.print(delim);
  myFile.print(a.acceleration.z,8); myFile.print(delim);
  myFile.print("w");
  myFile.print(w.gyro.x,8); myFile.print(delim);
  myFile.print(w.gyro.y,8); myFile.print(delim);
  myFile.print(w.gyro.z,8); myFile.print(delim);

  //bmp
  myFile.print("bmp");
  myFile.print(bmp.temperature,8); myFile.print(delim);
  myFile.print(bmp.pressure,8); myFile.print(delim);
  myFile.print(alt_bmp,8); myFile.print(delim);

  //GPS
  myFile.print("GPS");
  myFile.print(lat,8); myFile.print(delim);
  myFile.print(lon,8); myFile.print(delim);
  myFile.print(gps_alt,8); myFile.print(delim);
  myFile.print(heading,8); myFile.print(delim);
  myFile.println(speed,8);  

  myFile.close(); 
  #endif

  #ifdef DEBUG_SERIAL_INPUT
  //Use Serial for DEBUG [add pause function]
  Serial.print(current_time);
  Serial.print(delim);

  //mpu data sensors readings
  Serial.print(a.acceleration.x); Serial.print(delim);
  Serial.print(a.acceleration.y); Serial.print(delim);
  Serial.print(a.acceleration.z); Serial.print(delim);
  Serial.print(w.gyro.x); Serial.print(delim);
  Serial.print(w.gyro.y); Serial.print(delim);
  Serial.print(w.gyro.z); Serial.print(delim);

  //bmp
  Serial.print(bmp.temperature); Serial.print(delim);
  Serial.print(bmp.pressure); Serial.print(delim);
  Serial.print(alt_bmp); Serial.print(delim);

  Serial.print(lat); Serial.print(delim);
  Serial.print(lon); Serial.println(""); 
 
  #endif
  
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

// Rotate the motor so the absolute extension or retraction of the
// the shroud line is length
void rotate_motor(float length){
  myFile.println("***Rotate Motor***"); myFile.print("L"); myFile.print(length); myFile.print(delim);
  
  if(length > max_cord_lenth){
    length = max_cord_lenth;
  } else if (length < -max_cord_lenth){
    length = -max_cord_lenth;
  }

  length = -length;
  // pos is the absolute position in terms of the encoder
  int pos = round(turn_per_length * length);
  //int pos = length;
  int error = pos - motor.read();
  int init_error = error;
  myFile.print("Mpi"); myFile.print(motor.read()); myFile.print(delim); myFile.print("e_");

  //Serial.print("E_i"); Serial.println(motor.read());

  // Turn on motor controller
  digitalWrite(EN, HIGH);

  // P controller routine to turn motor to correct position without
  // too much overshoot.
  
  // Minimum pwm (power) the motor will be run at (to prevent stalling)
  int min_pwm = 180; 

  // Max allowable error in encoder before controller finishes
  int max_error = 200;

  // Timeout in ms before the controller automatically exits
  // This is to prevent inf loop due to oscillation or similar senarios.
  long timeout = 3000;
  long time_started = millis();

  while(abs(error) > max_error){
    error = pos - motor.read();
    
    int pwm = min_pwm + (255-min_pwm)*abs(error/init_error);
    if (pwm >255){
      pwm = 255;
      }

    if (millis()%200 == 0){ // pint per 200ms [data recovery: per reading interval = 0.2 sec]
      myFile.print(error);
      myFile.print(",");      
    }  

    if (error > 0)
      analogWrite(A1_3, pwm);
    else  
      analogWrite(A2_4, pwm);

    if((millis() - time_started) > timeout){
      myFile.print("timeout"); myFile.println("");
      break;
    }
  }
    digitalWrite(EN, LOW); //High impendence mode
    analogWrite(A1_3, 0);
    analogWrite(A2_4, 0);
    myFile.println("");
}

/*algorithm()
this function is for controlling the spin of magpi by controlling the control cord line
control flow as written follow:
  - get current course heading with TinyGPS++ object .courseTO()
  - calculate the error between current heading and ideal heading
      get principal angle (between -180 and 180 degree)
  - P Controller with error = Set point yaw - actual yaw
  - If actual yaw is in cutoff range, stop controlling
  - Reset motor position to 0 

*/
void algorithm(){ 
  myFile = SD.open(filename, FILE_WRITE);
  myFile.println("***C Event***");
  //int turn_no = 0;
  int turn_reset = 0;
  double yaw_req = 0.0;

  #if (defined REAL && !defined ASSY_REHERSAL)
  target_heading = myGPS.courseTo(lat, lon, TARGET_LAT, TARGET_LON);
  yaw_req = target_heading - heading;
  yaw_req = principal_angle(yaw_req);
  #endif

  #ifdef ASSY_REHERSAL
  yaw_req = principal_angle(rand());
  #endif
  myFile.print("Ci"); //Control Even initial
  myFile.print(current_time); myFile.print(",th");
  myFile.println(target_heading);
  
// get yaw difference -> P controller
// P controller needs -> current yaw               /////need to set the map of the pulley////////// 

  yaw = 0.0;

  elapsedMicros algorithm_time; //////////change back to elasped micros   //reset algorithm_time to find time passed and for integration

  myFile.println("*** P Event***"); //P control 

  int i = 0;
  while (!abs((error = yaw_req - yaw)<= yaw_control_cutoff) && (algorithm_time < control_timeout)) {
    last_time = algorithm_time; //t_n-1
    cord_length = error*k_p;//P controller = SetPoint - actual yaw
    
    rotate_motor(cord_length); //////////it is now a single function, check to see how to write the encoder data

    mpu.getGyroSensor()->getEvent(&w);

    delta_time = (algorithm_time - last_time)* 1e-6; //t_n - t_n-1
    yaw += kalmanFilter.updateEstimate(float(w.gyro.x))* rad_to_degree * delta_time ; //intergral ///yaw angle is accumulated 
    
    yaw = std::fmod(yaw, 360.0); //find yaw := remainder(mod 360)
    yaw = principal_angle(yaw);

    myFile.print("P"); myFile.print(i); myFile.print(delim); //e.g. P0, last_time, error, cordlength, delta_time, wx, yaw (per line)
    myFile.print((last_time * 1e-6), 8); myFile.print(delim);
    myFile.print(error, 8); myFile.print(delim);
    myFile.print(cord_length); myFile.print(delim);
    myFile.print(delta_time ,8); myFile.print(delim);
    myFile.print(w.gyro.x ,8); myFile.print(delim);
    myFile.println(yaw, 8);

    i++;
      
  }
  // reset control line length
  myFile.println("Pf");
  rotate_motor(0); //rest motor position to 0
  rotate_motor(0); //call twice to ensure 0

  myFile.close();
}

bool high_G(){

  #if (defined REAL && !defined ASSY_REHERSAL) 
    mpu.getAccelerometerSensor()->getEvent(&a); //get only accleraiontion
    return (a.acceleration.x *a.acceleration.x + a.acceleration.y *a.acceleration.y + a.acceleration.z *a.acceleration.z) >= 850.0;//Check if total accelration > 2g ((2*9.81)^2 ~ 400m/s^2)
  #endif

  #ifdef DEBUG_SERIAL_INPUT
    int status = 0;
    if(Serial.available() > 0){
    status = Serial.parseInt(); 
    Serial.print("status:");
    Serial.println(status);
    }

    mpu.getAccelerometerSensor()->getEvent(&a); //get only accleraiontion
    Serial.print(a.acceleration.x *a.acceleration.x + a.acceleration.y *a.acceleration.y + a.acceleration.z *a.acceleration.z); 
    return status > 3.0 ;
  #endif
  
  #if (defined ASSY_REHERSAL && !defined ASSY_REHERSAL)
    return 1;
  #endif

}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Set up");

  pinMode(PI_PIN, OUTPUT); //setting the I/O connected to PI be OUTPUT (pullup resistor should be setup on PI software)

  //pin setup for controlling motor
  pinMode(EN, OUTPUT);
  pinMode(A1_3, OUTPUT);
  pinMode(A2_4, OUTPUT);
  digitalWrite(EN, LOW); //High impendence mode
  digitalWrite(A1_3, LOW);
  digitalWrite(A2_4, LOW);

  motor.write(0); //Zero Encoder 
  set_SD();
  set_bmp();
  set_mpu();
  set_GPS();

/*
  get_data();
  save_data();
*/
  //add one set the motor to high impendace mode to prevent spinning
  //add set sensor to sleep OR not active
  
  #ifdef ASSY_REHERSAL
    launched = true;
  #endif

  while(!launched){ //while rocket is not launch, stay in the loop
    void_timestamp = millis();
    while(high_G() && !launched){  
      launched = ((millis()- void_timestamp) > 500)? true : false; // if high G for 0.5s, set state as launch
    }
  }
  myFile = SD.open(filename, FILE_WRITE);
  myFile.print("launched at"); myFile.println(millis());
  myFile.close();
  
  #ifdef ASSY_REHERSAL
    deployed = true;
  #endif
  void_timestamp = millis();
  while(launched && !deployed){ // while rocket is launched but magpi is not deployed
    if ((millis()-void_timestamp)>60000){
        break;
    }
    get_data();
    
    if(alt_bmp > max_alt){
      max_alt = alt_bmp;

    } else if((max_alt - alt_bmp) > 100.0){ //if descent from apogee more than 100m, set deployed true
      deployed = true;

    }
    save_data();
    
  }
  myFile = SD.open(filename, FILE_WRITE);
  myFile.print("DEPLOYED AT"); myFile.println(millis());
  myFile.close(); 

  digitalWrite(PI_PIN, HIGH);  //output HIGH logic to the PI_PIN
  #ifdef DEBUG_SERIAL_INPUT
  get_data();
  save_data();
  #endif
  
  
}

void loop(){
  #if (defined REAL && !defined ASSY_REHERSAL)
  
  get_data();

  if (((millis() - last_glide) > glide_interval) && (alt_bmp > alt_cutoff)){ //the logic seems not working to well, check if the problem is current time
    algorithm();
    last_glide = millis();//time now
  }
  
  save_data();

  #endif

  #ifdef ASSY_REHERSAL
  get_data();
  algorithm();
  save_data();
  #endif
}
