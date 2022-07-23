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

/*Gordon to do
- Start up: Use serial port???
- complete logic then add time_out routines [DONE]
- Algorthim data writing: Encoder, P controller output, every step
- check sensor sampling rate
- set sensor range

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

//states
bool launched = 0;
bool deployed = 0;
bool test_status = 0;

//variables used for Hall effect sensor /////////////need checking///////////
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

//variables used for GPS
double lat ,lon, lat_old, lon_old, speed, gps_alt, heading;
double elasped_time;
static const double TARGET_LAT = 12.123456, TARGET_LON = 12.123456; ////////////change GPS Const
unsigned long current_time, gps_time;

double filtered_alt;

//motor
#define EN 5 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //
int motor_position = -999; // to be zero by the the hall effect sensor  
Encoder motor(31, 32); 

//algorithm
double target_heading = 0.0;
double last_glide = 0.0;
const double glide_interval = 5.0; //5 sec glide time ////////check if I used micros or mills
/////////double max_cord = 30; //30mm ,note that the orginal length is 4.826cm 
int dir = 0;
const double turn_per_length = 5000.0/110.83; //turns/mm
const unsigned int control_timeout = 2000; // max 2 sec turn control ////////check if I used micros or mills
double yaw = 0.0;
const double yaw_control_cutoff = 5.0; //5degree cut off (for the error)
unsigned int void_timestamp = 0;
double error = 0.0;
const double k_p = 30.0/180.0; // max length/180 degree 
unsigned int delta_time = 0;
unsigned int last_time = 0;
const float alt_cutoff = 1.0; ////////change
const double rad_to_degree = 180.0/PI;
double cord_length = 0.0;

SimpleKalmanFilter kalmanFilter(1,1,0.01); ///////////change constants   //SimpleKalmanFilter(e_mea, e_est, q); 

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

//#define REAL
#define DEBUG_SERIAL_INPUT

#ifdef DEBUG_SERIAL_INPUT
void serial_debug(String str){
  Serial.print(str);
  while(Serial.available()==0){
  }
}
#endif

//Set-up for the sensors to be run in setup() code 
void set_SD(){
  Wire.begin();
  Serial.print("Initializing SD card...");
  
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

  #ifdef DEBUG
  if(Serial8){
    Serial.print("GPS succesfully initialized");
  } else{
    Serial.print("Serial8 not open");
  }
  #endif
}

void get_data(){
  #ifdef REAL
  
  bool reset = 0;
  unsigned long timestamp_GPS_package = 0;
  
  elapsedMicros since_last_GPS; //must be declared outside loop for access
  while(Serial8.available()>0){ // check for gps data
    if(!reset){
      timestamp_GPS_package = since_last_GPS;//time stamp for first GPS string package is recieved
      reset = 1;
    }
    myGPS.encode(Serial8.read());// encode 
  }

  if (myGPS.location.isUpdated()){////////////need check, should perform .isUpdated() for all data?
    lat = myGPS.location.lat();
    lon = myGPS.location.lng();
    gps_time = myGPS.time.value(); //Raw HHMMSSCC (u32)
    gps_alt = myGPS.altitude.meters();
    heading = myGPS.course.deg();
    speed = myGPS.speed.mps();

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

    //speed = myGPS.speed.mps();
  #endif
}

void save_data(){
  //Use Serial for DEBUG [add pause function]
  myFile = SD.open(filename, FILE_WRITE);
  myFile.print(current_time);
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

void rotate_motor(double length){
  int pos = round(turn_per_length * length);
  digitalWrite(EN, HIGH);
  
  if (pos > 0){

    digitalWrite(A1_3, HIGH);
    while(motor.read() < pos);

  } else if (pos < 0){ // e.g. pos_now = 0, set point < -10 => pos_now > set point then turn left
    
    digitalWrite(A2_4, HIGH);
    while(motor.read() > pos);

  }
  digitalWrite(EN, LOW);
  digitalWrite(A1_3, LOW);
  digitalWrite(A2_4, LOW);
  
}

void algorithm(){

  //int turn_no = 0;
  int turn_reset = 0;
  double yaw_req = 0.0;
  target_heading = myGPS.courseTo(lat, lon, TARGET_LAT, TARGET_LON);
  yaw_req = (target_heading - heading);
  yaw_req = principal_angle(yaw_req);
  
// get yaw difference -> P controller
// P controller needs -> current yaw               /////need to set the map of the pulley////////// 

  yaw = 0.0;
  elapsedMicros algorithm_time; //////////change back to elasped micros   //reset algorithm_time to find time passed and for integration

  while (!(((error = yaw_req - yaw)<= yaw_control_cutoff)&&(error >= -yaw_control_cutoff)) && (algorithm_time < control_timeout)) {
    
    last_time = algorithm_time; //t_n-1
    cord_length = error*k_p;//P controller = SetPoint - actual yaw      /////////need to check the ratio of turns to actuated distance
    
    rotate_motor(cord_length); //////////it is now a single function, check to see how to write the encoder data

    mpu.getGyroSensor()->getEvent(&w);

    delta_time = algorithm_time - last_time; //t_n - t_n-1
    yaw += kalmanFilter.updateEstimate(float(w.gyro.x))* rad_to_degree * delta_time; //intergral ///yaw angle is accumulated 
    
    yaw = std::fmod(yaw, 360.0); //find yaw := remainder(mod 360)
    yaw = principal_angle(yaw);

      
  }
  // reset control line length
  rotate_motor(0); //rest motor position to 0
  
  digitalWrite(EN, LOW); //disable motor when glide
  digitalWrite(A1_3, LOW);
  digitalWrite(A2_4, LOW);


  //glide
  last_glide = 0.0;//time now ////////use unix time or Micros()?

}

bool high_G(){
  #ifdef REAL
    mpu.getAccelerometerSensor()->getEvent(&a); //get only accleraiontion
    if ((a.acceleration.x *a.acceleration.x + a.acceleration.y *a.acceleration.y + a.acceleration.z *a.acceleration.z) >= 25.0){
      return 1;
    } else {
      return 0;
    }
  #endif

  #ifdef DEBUG_SERIAL_INPUT
    int status = 0;
    if(Serial.available() > 0){
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

  motor.write(0);
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
  Serial.print("launched");
  /*
  while(launched && !deployed){
    get_data();
    save_data();
    //add rasperry pi camera start
    //digitalWrite(pi_pin, HIGH);  ///////////////change pin/////////////////
  }
  */

  }
  
  
}

void loop(){
  #ifdef REAL
  get_data();

  if ((current_time - last_glide) > glide_interval && alt_bmp < alt_cutoff){
    algorithm(); ///////algorithm should have it's own get_data and sve_data (or just call them)
  }
  
  save_data();
  #endif

  #ifdef REAL
  get_data();
  save_data();
  #endif
}
