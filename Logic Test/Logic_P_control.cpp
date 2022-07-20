#include <iostream>
//states
bool launched = 0;
bool deployed = 0;
bool test_status = 0;

//variables used for Hall effect sensor /////////////need checking///////////
volatile char half_revolutions;
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
double target_heading = 0.0;
double last_glide = 0.0;
const double delta_static = 1.0; ////////////change accordingly///////////
const double turn_ratio = 1.0; ////////////change accordingly///////////
const double glide_interval = 1.0; ////////////change accordingly///////////
double turn_interval = 1.0; ////////////change accordingly///////////
double max_cord = 50; //50mm
int dir = 0;
const int control_timeout = 7;
double yaw = 0.0;
const double yaw_control_cutoff = 0.2;
unsigned int void_timestamp = 0;
double error = 0.0;
const double k_p = 5.0/180.0; // max length/180 degree 
unsigned int delta_time = 0;
unsigned int last_time = 0;
int turn_no = 0;
double yaw_req = 0.0;
double z = 0.0;
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

#define SEALEVELPRESSURE_HPA (1013.25)
unsigned t = 0;


int micros(){
    t++;
}

int read(){
    int step;
    std::cout << "Turned step:";
    std::cin >> step;
    return step;
}

double get_data(){
    int z;
    std::cout << "yaw rate (d/1sec):";
    std::cin >> z;
    return z;
}

int main() {
    while (1)
    {
        std::cout << "yaw_req: ";
        std::cin >> yaw_req;
        yaw = 0.0;
        last_time = 0.0;
        t = 0;
        
        
        while (!(((error = yaw_req - yaw) <= yaw_control_cutoff)&&(error >= -yaw_control_cutoff)) && ((micros()- void_timestamp) < control_timeout)) {
        std::cout << "error: " << error << '\n'; 
        
        turn_no = error*turn_ratio;//P controller = SetPoint - actual yaw      /////////need to check the ratio of turns to actuated distance
        
            while(turn_no != 0){ 
                //turn motor in the direction needed
                std::cout << "turn_no: " << turn_no << '\n';
                if(turn_no > 0){  
                    std::cout << "tuning right\n";
                } else if (turn_no < 0){
                    std::cout << "tuning left\n";
                } else{
                    std::cout << "no turning";
                }

                turn_no -= read(); // to check if motion is completed(check step difference)
                std::cout << '\n';
            }

            z = get_data();
            delta_time = 1.0;
            std::cout<< "delta_time: " << delta_time <<'\n';
            yaw += z * 1.0;

            if (yaw >360.0) {
            yaw += -360.0;
            } else if (yaw < 360.0) {
            yaw += 360.0;
            }
            
            std::cout << "Spined Yaw: " << yaw << '\n';
            //intergral  ////////change to actual axis + filtering + axis offset calibration 
            std::cout << '\n';


        }
        std::cout << "Exited loop\n\n";
    }
}