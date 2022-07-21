#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

//motor
#define EN 5 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //
int motor_position = -999; // to be zero by the the hall effect sensor  
Encoder motor(31, 32); 

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

void setup(){
//pin setup for controlling motor
    pinMode(EN, OUTPUT);
    pinMode(A1_3, OUTPUT);
    pinMode(A2_4, OUTPUT);
    digitalWrite(EN, LOW); //High impendence mode
    digitalWrite(A1_3, LOW);
    digitalWrite(A2_4, LOW);

    Serial.begin(9600);
    while(!Serial);
    
}

void loop(){
    Serial.print("Enter step [Left(-)/Right(+)]: ");
    while(Serial.available() == 0);
    rotate_motor(Serial.read());
    
    digitalWrite(EN, LOW); 
    digitalWrite(A1_3, LOW);
    digitalWrite(A2_4, LOW);
        

}
