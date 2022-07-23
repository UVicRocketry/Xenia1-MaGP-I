#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

//motor
#define EN 5 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //
int motor_position = -999; // to be zero by the the hall effect sensor  
Encoder motor(31, 32); 

const double turn_per_length = 5000.0/110.83;



void rotate_motor(float length){
  Serial.print("motor.read(): "); Serial.println(motor.read());
  int turns = round(turn_per_length * length);
  int error = 0;
  Serial.println(turns);
  digitalWrite(EN, HIGH);
  while(abs(error = turns - motor.read()) >= 50){
    if (error > 0){
      analogWrite(A1_3, abs(50 + round(255*error/turns)));
      
    }  else if (error < 0){ // e.g. pos_now = 0, set point < -10 => pos_now > set point then turn left
      analogWrite(A2_4, abs(50 + round(255*error/turns)));
    }
  }
}


void setup(){
//pin setup for controlling motor
    motor.write(0);
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
    Serial.print("motor.read(): "); Serial.println(motor.read());
    Serial.print("Length (+/-) : " );
    while(Serial.available() == 0){
      
    }
    
    Serial.println("Turning...");
    rotate_motor(Serial.parseFloat());
    
    digitalWrite(EN, LOW); //High impendence mode
    digitalWrite(A1_3, LOW);
    digitalWrite(A2_4, LOW); 
    
    while(Serial.available() > 0){
      Serial.read();
      }
     


    Serial.print("Done. Num of turns:");
    Serial.println(motor.read());



}
