#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

//motor
#define EN 39 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //
//int motor_position = -999; // to be zero by the the hall effect sensor  
Encoder motor(28, 29); 

const double turn_per_length = 4951/80.55;



// Rotate the motor so the absolute extension or retraction of the
// the shroud line is length
void rotate_motor(float length){
  
  // pos is the absolute position in terms of the encoder
  int pos = round(turn_per_length * length);
  //int pos = length;
  int error = pos - motor.read();
  int init_error = error;

  Serial.print("motor.read() Before "); Serial.println(motor.read());

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

    if (millis()%100 == 0){
      Serial.print(error);
      Serial.print(",");
      Serial.println(pwm);      
    }  

    if (error > 0)
      analogWrite(A1_3, pwm);
    else  
      analogWrite(A2_4, pwm);

    if((millis() - time_started) > timeout){
      Serial.println("Controller did not converge.");
      break;
    }
  }
    digitalWrite(EN, LOW); //High impendence mode
    analogWrite(A1_3, 0);
    analogWrite(A2_4, 0); 
}


void setup(){
//pin setup for controlling motor
    motor.write(0);
    pinMode(EN, OUTPUT);
    pinMode(A1_3, OUTPUT);
    pinMode(A2_4, OUTPUT);
    digitalWrite(EN, LOW); //High impendence mode
    analogWrite(A1_3, 0);
    analogWrite(A2_4, 0); 

    Serial.begin(9600);
    while(!Serial);
    
    
    

}

void loop(){
    Serial.print("Length (+/-) : " );
    while(Serial.available() == 0){
      
    }
    
    Serial.println("Turning...");
    rotate_motor(Serial.parseFloat());
    
    digitalWrite(EN, LOW); //High impendence mode
    analogWrite(A1_3, 0);
    analogWrite(A2_4, 0); 
    
    while(Serial.available() > 0){
      Serial.read();
      }
     


    Serial.print("motor.read() after:");
    Serial.println(motor.read());



}
