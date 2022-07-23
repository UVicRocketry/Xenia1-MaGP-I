#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

//motor
#define EN 5 //replace pin number
#define A1_3 3 //L293d channel 1A 3A 
#define A2_4 4 //
int motor_position = -999; // to be zero by the the hall effect sensor  
Encoder motor(31, 32); 

const double turn_per_length = 5000.0/110.83;



// Rotate the motor so the absolute extension or retraction of the
// the shroud line is length
void rotate_motor(float length){
  
  // pos is the absolute position in terms of the encoder
  int pos = round(turn_per_length * length);
  int error = pos - motor.read();
  int init_error = error;

  Serial.print("motor.read() "); Serial.println(motor.read());

  // Turn on motor controller
  digitalWrite(EN, HIGH);

  // P controller routine to turn motor to correct position without
  // too much overshoot.
  
  // Minimum pwm (power) the motor will be run at (to prevent stalling)
  int min_pwm = 204; 

  // Max allowable error in encoder before controller finishes
  int max_error = 50;

  // Timeout in ms before the controller automatically exits
  // This is to prevent inf loop due to oscillation or similar senarios.
  long timeout = 3000;
  long time_started = millis();

  while(abs(error) > max_error){
    error = pos - motor.read();
    
    
    int pwm = min_pwm + (255-min_pwm)*abs(error/pos);
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
    Serial.print("motor.read()before turn: : "); Serial.println(motor.read());
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
