#include <iostream>
#include "PetitFS.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
#define BAUD_RATE= 115200;

File data;
int pinCS=29;

 /* Purpose: Initialize constructor class and built in methods
     * Parameters:none
     * Returns: none
     * To-do: complete logData
    */
class Sensor{
    private:
    string name;
      
    public: 
    
     /* Purpose: Begins serial communication between a generic sensor and the microcontroler
     * Parameters:none
     * Returns: none
    */
    void start(){ //initializes serial communication in sensors 
        Serial.begin(BAUD_RATE);
        while (!Serial){
            delay(10);
             Serial.println(F(this.name +"sensor was sucessfully initialized"))
    }

    //try once again to initialize
    if(!this.name.begin()){
        Serial.println(F("failed to initialize"+ this.name));
        return;
        }
    }

    /* Purpose: Initializes serial communication and performs specific operations for the given sensors (this 
     *  can't be initialized generically)
     * Parameters: string type- the type of sensor to be initialized
     * Returns: none
     * To-do:
    */

    void start(string type){
        if (type=="sdCard"){
            if (SD.begin()){
                Serial.println(F("Initialization was succesful"));
            }
            data=SD.open("data.csv", FILE_WRITE);
            if (data){
            data.close();
            Serial.println(F("done"));
        }   else{
            Serial.println(F("an error ocurred"));
        }

          }else if (type=="gps"){
           SoftwareSerial ss(4,3);
           Serial.begin(BAUD_RATE);
           ss.begin(BAUD_RATE);
            }
            else{
                Serial.println(F("Couldn't initialize properly"));
                return;
            }
        }

     /* Purpose: When calling this method, the data obtained is written to the SD file
     * Parameters: data- the data to be logged into the card. **ask if it is more convinient to have it as an array.
     * Returns: none
     * To-do: complete this method
    */ 

    void logData(string data){ 
      return;
    }
    
}
    void setup(){ 
    Sensor bmp390;
    bmp390.name= bmp;


    Sensor mpu5069;
    mpu5060.name= mpu;

    Sensor Neo6;
    Neo6.name= gps;

   
    }

    void loop(){


    }

    /* Purpose: Fileter unecessary data gathered to optimize quality 
     * Parameters: none
     * Returns: **maybe return instead of print??
     * To-do:
    */
    void filterBMPdata(){
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }
     
    /* Purpose: Get readings for the bmp sensors 
     * Parameters: none
     * Returns: none **maybe return instead of print??
     * To-do:
    */
    

    void getbmpReadings(string dataType){
      if (dataType=="temperature"){
          Serial.print(bmp.temperature);
          Serial.print(" *C");
          Serial.print("\t");         
    }else if (dataType=="pressure"){
          Serial.print(bmp.pressure / 100.0);
          Serial.println(" hPa");
          Serial.print("\t");
    }else{
           Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
           Serial.println(" m");
           Serial.print("\t");
    }
    delay(3000);
    }

     /* Purpose: Get readings for the mpu sensors 
     * Parameters: none
     * Returns: **maybe return instead of print??
     * To-do:
    */

    void getmpuReadings(string dataType){
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
      
      if (dataType=="acceleration"){
        Serial.println(F("Acceleration X: "));
        Serial.print(F(a.acceleration.x));
        Serial.print(F("\t"));
        Serial.print(F(", Y: "));
        Serial.print(F(a.acceleration.y));
        Serial.print(F("\t"));
        Serial.print(F(", Z: "));
        Serial.print(F(a.acceleration.z));
        Serial.println(F(" m/s^2"));
        Serial.print(F("\t"));
              
    }else{
      Serial.print(F("Rotation X: "));
      Serial.print(F(g.gyro.x));
      Serial.print(F("\t"));
      Serial.print(F(", Y: "));
      Serial.print(F(g.gyro.y));
      Serial.print(F("\t"));
      Serial.print(F(", Z: "));
      Serial.print(F(g.gyro.z));
      Serial.print(F("\t"));
      Serial.println(F(" rad/s"));
      Serial.print(F("\t"));
    }
    delay(3000);
    }
     
     /* Purpose: Gets readings for position in the sensors 
     * Parameters: type of position wanted
     * Returns: **maybe return instead of print??
     * To-do:
    */
    void getPos(string positionType){
      while (ss.available() > 0){
        gps.encode(ss.read());

      if (gps.location.isUpdated()){
        if(positionType=="Latitude"){
        Serial.print(gps.location.lat(), 6);
        }else{
        Serial.println(gps.location.lng(), 6);
        }
      }   
   }

   

    
      
    
    

    
