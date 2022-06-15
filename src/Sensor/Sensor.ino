#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>

#include <pff.h>
#include <diskio.h>
#include <pffconf.h>
#include <pffArduino.h>
#include <PF.h>
#include <integer.h>
#include <PetitSerial.h>

#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
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
#define Hall_Sensor A0         
#define Hall_Sensor_D2
int Measure1=0,Measure2=0;             

File data;
int pinCS=29;
//Sensor objects
Adafruit_BMP3XX bmp;
Adafruit_MPU6050 mpu;
TinyGPS gps;
SoftwareSerial ss(4, 3);

void setup(){ 
    //Start bmp sensor serial communication
     Serial.begin(115200);
      while(!Serial){
        delay(10);
        //Serial.println(F("sensor was sucessfully initialized")); 
    }
      if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
      //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
      //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    return;
  }
  // start serial communication with mpu 
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return;
      delay(10);
    }
      // start communication with the gps    
     Serial.begin(115200);
     ss.begin(4800); 
     //start communication with the Hall effect sensor
     Serial.begin(115200);
     pinMode(Hall_Sensor, INPUT);
  }
  
void loop(){
    //it calls all of the sensors in a loop to avoid redundancy
  logToSD();
  

}

    /* Purpose: Get readings for the bmp sensors and stores them as a string
     * Parameters: none
     * Returns: string- data gathered from the sensor at given baud rate.
     * Precondition: Method should be run in loop function
     * To-do: check output and improve appearance 
    */
    
    String getbmpReadings(){ 
       if (! bmp.performReading()) {
    Serial.println("Failed to perform reading");
    return;
    }
    
        String bmpData="";
        bmpData+= ((String(bmp.temperature))+" *C "+ " \t" );   //append temperature
        bmpData+= ((String(bmp.pressure/100.0))+" hPa "+"\t" );
        bmpData+=String(bmp.readAltitude(SEALEVELPRESSURE_HPA), " m");

        delay(3000);
        return bmpData;
    }

      /* Purpose: Get readings for the mpu sensors 
     * Parameters: none
     * Returns: string[] 
     * Precondition: Method should be run in loop function
     * To-do: figure out header for returning array
     * 
    */
       String getmpuReadings(){
        sensors_event_t a, g, temp;
        String mpuReadings[2]= {"Acceleration: ", "Rotation "}; //creates an array 2 strings, one for acceleration readings and the other rotation
    
        mpu.getEvent(&a, &g, &temp);

        //add readings to the first entry in the array (for acceleration)
        mpuReadings[0]+=(" X: "+ String(a.acceleration.x)+ "\t" );
        mpuReadings[0]+=(" Y: "+String(a.acceleration.y)+ "\t" );
        mpuReadings[0]+=(" Z: "+ String(a.acceleration.z)+ "\t");

        //add readings to second entry
        mpuReadings[1]+= (" X: "+String(g.gyro.x) +"\t" );
        mpuReadings[1]+= (" Y: "+ String(g.gyro.y) +"\t" );
        mpuReadings[1]+=(" Z: "+ String(g.gyro.z)+"\t" );

        return mpuReadings[2];
        delay(3000);
    }

    /* Purpose: Get latitude and longitude according to baud rate with gps
     * Parameters: none
     * Precondition: Method should be run in loop function
     * Returns: str[] with latitude in position 0 and longitude in position 1
     * To-do: check how to return string array
    */

       String getGpsReadings(){
        String positions[2]= {"Latitude: ", "Longitude "};
        while(ss.available()>0){
            gps.encode(ss.read());
                 float flat, flon;
                 unsigned long age;
                  gps.f_get_position(&flat, &flon, &age);
                  positions[0]+=String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
                  positions[1]+=(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

            
        }
        return positions[2];
    }

    String getHallEffect(){
    /* Purpose: Get hall effect readings according to general baud rate and use it for decision making
     * Parameters: none
     * Precondition: Method should be run in loop function
     * Returns: string with both measurements
     * To-do: 
    */
        String measurements= " ";
        Measure1=analogRead(Hall_Sensor);    
        measurements+=(Measure1 + " \t");
        Measure2=digitalRead(Hall_Sensor);
        measurements+=(Measure2+ " \t");
        return measurements;
    }

    
    /* Purpose: Log data from the analog sensors to a file 
     * Parameters: none
     * Precondition: Method should be run in loop function
     * Returns: nothing 
     * To-do: 
    */
     void logToSD(){
         //call methods to log data 
         //File object and opens file to write
          File sensorData= SD.open("data.csv", FILE_WRITE);

            if(sensorData){
                 String bmpData= getbmpReadings();
                 String mpuData[2]= getmpuReadings();
                 String locationData[2]=getGpsReadings();
                 sensorData.println(bmpData);
                 sensorData.println(mpuData[2]);
                 sensorData.println(locationData[2]);
                sensorData.close();
                Serial.println("data was succesfully logged to SD card");
            } else{
                Serial.println("error with the SD card");
            }
            delay(100);
        }
