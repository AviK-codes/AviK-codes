#include <Arduino.h>
#include <WiFiNINA.h> 
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>
#include "RTClib.h"
#include <Wire.h>
#include <SPI.h>
#include <I2S.h>

//#include <Adafruit_BMP280.h>
#include <BMP280_DEV.h>

float temperature,  altitude;            // Create the temperature, pressure and altitude variables
uint32_t pressure;
//BMP280_DEV bmp280(10);                            // Instantiate (create) a BMP280_DEV object and set-up for SPI operation on digital pin D10
BMP280_DEV bmp280;
  
char buff[6]={0,0,0,0,0,0};

// https://create.arduino.cc/projecthub/rijk_meurs/iot-weather-station-4c29c6
// https://create.arduino.cc/projecthub/SurtrTech/bmp280-measure-temperature-pressure-and-altitude-e1c857

//Adafruit_BMP280 bmp; // I2C Interface

void setup()
 {
  //Serial.begin(115200);
  Serial.begin(921600);
  while (!Serial);
  //bmp280.setClock(1000000);
  bmp280.begin(0x76);                                 // Default initialisation, place the BMP280 into SLEEP_MODE
  bmp280.begin(NORMAL_MODE, OVERSAMPLING_X1, OVERSAMPLING_X1, IIR_FILTER_OFF, TIME_STANDBY_05MS);
  //Serial.println("Trying to initialize  BMP280");
/*
  if (!bmp.begin(0x76))
   {
    Serial.println(F("Failed to initialize BMP280 sensor, check wiring!"));
    while (1);
   }
   //Serial.println("Succesfull to initialize BMP280");

// Default settings from datasheet.
//  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
//                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
//                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling SAMPLING_X16 */
//                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. FILTER_X16*/
//                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. STANDBY_MS_500  */

  // start I2S at 16 kHz with 32-bits per sample
  
  //Serial.println("Trying to initialize I2S!");
  //I2S_PHILIPS_MODE
  //I2S_RIGHT_JUSTIFIED_MODE
  //I2S_LEFT_JUSTIFIED_MODE
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  //else Serial.println("Succesfull to initialize I2S!");
 }

int cycle=0;

void loop()
 {
  
  // read a sample

  int sample = I2S.read();

  if ((sample == 0) || (sample == -1) )
   {
    return;
   }
  
  //Serial.write((char *)synch ,2);
  
  sample = sample*4;
  buff[0] =*(((char *) (&sample)+2));
  buff[1] =*(((char *) (&sample)+3));

  //buff[0] =*(((char *) (&sample)+1));         
  //buff[1] =*(((char *) (&sample)+2));       
  //buff[2] =*(((char *) (&sample)+3)); 

  cycle++;

  if (cycle==80)
   {
    bmp280.getCurrentPressure(pressure);
    //bmp280.startForcedConversion();                 // Start BMP280 forced conversion (if we're in SLEEP_MODE)
    //if (bmp280.getMeasurements(temperature, pressure, altitude)) // Check if the measurement is complete
    //  { 
    //int pres= bmp.readPressure();
    
    //Serial.print(pressure);    // avi
    //Serial.print(F("hPa   ")); // avi
    //pressure=pressure*10; 
    //int pres = (float)pressure/600;
    
    //Serial.print(pressure+String(",")); //displaying the Pressure in Pascals, you can change the unit
    //pressure=pressure>>8;
    //Serial.print(pressure+String(","));
    //Serial.println(millis()%1000);  

    //int pres = pressure >> 9;
    int pres = pressure & 0x0000ffff;
    buff[2] = *(((char *) (&pres)));
    buff[3] = *(((char *) (&pres)+1));  
    
    //pressure = pressure >> 1;
    //buff[3] = *(((char *) (&pressure)));
    //buff[4] = *(((char *) (&pressure)+1)); 
    //buff[5] = *(((char *) (&pressure)+2));

    cycle=0;
   }

  // }

  //buff[2] = 0xb;
  //buff[3] = 0xc;

  Serial.write (buff, 4); 
  //Serial.write (buff, 6);
  
  ////uint16_t pres= bmp.readPressure();
  ////Serial.write((char *)&pres ,2);

  //Serial.write(lowByte((int)sample));  // send low byte
  //Serial.write(highByte((int)sample)); // send high byte

  //Serial.write(int16_t(sample && 0xFF));  // send low byte
  //Serial.write(int16_t(sample >> 8));     // send high byte
  //Serial.write(sample, 2);
   
   // Serial.print(F("Temperature = "));
   // Serial.print(bmp.readTemperature());
   // Serial.println(" *C");

   //Serial.print(F("Pressure = "));
   //Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
   // Serial.println(" hPa");

   // Serial.print(F("Approx altitude = "));
   // Serial.print(bmp.readAltitude(1019.66)); //The "1019.66" is the pressure(hPa) at sea level in day in your region
   // Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude

   // Serial.println();
   // delay(2000);
  
  //Serial.print(bmp.readPressure()+String(",")); //displaying the Pressure in Pascals, you can change the unit
  //Serial.print(bmp.readTemperature()+String(","));
  //Serial.println(millis()%1000);

  //bmp280.startForcedConversion();                 // Start BMP280 forced conversion (if we're in SLEEP_MODE)
  //if (bmp280.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
  //{
    /*
    Serial.print(temperature);                    // Display the results    
    Serial.print(F("*C   "));
    Serial.print(pressure);    
    Serial.print(F("hPa   "));
    Serial.print(altitude);
    Serial.println(F("m"));
    */
    //bmp280.getCurrentPressure(pressure);
    //Serial.print(pressure+String(",")); //displaying the Pressure in Pascals, you can change the unit
    //Serial.print(temperature+String(","));
    //Serial.println(millis()%1000);  
  //}

 }
