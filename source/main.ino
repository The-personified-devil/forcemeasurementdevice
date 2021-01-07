// Version 0.10
// Copyright © 2020 Leonhard Saam
// Licensed under GNU GPL v3

// #include "../external/New-LiquidCrystal/LCD.h"
// #include <../external/New-LiquidCrystal/LiquidCrystal_I2C.h>
// #include <Wire.h>

#define I2C_ADDR 0x27 //Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);


/**************************************************************************
 *        Global Values
 **************************************************************************/

int topTrigger = 640;
int bottomTrigger = 280;
const int usInSec = 100;

int inputPin = A0;

enum Mode {Measurement, Tuning, Options};
int mode;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), interrupt, FALLING);
  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Kalibrierung");
  lcd.setCursor(0,1);
  lcd.print("empfohlen");
  mode = Tuning;
}

class measurementMode;
class tuningMode;
class optionsMode;

measurementMode measurementModeObj;
tuningMode tuningModeObj;
optionsMode optionsModeObj;


void loop(){
  switch (mode){
    case Measurement:
      measurementModeObj.measurement();
      break;

    case Tuning:
      tuningModeObj.tuning();
      break;

    case Options:
      optionsModeObj.options();
      break;
  }
}

class measurementMode{
  private:
    bool validMeasurement = false;

    int freqMeasurement(){
      int measurement1;
      int measurement2;
      int measurement3;

      for (int measurementIteration = 1; measurementIteration <= 3; measurementIteration++) // Create measurement points
      {
        int waveCounter;
        int startTime = micros();
        bool completeWave = true;
        int freq;

        for (int probeIteration = 0; probeIteration < 1000; probeIteration ++)
        {
          int input = analogRead(inputPin);
          if (completeWave && input >= topTrigger){
            waveCounter ++;
            completeWave = false;
          }
          else if (input <= bottomTrigger) completeWave = true; // Avoid adding incomplete waves to the wave counter
        }

        int endTime = micros(); // Calculate Frequency
        int timeDiff = startTime - endTime;
        int timepWave = timeDiff / waveCounter;
        freq  = usInSec / timepWave;

        switch (measurementIteration){ // Save calculated frequency in the correct measurement point.
          case 1:
            measurement1 = freq;
            break;

          case 2:
            measurement2 = freq;
            break;

          case 3:
            measurement3 = freq;
            break;
        }
      }


      return result;
    }

  public:
    void measurement(){
      while (!validMeasurement && mode == Measurement) freqMeasurement();
      if (mode == Measurement) return;
    }
};

class tuningMode{
  public:
    void tuning(){

    }
};

class optionsMode{
  public:
    void optionsInterrupt(){

    }
    void options(){
    
    }
};
