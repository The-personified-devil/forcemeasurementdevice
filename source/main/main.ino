// Version 0.10
// Copyright © 2020 Leonhard Saam
// Licensed under GNU GPL v3

#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <MemoryFree.h>

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

int topTrigger = 560;
int bottomTrigger = 440;
const long usInSec = 1000000;

const int measurementIterationGoal = 6;
const int mmPointIterationGoal = 3;
const int probeIterationGoal = 1000;

const int deviationAllowance = 20;
const int varianceAllowance = 10;

int inputPin = A0;

enum Mode {Measurement, Tuning, Options};
int mode;

void setup() {
  Serial.begin(9600);
//   attachInterrupt(digitalPinToInterrupt(2), interrupt, FALLING);
  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Kalibrierung");
  lcd.setCursor(0,1);
  lcd.print("empfohlen");
  mode = Measurement;
}

class measurementMode{
  private:
    bool validMeasurement = false;
    int measurementPoints[mmPointIterationGoal];
    int measurements[measurementIterationGoal];

    void mmPointCreator() {
      for (int mmPointIteration = 0; mmPointIteration < mmPointIterationGoal; mmPointIteration++) // Create measurement points
        {
          int waveCounter = 0;
          long startTime = micros();
          bool completeWave = true;

          for (int probeIteration = 0; probeIteration < probeIterationGoal; probeIteration ++)
          {
            int input = analogRead(inputPin);
            if (completeWave && input >= topTrigger) {
              waveCounter ++;
              completeWave = false;
            }
            else if (input <= bottomTrigger) completeWave = true; // Avoid adding incomplete waves to the wave counter
          }

          long endTime = micros(); // Calculate Frequency
          long timeDiff = endTime - startTime;
          long timepWave = timeDiff / waveCounter;
          int freq  = usInSec / timepWave;
          bool validResult; // TODO: Assess if there was no clear overariching average
          measurementPoints[mmPointIteration] = freq;
          Serial.print("Measurement point creator: ");
          Serial.println(freq);
          Serial.println ("What the fuck");
          // Serial.println(waveCounter);
          // Serial.println(timeDiff);
          // Serial.println(timepWave);
          // Serial.println(mmPointIteration);
          // Serial.println("LOL");
          // group frequencies and get modal of frequencies
          // get number of neighbours for number and get modal from number of neighbours
        }
    }

    int mmCreator() {
      int measurementCounter = 0;

      while (measurementCounter < measurementIterationGoal) {
        mmPointCreator();

        bool validRun = true;

        int maxValue = measurementPoints[0];
        int minValue = measurementPoints[0];
        Serial.println ("hello");
        for (int i = 0; i < mmPointIterationGoal; i++){
          int currentVal = measurementPoints[i];
          if (currentVal > maxValue) maxValue = currentVal;
          if (currentVal < minValue) minValue = currentVal;
        }

        int necessaryVal = maxValue - minValue;
        Serial.println ("before catastrophe");
        Serial.print ("Necessary value: ");
        Serial.println (necessaryVal);
        Serial.print ("Free RAM: ");
        Serial.println(freeMemory());
        delay (1000);
        
        if (necessaryVal > 0){

        
        byte neighbourCounter [necessaryVal];
        Serial.println ("after catastrophe");
        Serial.print ("Necessary value: ");
        Serial.println (necessaryVal);
        for (int i = 0; i < necessaryVal; i++){
          neighbourCounter[i] = 0;
          // Serial.print("Neighbour counter: ");
          // Serial.println(neighbourCounter[i]);
          // delay(100);
        }

        // O leads to undefined behavior because I was too lazy
        for (int i = 0; i < mmPointIterationGoal; i++){
          int currentVal = measurementPoints[i];
          int posValue = currentVal-minValue;
          for (int i = 0; i < varianceAllowance; i++){
            neighbourCounter[posValue + i] ++;
            neighbourCounter[posValue - i - 1] ++;
          }
        }

        Serial.print ("Necessary value: ");
        Serial.println (necessaryVal);

        for (int i = 0; i < necessaryVal; i++){
          Serial.print ("Value: ");
          Serial.println (neighbourCounter[i]);
          // Serial.print("FMemory: ");
          // Serial.println(freeMemory());
          // delay (100);
        }
        Serial.println("HelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmeHelpmev");
        delay(1000);
        }
        int modeOfMms = 0;  // TODO: Calculate the mode average of the measurements
        for (uint8_t i = 0; i < mmPointIterationGoal; i++) {
          modeOfMms = (measurementPoints[i] > measurementPoints[modeOfMms]) ? i : modeOfMms;
        }

        /*
          Calculate whether the measurements were valid
        */
        int deviationCounter = 0;

        for (int i = 0; i < mmPointIterationGoal; i++) {
          int currentMm = measurementPoints[i];

          if (currentMm < 50) { // If the frequency is way to low we know that we have either left or have entered a overtone heavy portion, so we invalidate the entire measurement.
            validRun = false;
            break;
          }
          if (abs(modeOfMms - currentMm) > deviationAllowance) deviationCounter ++;
        }

        if (deviationCounter > (2/3 * mmPointIterationGoal)) validRun = false;

        if (validRun) { // Save into measurements and count up the measurement counter.
          measurements[measurementCounter] = modeOfMms;
          measurementCounter ++;
        }
        Serial.println(modeOfMms);
      }
    }

    int freqMeasurement() {
      mmCreator();
      // return result;
    }

  public:
    void measurement() {
      while (!validMeasurement && mode == Measurement) freqMeasurement();
      // if (mode == Measurement) return;
    }

    void reset() {
      // TODO: Write a reset function, so that the options can get called at any point in time
    }
};

class tuningMode{
  public:
    void tuning() {

    }
};

class optionsMode{
  public:
    void optionsInterrupt() {

    }
    void options() {

    }
};


// measurementMode *mmModeObj = new measurementMode();
// tuningMode *tModeObj = new tuningMode();
// optionsMode *oModeObj = new optionsMode();

measurementMode mmModeObj;
tuningMode tModeObj;
optionsMode oModeObj;

void loop() {
  Serial.println("Loop");
  Serial.println(mode);
  switch (mode) {
    case Measurement:
      Serial.println("Measurement");
      mmModeObj.measurement();

      break;

    case Tuning:
      Serial.println("Tuning");
      tModeObj.tuning();
      break;

    case Options:
      Serial.println("Options");
      oModeObj.options();
      break;
  }
}

