// Version 0.10
// Copyright © 2020 Leonhard Saam
// Licensed under GNU GPL v3

#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "MemoryFree.h"
#include <Arduino.h>
#include <assert.h>

#define I2C_ADDR 0x27 //Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

int readCount = 3; // Does this have any purpose?
int pinreads[] = {120, 89, 90};
int readpos = 0;


LiquidCrystal_I2C lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

int topTrigger = 540;
int bottomTrigger = 460;
const long usInSec = 1000000;

const int measurementIterationGoal = 6;
const int mmPointIterationGoal = 15; // With this configuration we can only support a max of 50 hz.
const int probeIterationGoal = 500;

const int deviationAllowance = 20;
const int mmPtVarianceAllowance = 5;
const int mmVarianceAllowance = 5;
const int mmPtNeighbourVarianceAllowance = 20;
const int mmNeighbourVarianceAllowance = 20;

const int minBarrier = 50;
const int inputPin = A0;

enum Mode
{
  Measurement,
  Tuning,
  Options
};
int mode;

void setup()
{
  Serial.begin(500000);
  // attachInterrupt(digitalPinToInterrupt(2), interrupt, FALLING);
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

class measurer {
private:
  bool validMeasurement = true;
  bool validRun = true;
  int measurementPoints[mmPointIterationGoal] = {119, 89, 90};
  int measurements[measurementIterationGoal];

  void overMinBarrier() {
    int lowCounter = 0;
      for (int i = 0; i < mmPointIterationGoal; i++) {
        int& currentVal = measurementPoints[i];
        if (currentVal < 50) {
          lowCounter ++;
        }
      }
      if (lowCounter > (1./3. * mmPointIterationGoal)) {
        validRun = false;
      }

  }
                    //                        measurementIterationGoal     mmVarianceAllowance              mmVarianceAllowance
                    // measurementPoints[]    mmPointIterationGoal         mmPtNeighbourVarianceAllowance   mmPtVarianceAllowance
  float averageFinder(int* inputArrayPtr, const int& inputArrayLenght, const int& neighbourVarianceAllowance, const int& varianceAllowance) {
    int*& inputArray = inputArrayPtr;
    int maxValue = inputArray[0]; // Determine maximal and minimal value
    int minValue = inputArray[0];

    for (int i = 0; i < inputArrayLenght; i++)
    {
      int& currentVal = inputArray[i]; // Experimental
      if (currentVal > maxValue)
        maxValue = currentVal;
      if (currentVal < minValue)
        minValue = currentVal;
    }

    int necessaryVal = maxValue - minValue + 1; // Needed Size of neighbourCounter

    if (necessaryVal >= neighbourVarianceAllowance) // We don't need to do all of this if all values are the same or close enough together to take a clean mean without preprocessing.
    {
      byte neighbourCounter[necessaryVal] = {0};
      // Serial.print("Necessary value: ");
      // Serial.println(necessaryVal);

      for (int i = 0; i < inputArrayLenght; i++) // Create histogram
      {
        int currentVal = inputArray[i];
        int posValue = currentVal - minValue;
        int posValue2 = maxValue - currentVal;

        for (int j = 1; j <= varianceAllowance && j <= posValue; j++){
          int writepos = posValue - j;
          int addVal = varianceAllowance - j;
          assert(writepos < necessaryVal);
          assert(addVal <= varianceAllowance);
          neighbourCounter[writepos] += addVal;
        }
        for (int j = 0; j <= varianceAllowance && j <= posValue2; j++){
          int writepos = posValue + j;
          int addVal = varianceAllowance - j;
          assert(writepos < necessaryVal);
          assert(addVal <= varianceAllowance);
          neighbourCounter[writepos] += addVal;
        }
      }
      for (int i = 0; i < necessaryVal; i++) {
        int currentVal = neighbourCounter[i];
        // Serial.print("Ncounter: ");
        // Serial.println(currentVal);
      }
      int maxNeighbours = 0; // Get Point with most variables nearby
      int mNValuesSize = 0;
      int mNValuesIndex = 0;

      for (int i = 0; i < necessaryVal; i++){
        int currentVal = neighbourCounter[i];
        if (currentVal > maxNeighbours) maxNeighbours = currentVal;
      }

      for (int i = 0; i < necessaryVal; i++){
        int currentVal = neighbourCounter[i];
        if (currentVal == maxNeighbours) mNValuesSize ++;
      }

      int mNValues [mNValuesSize];
      float mean;

      for (int i = 0; i < necessaryVal; i++){
        int currentVal = neighbourCounter[i];
        if (currentVal == maxNeighbours){
            mNValues[mNValuesIndex] = minValue + i;
            // Serial.println(minValue + i);
            mNValuesIndex ++;
        }
      }

      // Maybe set max deviation to 50 and then take median/mean from there and maybe take 20 values around mean and take the median of them
      if ((mNValues[mNValuesSize - 1] - mNValues[0] + 1) <= neighbourVarianceAllowance) { // Actually determine mean
        long sumOfVals = 0;
        for (int i = 0; i < mNValuesSize; i++){
          sumOfVals += mNValues[i];
        }
        mean = (float) sumOfVals / (float) mNValuesSize;
        return mean;
      }

      else {
        return 0;
      }
    }

    else if (necessaryVal == 1) {
      return inputArray[0];
    }

    else {
      int sumOfmmPoints = 0;
      for (int i = 0; i < inputArrayLenght; i++)
        sumOfmmPoints += inputArray[i];
      float mean = (float) sumOfmmPoints / (float) inputArrayLenght;
      return mean;
    }
  }

  void mmPointCreator() {
    for (int mmPointIteration = 0; mmPointIteration < mmPointIterationGoal; mmPointIteration++) // Create measurement points
    {
      int waveCounter = -2; // Account for start synchronistion
      long startTime;
      bool completeWave = true;

      while (waveCounter < 10) // Add two to allow for a good starting time
      {
        int input = analogRead(inputPin);
        if (completeWave && input >= topTrigger)
        {
          waveCounter++;
          completeWave = false;
          if (waveCounter == 0)
            startTime = micros();
        }
        else if (input <= bottomTrigger)
          completeWave = true; // Avoid adding incomplete waves to the wave counter
      }

      long endTime = micros(); // Calculate Frequency
      long timeDiff = endTime - startTime;
      long timepWave = timeDiff / waveCounter;
      int freq = usInSec / timepWave;
      measurementPoints[mmPointIteration] = freq;

      // Serial.print("Measurement point creator: ");
      // Serial.println(freq);
    }
  }

  float mmCreator()
  {
    int measurementIndex = 0;
    validMeasurement = true;
    while (measurementIndex < measurementIterationGoal)
    {
      validRun = true;
      float mean;
      mmPointCreator();
      overMinBarrier();

      if (validRun) {
        mean = averageFinder(measurementPoints, mmPointIterationGoal, mmPtNeighbourVarianceAllowance, mmPtVarianceAllowance);
        Serial.print("Mean: ");
        Serial.println(mean);

        measurements[measurementIndex] = round(mean);
        measurementIndex ++;
      }
      else
        Serial.println("Invalid run");

      lcd.setCursor(0,0);
      lcd.print("Zu messsen: ");
      lcd.setCursor(12,0);
      byte toMeasure = measurementIterationGoal - measurementIndex;
      lcd.print(toMeasure);
      lcd.setCursor(0,1);
      lcd.print("Durchlaeufe");
    }

    float mean = averageFinder(measurements, measurementIterationGoal, mmNeighbourVarianceAllowance, mmVarianceAllowance);
    if (mean == 0) {
      validMeasurement = false;
    }
    return mean;
  }

public:
  float measure() {
    float result = mmCreator();

    if (validMeasurement) {
      Serial.print("End result: ");
      Serial.println(result);
    }
    else
    {
      Serial.println("Invalid measurement");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Neue Messung");
      lcd.setCursor(0,1);
      lcd.print("notwendig");
      delay(2000);
    }
    Serial.println("===============================================================================");
    return result;
  }
} measurerObj;

class measurementMode {
public:
  void measurement() {
    float freq = measurerObj.measure();
    float force = calculateforce(freq);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Kraft in N: ");
    lcd.setCursor(12,0);
    lcd.print(force);
    lcd.setCursor(0,1);
  }

private:
  float calculateforce(float& freq) // Incomplete and undynamic
  {
    const float m = 3.5387;
    const float b = 67.05;
    float force = freq * m - b;
    return force;
  }
} mmModeObj;

class tuningMode
{
public:
  void tuning()
  {
  }
} tModeObj;

class optionsMode
{
public:
  void optionsInterrupt()
  {
  }
  void options()
  {
  }
} oModeObj;

void loop()
{
  Serial.println("Loop");
  Serial.println(mode);
  switch (mode)
  {
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
