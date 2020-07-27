// Version 2.4
// Copyright © 2020 Leonhard Saam
// Licensed under GNU GPL v3

// Initsialisierung der Variablen

// Oberer und unterer Schwellenwert als analoger Wert
int toptrigger = 640;
int bottomtrigger = 280;

float anzahl = 1000000; // Anzahl der vorhandenen Microsekunden in einer Sekunde.

float time; // Zeitpunkt des Beginns der Zählung.
float time2; // Zeitpunkt des Endes der Zählung.

bool status = false; // Angabe, ob der vorherige Wert minus war(Zur Entfernung von Mehrfachzählfehlern.

int eingabe; // Gemessener analoger Wert
int wert; // Anzahl der ermittelten Schwingungswellen

bool boolbasefreq = true; // Bestimmung ob der Durchlauf die Richtfrequenz zum Ermitteln des Frequenzunterschiedes festlegt
int basefreq = 0; // Die Richtfrequenz, gegen die mit Gewicht gemessene Werte verglichen werden

// Speicherungen der Variable "wert"
int save_1 = 0; 
int save_2 = 0; 
int save_3 = 0; 

// Underschied der "save_" Variablen voneinander
int calc1 = 0;
int calc2 = 0;
int calc3 = 0;

int savecounter = 0; // Anzahl der Durchlaufe der Speicherung der Variable "wert" 
int addedsaves = 0; // Die Durchschnittsfrequenz von 2 bzw. 3 "save_" Werten


bool gotrough = true; // Bestimmt, ob ein neuer Wert gemessen werden soll, da der aktuelle außerhalb des Sprektrums ist
bool calcit = false; // Bestimmt ob nach diesem Speicherungsdurchgang das Endergebnis ausgerechnet werden soll

bool saveruntrue = true; // Bestimmt, ob in diesem Durchgang schon Saves gespeichert wurden
int saverun = 0; // Nummer der Speicherungsdurchgänge von Durchschnittswerten

// Saves, aus denen das Endergebnis errechnet wird
int bigsave_1 = 0;
int bigsave_2 = 0;
int bigsave_3 = 0;
int bigsave_4 = 0;
int bigsave_5 = 0;
int bigsave_6 = 0;

// Bestimmt ob der geweilg dazugehörige Save in das Endergebnis einbezogen wird
int addup_2 = 0;
int addup_3 = 0;
int addup_4 = 0;
int addup_1 = 0;
int addup_5 = 0;

bool notzero = false; // Verhinderung, dass der 6. Gespeicherte Wert 0 ist
bool saveit = false; // Bestimmt ob "addedsaves" als Save gespeichert werden soll

int calcupall = 0; // Summe der addierten Saves
int endcalc; // Endergebnis

bool negative = false; // Bestimmt ob der erste Wert verwendet wurde

bool usenegative = true; // Bestimmt, ob der Richtwert, wenn der 1. Wert ungültig ist, schon gesetzt ist
int notused = 0; // Richtwert für die Überprüfung der Variablen, wenn der erste Wert nicht zählt

// Die Differenz zwischen den überbleibenden Werten, wenn der erste Wert negativ ist.
int hugecalc1 = 0;
int hugecalc2 = 0;
int hugecalc3 = 0;
int hugecalc4 = 0;

int notusedcounter = 0; // Zählt, wie viele Werte nicht verwendet wurden, wenn der erste Wert nicht verwendet wird

// Bestimmt, ob die dazugehörigen Saves verwendet werden sollen, wenn sie fälschlicherweise nicht verwendet wurden
bool pleaseuse4 = false;
bool pleaseuse3 = false;
bool pleaseuse2 = false;
bool pleaseuse1 = false;




void setup() {
  Serial.begin(9600); // Initsialisierung des Ausgabe-Protokolls.
}


// Frequenzermittler für Signal
int frequenzmesser()
{
  while (gotrough == true) // Verhindert, dass kein Wert gespeichert wird
  {
    for (int rounds = 1; rounds <= 3; rounds ++) // Misst die Frequenz 3x
    {
      time = micros(); // Messung des Beginns der Zählung.
      for (int durchlauf = 0; durchlauf < 1000; durchlauf ++) // Schleife zur Begrenzung der Anzahl an Messungen.
      {
        eingabe = analogRead(A0); // Einlesung des Analog-Wertes.
        if (eingabe >= toptrigger && status == false) // Bedingung zur Bestimmung ob Eingabe den Grenzwert überschreitet oder nicht
        {
          wert ++; // Erhöhung der Anzahl der positiven Schwingungswellen um eins.
          status = true; // Setzung von status auf "über dem Grenzwert"
        }
        else if (eingabe <= bottomtrigger ) // Absicherung ob Eingabe wirklich unter dem unteren Grenzwert ist.
        {
          status= false; // Setzung von status "unter dem unteren Grenzwert"
        }
      }
      time2 = micros(); // Ermitteln des Endzeitpunktes der Zählung.
      time2 -= time; // Errechnen der insgesamten Durchlaufdauer der Zählungen.
      time2 /= wert; // Errechnen der Dauer einer positiven Welle.
      anzahl /= time2; // Errechnen der Frequenz des Signals.
      if (savecounter == 0) // Ermittelt in welche Variable die errechnete Frequenz gespeichert werden muss
      {
        save_1 = anzahl; 
      }
        if (savecounter == 1)
      {
        save_2 = anzahl; 
      }
        if (savecounter == 2)
      {
        save_3 = anzahl; 
        Serial.print("Gemessene Werte: ");
        Serial.print(save_1);
        Serial.print("; ");
        Serial.print(save_2);
        Serial.print("; ");
        Serial.print(save_3);
        Serial.println(";");
      }

      savecounter ++; // Erhöhung des Durchgangzählers um 1
      anzahl = 1000000; // Zurücksetzung von anzahl.
      time2 = 0; // Zurücksetzung von time2.
      time = 0; // Zurücksetzung von time.
      status = false; // Zurücksetzung von status.
      wert = 0; // Zurücksetzung der Zählungsanzahl.
    }
    calc1 = save_1-save_2; // Errenchnung des Unterschiedes zwischen den gespeicherten Werten
    calc2 = save_1-save_3;
    calc3 = save_2-save_3;
    saveruntrue = true; // Legt fest, ob in diesem Durchgang ein Save gespeichert werden soll
    if (calc1 < 10 && calc1 > -10 && calc2 < 10 && calc2 > -10 && calc3 < 10 && calc3 > -10) // Bestimmt welche der 3 Werte genommen werden sollen
    {
      addedsaves = (save_1 + save_2 + save_3) /3;
      saveit = true;
    }  
    if ((calc1 > 10 || calc1 < -10) && (calc2 > 10 || calc2 < -10) && (calc3 < 10 && calc3 > -10))
    {
      addedsaves = (save_2 + save_3) /2;
      saveit = true;
    }
    if ((calc1 < 10 && calc1 > -10) && (calc2 > 10 || calc2 < -10) && (calc3 > 10 || calc3 < -10))
    {
      addedsaves = (save_1 + save_2) /2;  
      saveit = true;
    }
    if ((calc1 > 10 || calc1 < -10) && (calc2 < 10 && calc2 > -10) && (calc3 > 10 || calc3 < -10))
    {
      addedsaves = (save_1 + save_3) /2;
      saveit = true;
    }
    
    if (saveit == true) // Ermittelt ob ein Wert gespeichert werden soll
    {
      if (saverun == 0) // Ermittelt in welchen Save die Daten gespeichert werden sollen
      {
        bigsave_1 = addedsaves;
        if (bigsave_1 < 10 || bigsave_1 > 400 || bigsave_1 < 150) // Bestimmt, ob der Wert im Spektrum ist 
        {
          saverun --;
        }
        saverun ++;
        saveruntrue = false;
      }
      if (saverun == 1 && saveruntrue == true)
      {
        bigsave_2 = addedsaves;
        if (bigsave_2 < 10 || bigsave_2 > 400 || bigsave_2 < 150)
        {
          saverun --;
        }
        saverun ++;
        saveruntrue = false;
      }
      if (saverun == 2 && saveruntrue == true)
      {
        bigsave_3 = addedsaves;
        if (bigsave_3 < 10  || bigsave_3 > 400 || bigsave_3 < 150)
        {
          saverun --;
        }
        saverun ++;
        saveruntrue = false;
      }
      if (saverun == 3 && saveruntrue == true)
      {
        bigsave_4 = addedsaves;
        if (bigsave_4 < 10 || bigsave_4 > 400 || bigsave_4 < 150)
        {
          saverun --;
        }
        saverun ++;
        saveruntrue = false;
      }
      if (saverun == 4 && saveruntrue == true)
      {
        bigsave_5 = addedsaves;
        if (bigsave_5 < 10 || bigsave_5 > 400 || bigsave_5 < 150)
        {
          saverun --;
        }
        saverun ++;
        saveruntrue = false;
      }
      if (saverun == 5 && saveruntrue == true)
      {
        bigsave_6 = addedsaves;
        notzero = true;
        if (bigsave_6 < 10  || bigsave_6 > 400 || bigsave_6 < 150)
        {
          saverun --;
          notzero = false;
        }
        if (notzero == true) // Verhindert, dass der 6. Save 0 ist
        {
          calcit = true;
        }

        saverun ++;
        saveruntrue = false;
      }
      saveit = false;      
    }
    Serial.print("Speicherungsdurchgang: ");
    Serial.println(saverun);
    if (calcit == true) // Rechnet das Endergebnis aus
    {
      Serial.println();
      Serial.println();
      Serial.println("Ausrechnen:");
      Serial.println();
      calcit = false;
      saverun = 0;

      int bigcalc_1 = bigsave_1 - bigsave_2; // Differenz zwischen den Saves
      int bigcalc_2 = bigsave_1 - bigsave_3;
      int bigcalc_3 = bigsave_1 - bigsave_4;
      int bigcalc_4 = bigsave_1 - bigsave_5;
      int bigcalc_5 = bigsave_1 - bigsave_6;

      Serial.println("Freqenzdifferenzen zwischen den Saves:");
      Serial.print("Die Differenz zwischen dem 1. und dem 2. Wert ist: ");
      Serial.println(bigcalc_1);
      Serial.print("Die Differenz zwischen dem 1. und dem 3. Wert ist: ");
      Serial.println(bigcalc_2);
      Serial.print("Die Differenz zwischen dem 1. und dem 4. Wert ist: ");
      Serial.println(bigcalc_3);
      Serial.print("Die Differenz zwischen dem 1. und dem 5. Wert ist: ");
      Serial.println(bigcalc_4);
      Serial.print("Die Differenz zwischen dem 1. und dem 6. Wert ist: ");
      Serial.println(bigcalc_5);
      Serial.println();
      
      if (bigcalc_1 < 10 && bigcalc_1 > -10) // Bestimmt, ob die Differenzen im Spektrum sind und markiert die zu verwendenden Saves
      {
        addup_1 = 1;       }
      else {
        addup_1 = 0;
      }
      if (bigcalc_2 < 10 && bigcalc_2 > -10)
      {
        addup_2 = 1;      }
      else {
        addup_2 = 0;
      }
      if (bigcalc_3 < 10 && bigcalc_3 > -10)
      {
        addup_3 = 1;      }
      else {
        addup_3 = 0;
      }
      if (bigcalc_4 < 10 && bigcalc_4 > -10)
      {
        addup_4 = 1;
      }
      else{
        addup_4 = 0;
      }
      if (bigcalc_5 < 10 && bigcalc_5 > -10)
      {
        addup_5 = 1; 
      }
      else {
        addup_5 = 0;
      }
      int addall = addup_1 + addup_2 + addup_3 + addup_4 + addup_5; // Rechnet aus, wie viele Werte verwendet werden
      int usecount = 0;
      int usecount_1 = 0;
      if (addall >= 3) // Ermittelt, ob man den ersten Wert verwenden kann
      {
        Serial.println("Es sind mindestens 3 Werte ähnlich genug zu dem ersten Wert.");
        Serial.println();
        Serial.println("Der 1. Save wird verwendet.");
        negative = false;
        calcupall = calcupall + bigsave_1;
        usecount = 1; 
        if (addup_1 == 1) // Wenn der Wert zu "zu verwenden" gesetzt wurde, wird er zu der Summe aller Saves hinzugerechnet
        {
          calcupall = calcupall + bigsave_2;
          usecount ++;
          Serial.println("Der 2. Save wird verwendet.");
        }
        if (addup_2 == 1)
        {
          calcupall = calcupall + bigsave_3;
          usecount ++;
          Serial.println("Der 3. Save wird verwendet.");
        }
        if (addup_3 == 1)
        {
          calcupall = calcupall + bigsave_4;
          usecount ++;
          Serial.println("Der 4. Save wird verwendet.");
        }
        if (addup_4 == 1)
        {
          calcupall = calcupall+ bigsave_5;
          usecount ++;
          Serial.println("Der 5. Save wird verwendet.");
        }
        if (addup_5 == 1)
        {
          calcupall = calcupall + bigsave_6;
          usecount ++;
          Serial.println("Der 6. Save wird verwendet.");
        }
        Serial.println();
        Serial.println("Die Werte der Saves:");
        Serial.print("Der Wert des 1. Saves: ");  
        Serial.println(bigsave_1);
        Serial.print("Der Wert des 2. Saves: ");
        Serial.println(bigsave_2);
        Serial.print("Der Wert des 3. Saves: ");
        Serial.println(bigsave_3);
        Serial.print("Der Wert des 4. Saves: ");
        Serial.println(bigsave_4);
        Serial.print("Der Wert des 5. Saves: ");
        Serial.println(bigsave_5);
        Serial.print("Der Wert des 6. Saves: ");
        Serial.println(bigsave_6);
        Serial.println();
      }
      else if (addall < 3) // Tritt ein, wenn der erste Wert nicht stimmt.
      {
        Serial.println("Es sind nicht genug Werte ähnlich genug zu dem ersten Wert.");
        Serial.println();
        
        negative = true;
        if (addup_1 == 0) // Ermittelt, welche Werte verwendet werden können
        {
          calcupall = calcupall + bigsave_2;
          usecount_1 ++;
          notused = bigsave_2;
          usenegative = false;
          Serial.println("Der 2. Save wird als Richtwert verwendet.");
          Serial.println("Der 2. Save wird verwendet.");
        }
        if (addup_2 == 0)
        {
          calcupall = calcupall + bigsave_3;
          usecount_1 ++;
          if (usenegative == true)
          {
            notused = bigsave_3;
            usenegative = false;
            Serial.println("Der 3. Save wird als Richtwert verwendet."); // Vergleicht Wert gegen Richtwert
          }
          else{hugecalc1 = notused - bigsave_3;} // Bestimmt was der Richtwert zum Vergleich der Variablen ist
          Serial.println("Der 3. Save wird verwendet.");
        }
        if (addup_3 == 0)
        {
          calcupall = calcupall + bigsave_4;
          usecount_1 ++;
          if (usenegative == true)
          {
            notused = bigsave_4;
            usenegative = false;
            Serial.println("Der 4. Save wird als Richtwert verwendet.");
          }
          else {hugecalc2 = notused - bigsave_4;}
          Serial.println("Der 4. Save wird verwendet.");
        }
        if (addup_4 == 0)
        {
          calcupall = calcupall + bigsave_5;
          usecount_1 ++;
          if (usenegative == true)
          {
            notused = bigsave_5;
            usenegative = false;
            Serial.println("Der 5. Save wird als Richtwert verwendet.");
          }
          else {hugecalc3 = notused - bigsave_5;}
          Serial.println("Der 5. Save wird verwendet.");
        }
        if (addup_5 == 0)
        {
          calcupall = calcupall + bigsave_6; //Wert wird zu Gesamtergebniss dazuaddiert.
          usecount_1 ++;
          Serial.println("Der 6. Save wird verwendet.");
          hugecalc4 = notused - bigsave_6;
        }
        Serial.println();
        if (!(hugecalc1 < 10 && hugecalc1 > -10)) // Bestimmt ob der Wert wirklich verwendet werden kann
        {
          calcupall = calcupall - bigsave_3;
          notusedcounter ++;
          usecount_1 --;
          pleaseuse1 = true;
          Serial.println("Der 3. Save wird doch nicht verwendet.");
        }
        if (!(hugecalc2 < 10 && hugecalc2 > -10))
        {
          calcupall = calcupall - bigsave_4;
          notusedcounter ++;
          usecount_1 --;
          pleaseuse2 = true;
          Serial.println("Der 4. Save wird doch nicht verwendet.");
        }
        if (!(hugecalc3 < 10 && hugecalc3 > -10))
        {
          calcupall = calcupall - bigsave_5;
          notusedcounter ++;
          usecount_1 --;
          pleaseuse3 = true;
          Serial.println("Der 5. Save wird doch nicht verwendet.");
        }
        if (!(hugecalc4 < 10 && hugecalc4 > -10))
        {
          calcupall = calcupall - bigsave_6;
          notusedcounter ++;
          usecount_1 --;
          pleaseuse4 = true;
          Serial.println("Der 6. Save wird doch nicht verwendet.");
        }
        Serial.println();
        if (notusedcounter >= 3) // Tritt ein, wenn zu viele Werte nicht verwendet werden
        {
          Serial.println("Der als Richtwert verwendete Save wird doch nicht verwendet.");
          Serial.println();
          calcupall = calcupall - notused;
          usecount_1 --;
          if (pleaseuse1 == true) // Verwendet die davor fälschlicherweise nicht genutzten Variablen
          {
            calcupall = calcupall + bigsave_3;
            usecount_1 ++;
            Serial.println("Der 3. Save wird doch verwendet.");
          }
          if (pleaseuse2 == true)
          {
            calcupall = calcupall + bigsave_4;
            usecount_1 ++;
            Serial.println("Der 4. Save wird doch verwendet.");
          }
          if (pleaseuse3 == true)
          {
            calcupall = calcupall + bigsave_5;
            usecount_1 ++;
            Serial.println("Der 5. Save wird doch verwendet.");
          }
          if (pleaseuse4 == true)
          {
            calcupall = calcupall + bigsave_6;
            usecount_1 ++;
            Serial.println("Der 6. Save wird doch verwendet.");            
          }
        }
        Serial.println();
        Serial.println("Die Werte der Saves:");
        Serial.print("Der Wert des 1. Saves: ");  
        Serial.println(bigsave_1);
        Serial.print("Der Wert des 2. Saves: ");
        Serial.println(bigsave_2);
        Serial.print("Der Wert des 3. Saves: ");
        Serial.println(bigsave_3);
        Serial.print("Der Wert des 4. Saves: ");
        Serial.println(bigsave_4);
        Serial.print("Der Wert des 5. Saves: ");
        Serial.println(bigsave_5);
        Serial.print("Der Wert des 6. Saves: ");
        Serial.println(bigsave_6);
        Serial.println();
      }
      if (negative == false) // Errechnet die Endfrequenz, wenn der erste Wert verwendet wird
      {
        endcalc = calcupall / usecount;
        Serial.print("Endergebnis: ");
        Serial.println(endcalc);
        calcupall = 0;
        gotrough = false; 
      }
      else // Gegenteil
      {
        endcalc = calcupall / usecount_1;
        Serial.print("Endergebnis: ");
        Serial.println(endcalc);
        calcupall = 0; 
        gotrough = false;       
      }
  }
  delay (250);
  savecounter = 0;
  save_1 = 0;
  save_2 = 0;
  save_3 = 0;
  }
  return endcalc; // Gibt das Endergebnis zuruek
}

void loop()
{
  if (boolbasefreq == true) // Setzt die Richtfrequenz
  {
    basefreq = frequenzmesser();
    boolbasefreq = false;
    delay (4000);
    gotrough = true;
  }
  else // Ermittelt die Differenz zwischen der Richtfrequenz und der Frequenz mit Gewicht
  {
    Serial.println();
    Serial.println("2. Frequenz: ");
    int freq = frequenzmesser ();
    int freqdiff = basefreq - freq;
    freqdiff = abs(freqdiff);
    Serial.print("Frequenzdiffernz zum Grundton: ");
    Serial.println(freqdiff);
    Serial.println();
    gotrough = true;
    delay (2000);
    
  }
}
