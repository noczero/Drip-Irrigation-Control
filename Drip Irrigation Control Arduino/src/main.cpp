/* 
To do :
1. Read soil moist (done)
2. Read all ph (done)
3. Send data to esp8266 
4. Add logic for controling solenoid

@note :
change 
*/

#include <Arduino.h>
#include "I2CScanner.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>

#define DEBUG

// relay define
#define R1 22
#define R2 24
#define R3 26
#define R4 28
#define R5 30
#define R6 32
#define R7 34
#define R8 36
#define R9 38
#define R10 40
#define R11 42
#define R12 44
#define R13 23
#define R14 25
#define R15 27
#define R16 29
#define R17 31
#define R18 33
#define R19 35
#define R20 37
#define RSolenoidWater 39
#define RSolenoidTreat 41
#define RPump 43

// define analog pins as array
static const uint8_t soilHumidityAnalogPins[4] = {A0, A1, A2, A3};
static const uint8_t phAnalogPins[12] = {A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

// define relay pins as array
static const uint8_t solenoidPrimerPins[23] = {R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15, R16, R17, R18, R19, R20, RSolenoidWater, RSolenoidTreat, RPump};

// array value
int soilHumidityValue[20]; // array length 20
char soilHumidityCode[20];
int phValue[12]; // ph length 12
boolean solenoidPrimerStatus[23];
char solenoidPrimerStatusCode[23];

// relay command
int relay_command_id = -1;  // set init value
String command_status = ""; // this value store command, available command water for watering, treat for treatment.

I2CScanner scanner;

// ads115 instance
Adafruit_ADS1115 adsFirst, adsSecond, adsThird, adsFourth; /* Use this for the 12-bit version */

// i2c communication address
#define I2CAddressSlave 0x08
#define I2CADdressMaster 4

char dataSend[168];
// Soil, Solenoid, pH
char allDataFormat[168] = "OK,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d";

// send data to master
void sendToMaster()
{
  sprintf(dataSend, allDataFormat,
          soilHumidityCode[0],
          soilHumidityCode[1],
          soilHumidityCode[2],
          soilHumidityCode[3],
          soilHumidityCode[4],
          soilHumidityCode[5],
          soilHumidityCode[6],
          soilHumidityCode[7],
          soilHumidityCode[8],
          soilHumidityCode[9],
          soilHumidityCode[10],
          soilHumidityCode[11],
          soilHumidityCode[12],
          soilHumidityCode[13],
          soilHumidityCode[14],
          soilHumidityCode[15],
          soilHumidityCode[16],
          soilHumidityCode[17],
          soilHumidityCode[18],
          soilHumidityCode[19],
          solenoidPrimerStatusCode[0],
          solenoidPrimerStatusCode[1],
          solenoidPrimerStatusCode[2],
          solenoidPrimerStatusCode[3],
          solenoidPrimerStatusCode[4],
          solenoidPrimerStatusCode[5],
          solenoidPrimerStatusCode[6],
          solenoidPrimerStatusCode[7],
          solenoidPrimerStatusCode[8],
          solenoidPrimerStatusCode[9],
          solenoidPrimerStatusCode[10],
          solenoidPrimerStatusCode[11],
          solenoidPrimerStatusCode[12],
          solenoidPrimerStatusCode[13],
          solenoidPrimerStatusCode[14],
          solenoidPrimerStatusCode[15],
          solenoidPrimerStatusCode[16],
          solenoidPrimerStatusCode[17],
          solenoidPrimerStatusCode[18],
          solenoidPrimerStatusCode[19],
          solenoidPrimerStatusCode[20],
          solenoidPrimerStatusCode[21],
          solenoidPrimerStatusCode[22],
          phValue[0],
          phValue[1],
          phValue[2],
          phValue[3],
          phValue[4],
          phValue[5],
          phValue[6],
          phValue[7],
          phValue[8],
          phValue[9],
          phValue[10],
          phValue[11]);

  Wire.write(dataSend);
  Serial.println(dataSend);
}

void controlRelay(String action, String Code, String RelayID)
{
  int relayIdx = RelayID.toInt() - 1;
  if (action == "ON")
  {
    if (Code == "WATER")
    {
      digitalWrite(RSolenoidWater, LOW);               // ON
      digitalWrite(RSolenoidTreat, HIGH);              // OFF
      digitalWrite(RPump, LOW);                        // ON
      digitalWrite(solenoidPrimerPins[relayIdx], LOW); //ON
    }
    else if (Code == "TREAT")
    {
      digitalWrite(RSolenoidTreat, LOW);               // ON
      digitalWrite(RSolenoidWater, HIGH);              // OFF
      digitalWrite(RPump, LOW);                        // ON
      digitalWrite(solenoidPrimerPins[relayIdx], LOW); //ON
    }
  }
  else if (action == "OFF")
  {
    digitalWrite(RSolenoidWater, HIGH);               // ON
    digitalWrite(RSolenoidTreat, HIGH);               // OFF
    digitalWrite(RPump, HIGH);                        // ON
    digitalWrite(solenoidPrimerPins[relayIdx], HIGH); //ON
  }

// turn off another
//for (int i = 0; i < 20; i ++){
//  if (i != relayIdx){
//    digitalWrite(RSolenoidTreat, HIGH); // OFF
//  }
//}
}

// read data from master
void readFromMaster(int howMany)
{
  Serial.println("Command : ");
  String response = "";
  while (0 < Wire.available())
  {
    char c = Wire.read(); /* receive byte as a character */
    //Serial.print(c);      /* print the character */
    response += c;
  }
  Serial.println(response); /* to newline */
  Serial.println();

  // String to array
  char responseChar[13];
  response.toCharArray(responseChar, 13);

  // initialize first part (string, delimiter)
  char *ptr = strtok(responseChar, ",");
  int rows = 0;

  String receivedData[3];
  while (ptr != NULL)
  {
    //Serial.println(ptr);
    receivedData[rows] = ptr;
    // create next part
    ptr = strtok(NULL, ",");
    rows++;
  }
  //Serial.println(rows);
  if (rows == 3) {
      Serial.println("Action:" + receivedData[0]);
      Serial.println("Code: " + receivedData[1]);
      Serial.println("Solenoid: " + receivedData[2]);

      controlRelay(receivedData[0],receivedData[1],receivedData[2]);
  }
}

// setup i2c communication
void setupI2CCom()
{
  // SLave
  Wire.begin(I2CAddressSlave);
  Wire.setWireTimeout(6000, true);
  Wire.onReceive(readFromMaster); /* register receive event */
  Wire.onRequest(sendToMaster);   /* register request event */
}

// detect i2c device for ADS115
void i2cScanner()
{
  scanner.Init();
  scanner.Scan();
}

void setupRelay()
{
  // Using Normally Open Configuration Relay, LOW for ON, HIGH for OFF

  for (int i = 0; i < 20; i++)
  {
    pinMode(solenoidPrimerPins[i], OUTPUT);
    digitalWrite(solenoidPrimerPins[i], HIGH); // OFF
  }

  pinMode(RSolenoidTreat, OUTPUT);
  digitalWrite(RSolenoidTreat, HIGH); // OFF

  pinMode(RSolenoidWater, OUTPUT);
  digitalWrite(RSolenoidWater, HIGH); // OFF

  pinMode(RPump, OUTPUT);
  digitalWrite(RPump, HIGH); // OFF
}

void setupADS1115()
{
  adsFourth.begin(0x48);
  adsFirst.begin(0x4B);
  adsSecond.begin(0x4A);
  adsThird.begin(0x49);
}

// convert raw humidity to string
int percentageMoist = 0;
char convertSoilValueToText(int rawValue, boolean ADS)
{
  //    Convert RAW Analog to discrete (low, normal, high) for soil moist level
  if (ADS)
  {
    // RAW Value ADS Module range 0-32767
    percentageMoist = map(rawValue, 0, 32767, 0, 100);
  }
  else
  {
    // using standar Analog, 0-1024
    percentageMoist = map(rawValue, 0, 1024, 0, 100);
  }

  if (25 < percentageMoist && percentageMoist <= 100)
  {
    return 'H'; // HIGH
  }
  else if (15 <= percentageMoist && percentageMoist <= 25)
  {
    return 'N'; // N
  }
  else if (0 < percentageMoist && percentageMoist < 15)
  {
    return 'L'; // L
  }
}

// read ads
void readSoilHumidity()
{
  int adsCH = 0; // value 0,1,2,3 then reset

  // iterate over 20 soil humidity sensor, ADS and Analog
  for (int i = 0; i < 20; i++)
  {

    // reset index channel
    if (adsCH % 4 == 0)
    {
      adsCH = 0;
    }

    if (i < 4)
    {
      // for first ads, range index 0-3
      soilHumidityValue[i] = adsFirst.readADC_SingleEnded(adsCH);

      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], true);
    }
    else if (i < 8)
    {
      // for second ads, range index 4-7
      soilHumidityValue[i] = adsSecond.readADC_SingleEnded(adsCH);

      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], true);
    }
    else if (i < 12)
    {
      // for third ads, range index 8-11
      soilHumidityValue[i] = adsThird.readADC_SingleEnded(adsCH);

      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], true);
    }
    else if (i < 16)
    {
      // for foruth ads, range index 12-15
      soilHumidityValue[i] = adsFourth.readADC_SingleEnded(adsCH);

      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], true);
    }
    else
    {
      // read analog PIN, range index 16-19
      soilHumidityValue[i] = analogRead(soilHumidityAnalogPins[adsCH]);

      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], false);
    }

    adsCH++;
  }
}

void readPH()
{
  for (int i = 0; i < 12; i++)
  {
    phValue[i] = analogRead(phAnalogPins[i]);
  }
}

char convertRelayToTxt(int relayStatus)
{
  if (relayStatus)
  {
    return '1'; // HIGH
  }
  else
  {
    return '0'; // LOW
  }
}

void readRelay()
{
  for (int i = 0; i < 23; i++)
  {
    int relayStatus = digitalRead(solenoidPrimerPins[i]);
    solenoidPrimerStatus[i] = relayStatus;
    solenoidPrimerStatusCode[i] = convertRelayToTxt(relayStatus);
  }
}

unsigned long intervalMonitoring = 100;     // the time we need to wait
unsigned long previousMillisMonitoring = 0; // millis() returns an unsigned long.
void monitoring()
{
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (2000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisMonitoring) >= intervalMonitoring)
  {
    previousMillisMonitoring = millis(); // grab current time as previous millis

    // read sensor
    readSoilHumidity();
    readPH();

    // read relay
    readRelay();
  }
}

unsigned long intervalctrlR1 = 1000 * 60 * 2; // the time we need to wait 2 minutes
unsigned long previousMillisctrlR1 = 0;       // millis() returns an unsigned long.
void ctrlR1()
{
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (2000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisctrlR1) >= intervalctrlR1)
  {
    previousMillisctrlR1 = millis(); // grab current time as previous millis

    // shutdown the relay
    digitalWrite(relay_command_id, LOW);
  }
}

// debug mode, print information to serial
unsigned long intervalDEBUG = 500;     // the time we need to wait
unsigned long previousMillisDEBUG = 0; // millis() returns an unsigned long.
void debug()
{
#ifdef DEBUG
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (2000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisDEBUG) >= intervalDEBUG)
  {
    previousMillisDEBUG = millis(); // grab current time as previous millis

    Serial.print(", Soil Hum.:[");

    // debug received value from aarduino
    // parse soil humidty
    for (int i = 0; i < 20; i++)
    {
      /* code */
      Serial.print(soilHumidityValue[i]);
      Serial.print(",");
    }
    Serial.print("]");
    Serial.print(", pH : [ ");

    // parse ph value
    for (int i = 0; i < 20; i++)
    {
      /* code */
      Serial.print(phValue[i]);
      Serial.print(",");
    }

    Serial.print("]");
    Serial.print(", solenoid : [ ");

    // parse solenoid primer
    for (int i = 0; i < 20; i++)
    {
      /* code */
      Serial.print(solenoidPrimerStatus[i]);
      Serial.print(",");
    }

    Serial.print("] Solenoid Water : ");
    Serial.print(digitalRead(RSolenoidWater));

    Serial.print(", Solenoid Treat : ");
    Serial.print(digitalRead(RSolenoidTreat));

    Serial.print(", Water Pump : ");
    Serial.print(digitalRead(RPump));

    Serial.println();
  }
#endif
}

void setup()
{
  // Serial 0
  Serial.begin(9600);
  while (!Serial)
  {
  };

  // Serial 1, for communication to esp8266
  Serial1.begin(9600);
  while (!Serial1)
  {
  };

  // Setup i2c scanner
  i2cScanner();

  // set all relay as output
  setupRelay();

  // setup ads
  setupADS1115();

  // SETUP I2C COMMUNICATION
  setupI2CCom();
}

void loop()
{
  monitoring();

  debug();

  delay(500);
}
