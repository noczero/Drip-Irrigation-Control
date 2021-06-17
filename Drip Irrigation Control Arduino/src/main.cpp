/* 
To do :
1. Read soil moist (done)
2. Read all ph (done)
16. Send data to esp8266 
4. Add logic for controling solenoid

@note :
change 
*/

#include <Arduino.h>
#include "I2CScanner.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>
#include <SchedTask.h>

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
bool isWaterModeSchedule, isWaterMode, isTreatMode, isOffMode, isOnMode = false;

I2CScanner scanner;

// ads115 instance
Adafruit_ADS1115 adsFirst, adsSecond, adsThird, adsFourth; /* Use this for the 12-bit version */

// i2c communication address
#define I2CAddressSlave 0x08
#define I2CADdressMaster 4

char dataSend[168];
// Soil, Solenoid, pH
char allDataFormat[168] = "OK,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d";

// Water Duration
const unsigned long WATERINGDURATION = 3000 * 60UL;       // 3 minutes
const unsigned long WATERINGMANUALDURATION = 2000 * 60UL; // 2 minutes
// const unsigned long WATERINGDURATION = 2000; // 2 SECONDS

// Scheduler
void turnOnSP1();
void turnOffSP1();
void turnOnSP2();
void turnOffSP2();
void turnOnSP3();
void turnOffSP3();
void turnOnSP4();
void turnOffSP4();
void turnOnSP5();
void turnOffSP5();
void turnOnSP6();
void turnOffSP6();
void turnOnSP7();
void turnOffSP7();
void turnOnSP8();
void turnOffSP8();
void turnOnSP9();
void turnOffSP9();
void turnOnSP10();
void turnOffSP10();
void turnOnSP11();
void turnOffSP11();
void turnOnSP12();
void turnOffSP12();
void turnOnSP13();
void turnOffSP13();
void turnOnSP14();
void turnOffSP14();
void turnOnSP15();
void turnOffSP15();
void turnOnSP16();
void turnOffSP16();
void turnOnSP17();
void turnOffSP17();
void turnOnSP18();
void turnOffSP18();
void turnOnSP19();
void turnOffSP19();
void turnOnSP20();
void turnOffSP20();

SchedTask turnOnSP1Task(NEVER, ONESHOT, turnOnSP1); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP1Task(NEVER, ONESHOT, turnOffSP1);

SchedTask turnOnSP2Task(NEVER, ONESHOT, turnOnSP2); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP2Task(NEVER, ONESHOT, turnOffSP2);

SchedTask turnOnSP3Task(NEVER, ONESHOT, turnOnSP3); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP3Task(NEVER, ONESHOT, turnOffSP3);

SchedTask turnOnSP4Task(NEVER, ONESHOT, turnOnSP4); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP4Task(NEVER, ONESHOT, turnOffSP4);

SchedTask turnOnSP5Task(NEVER, ONESHOT, turnOnSP5); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP5Task(NEVER, ONESHOT, turnOffSP5);

SchedTask turnOnSP6Task(NEVER, ONESHOT, turnOnSP6); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP6Task(NEVER, ONESHOT, turnOffSP6);

SchedTask turnOnSP7Task(NEVER, ONESHOT, turnOnSP7); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP7Task(NEVER, ONESHOT, turnOffSP7);

SchedTask turnOnSP8Task(NEVER, ONESHOT, turnOnSP8); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP8Task(NEVER, ONESHOT, turnOffSP8);

SchedTask turnOnSP9Task(NEVER, ONESHOT, turnOnSP9); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP9Task(NEVER, ONESHOT, turnOffSP9);

SchedTask turnOnSP10Task(NEVER, ONESHOT, turnOnSP10); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP10Task(NEVER, ONESHOT, turnOffSP10);

SchedTask turnOnSP11Task(NEVER, ONESHOT, turnOnSP11); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP11Task(NEVER, ONESHOT, turnOffSP11);

SchedTask turnOnSP12Task(NEVER, ONESHOT, turnOnSP12); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP12Task(NEVER, ONESHOT, turnOffSP12);

SchedTask turnOnSP13Task(NEVER, ONESHOT, turnOnSP13); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP13Task(NEVER, ONESHOT, turnOffSP13);

SchedTask turnOnSP14Task(NEVER, ONESHOT, turnOnSP14); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP14Task(NEVER, ONESHOT, turnOffSP14);

SchedTask turnOnSP15Task(NEVER, ONESHOT, turnOnSP15); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP15Task(NEVER, ONESHOT, turnOffSP15);

SchedTask turnOnSP16Task(NEVER, ONESHOT, turnOnSP16); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP16Task(NEVER, ONESHOT, turnOffSP16);

SchedTask turnOnSP17Task(NEVER, ONESHOT, turnOnSP17); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP17Task(NEVER, ONESHOT, turnOffSP17);

SchedTask turnOnSP18Task(NEVER, ONESHOT, turnOnSP18); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP18Task(NEVER, ONESHOT, turnOffSP18);

SchedTask turnOnSP19Task(NEVER, ONESHOT, turnOnSP19); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP19Task(NEVER, ONESHOT, turnOffSP19);

SchedTask turnOnSP20Task(NEVER, ONESHOT, turnOnSP20); // define the turn on task (dispatch now, one time)
SchedTask turnOffSP20Task(NEVER, ONESHOT, turnOffSP20);

SchedTask turnOnScheduleArray[] = {turnOnSP1Task, turnOnSP2Task, turnOnSP3Task, turnOnSP4Task, turnOnSP5Task,
                                   turnOnSP6Task, turnOnSP7Task, turnOnSP8Task, turnOnSP9Task, turnOnSP10Task,
                                   turnOnSP11Task, turnOnSP12Task, turnOnSP13Task, turnOnSP14Task, turnOnSP15Task,
                                   turnOnSP16Task, turnOnSP17Task, turnOnSP18Task, turnOnSP19Task, turnOnSP20Task};

// -- FUNCTION START
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

void waterMode()
{
  digitalWrite(RSolenoidWater, LOW);  // ON
  digitalWrite(RSolenoidTreat, HIGH); // OFF
  digitalWrite(RPump, LOW);           // ON
}

void treatMode()
{
  digitalWrite(RSolenoidTreat, LOW);  // ON
  digitalWrite(RSolenoidWater, HIGH); // OFF
  digitalWrite(RPump, LOW);           // ON
}

void offMode()
{
  digitalWrite(RSolenoidWater, HIGH); // OFF
  digitalWrite(RSolenoidTreat, HIGH); // OFF
  digitalWrite(RPump, HIGH);          // OFF

  // turn off RELAY
  for (int i = 0; i < 20; i++)
  {
    digitalWrite(solenoidPrimerPins[i], HIGH);
  }
}

void startTaskByID(int relayIdx)
{
  switch (relayIdx)
  {
  case 0:
    turnOnSP1Task.setNext(0);
    break;
  case 1:
    turnOnSP2Task.setNext(0);
    break;
  case 2:
    turnOnSP3Task.setNext(0);
    break;
  case 3:
    turnOnSP4Task.setNext(0);
    break;
  case 4:
    turnOnSP5Task.setNext(0);
    break;
  case 5:
    turnOnSP6Task.setNext(0);
    break;
  case 6:
    turnOnSP7Task.setNext(0);
    break;
  case 7:
    turnOnSP8Task.setNext(0);
    break;
  case 8:
    turnOnSP9Task.setNext(0);
    break;
  case 9:
    turnOnSP10Task.setNext(0);
    break;
  case 10:
    turnOnSP11Task.setNext(0);
    break;
  case 11:
    turnOnSP12Task.setNext(0);
    break;
  case 12:
    turnOnSP13Task.setNext(0);
    break;
  case 13:
    turnOnSP14Task.setNext(0);
    break;
  case 14:
    turnOnSP15Task.setNext(0);
    break;
  case 15:
    turnOnSP16Task.setNext(0);
    break;
  case 16:
    turnOnSP17Task.setNext(0);
    break;
  case 17:
    turnOnSP18Task.setNext(0);
    break;
  case 18:
    turnOnSP19Task.setNext(0);
    break;
  case 19:
    turnOnSP20Task.setNext(0);
    break;
  }
}

String prevCommand = "";
void controlRelay(String action, String Code, String RelayID)
{
  /*
  FORMAT : 
  action,code,relayID
  ON,WATER,1 -> will turn on water mode on SP with R1  
  ON,TREAT,1 -> will turn on treat mode on SP with R1
  ON,SCHED,7 -> will turn on schedule mode by 07:00
  ON,SCHED,17 -> will turn on schedule mode by 17:00

  OFF,WATER,1 -> will turn OFF water mode on SP with R1  
  OFF,TREAT,1 -> will turn OFF treat mode on SP with R1
  */
  String incomingCMD = action + Code + RelayID; //
  int relayIdx = RelayID.toInt() - 1;
  if (incomingCMD != prevCommand)
  {
    if (action == "ON")
    {
      isOnMode = true;
      if (Code == "WATER")
      {
        isWaterMode = true;
        digitalWrite(solenoidPrimerPins[relayIdx], LOW); //ON
        waterMode();

        // call task
        startTaskByID(relayIdx);
      }
      else if (Code == "TREAT")
      {
        isTreatMode = true;
        digitalWrite(solenoidPrimerPins[relayIdx], LOW); //ON
        treatMode();

        // call task
        startTaskByID(relayIdx);
      }
      else if (!isWaterModeSchedule && Code == "SCHED")
      {

        // relayID become hours
        int hours = relayIdx + 1;
        if (hours == 7 || hours == 17 || hours == 23)
        {
          // schedule mode
          Serial.println("Water mode scheduler start..");
          isWaterModeSchedule = true; // set true water schedule;

          turnOnSP1Task.setNext(0); // start first schedule
        }
      }
    }
    else if (action == "OFF")
    {
      isOffMode, isOnMode = true;
      if (Code == "WATER")
      {
        isWaterMode = false;
        digitalWrite(solenoidPrimerPins[relayIdx], HIGH); //OFF
        waterMode();
      }
      else if (Code == "TREAT")
      {
        isTreatMode = false;
        digitalWrite(solenoidPrimerPins[relayIdx], HIGH); //OFF
        treatMode();
      }
      else
      {
        offMode();
      }
    }
    prevCommand = incomingCMD;
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
  char responseChar[14];
  response.toCharArray(responseChar, 14);

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
  if (rows == 3)
  {
    Serial.println("Action:" + receivedData[0]);
    Serial.println("Code: " + receivedData[1]);
    Serial.println("Solenoid: " + receivedData[2]);

    controlRelay(receivedData[0], receivedData[1], receivedData[2]);
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

// will return true if relay for water is ON
bool checkWaterModeON()
{
  int relayStatus = digitalRead(RSolenoidWater); // if 0 then return true [ON], 1 return false [OFF]
  if (relayStatus == 0)
  {
    Serial.println("Solenoid Water : ON");
    return true;
  }
  return false;
}

// will return true if relay for water is ON
bool checkTreatModeON()
{
  int relayStatus = digitalRead(RSolenoidTreat);
  if (relayStatus == 0)
  {
    Serial.println("Solenoid Treatmenr : ON");
    return true;
  }
  return false;
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
unsigned long intervalDEBUG = 1000;     // the time we need to wait
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
    for (int i = 0; i < 12; i++)
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

  // Schedule Task
  SchedBase::dispatcher(); // dispatch any tasks due

  delay(500);
}

void turnOnSP1()
{
  Serial.println("Start task 1");
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  // water mode true and relay treat mode is OFF
  {
    if (soilHumidityCode[0] == 'L')
    {
      Serial.println("[Schedule] Watering SP1");
      digitalWrite(solenoidPrimerPins[0], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP1Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP1..");
      turnOffSP1Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP1");
    turnOffSP1Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP1");
    turnOffSP1Task.setNext(WATERINGMANUALDURATION);
  }
}

void turnOffSP1()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP2Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP1");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP2()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[1] == 'L')
    {
      Serial.println("Watering SP2");
      digitalWrite(solenoidPrimerPins[1], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP2Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP2..");
      turnOffSP2Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP2");
    turnOffSP2Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP2");
    turnOffSP2Task.setNext(WATERINGMANUALDURATION);
  }
}

void turnOffSP2()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP3Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP2");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP3()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[2] == 'L')
    {
      Serial.println("Watering SP3");
      digitalWrite(solenoidPrimerPins[2], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP3Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP3..");
      turnOffSP3Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP3");
    turnOffSP3Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP3");
    turnOffSP3Task.setNext(WATERINGMANUALDURATION);
  }
}

void turnOffSP3()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP4Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP3");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP4()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[3] == 'L')
    {
      Serial.println("Watering SP4");
      digitalWrite(solenoidPrimerPins[3], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP4Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP4..");
      turnOffSP4Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP4");
    turnOffSP4Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP4");
    turnOffSP4Task.setNext(WATERINGMANUALDURATION);
  }
}

void turnOffSP4()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP5Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP4");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP5()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[4] == 'L')
    {
      Serial.println("Watering SP5");
      digitalWrite(solenoidPrimerPins[4], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP5Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP5..");
      turnOffSP5Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP5");
    turnOffSP5Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP5");
    turnOffSP5Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP5()
{

  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP6Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP5");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP6()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[5] == 'L')
    {
      Serial.println("Watering SP6");
      digitalWrite(solenoidPrimerPins[5], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP6Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP6..");
      turnOffSP6Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP6");
    turnOffSP6Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP6");
    turnOffSP6Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP6()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP7Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP6");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP7()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[6] == 'L')
    {
      Serial.println("Watering SP7");
      digitalWrite(solenoidPrimerPins[6], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP7Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP7..");
      turnOffSP7Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP7");
    turnOffSP7Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP7");
    turnOffSP7Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP7()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP8Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP7");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP8()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[7] == 'L')
    {
      Serial.println("Watering SP8");
      digitalWrite(solenoidPrimerPins[7], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP8Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP8..");
      turnOffSP8Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP8");
    turnOffSP8Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP8");
    turnOffSP8Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP8()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP9Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP8");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP9()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[8] == 'L')
    {
      Serial.println("Watering SP9");
      digitalWrite(solenoidPrimerPins[8], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP9Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP9..");
      turnOffSP9Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP9");
    turnOffSP9Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP9");
    turnOffSP9Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP9()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP10Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP9");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP10()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[9] == 'L')
    {
      Serial.println("Watering SP10");
      digitalWrite(solenoidPrimerPins[9], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP10Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP10..");
      turnOffSP10Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP10");
    turnOffSP10Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP10");
    turnOffSP10Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP10()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP11Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP10");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP11()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[10] == 'L')
    {
      Serial.println("Watering SP11");
      digitalWrite(solenoidPrimerPins[10], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP11Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP11..");
      turnOffSP11Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP11");
    turnOffSP11Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP11");
    turnOffSP11Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP11()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP12Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP11");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP12()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[11] == 'L')
    {
      Serial.println("Watering SP12");
      digitalWrite(solenoidPrimerPins[11], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP12Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP12..");
      turnOffSP12Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP12");
    turnOffSP12Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP12");
    turnOffSP12Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP12()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP13Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn Off SP12");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP13()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[12] == 'L')
    {
      Serial.println("Watering SP13");
      digitalWrite(solenoidPrimerPins[12], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP13Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP13..");
      turnOffSP13Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP13");
    turnOffSP13Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP13");
    turnOffSP13Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP13()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP14Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP13");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP14()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[13] == 'L')
    {
      Serial.println("Watering SP14");
      digitalWrite(solenoidPrimerPins[13], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP14Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP14..");
      turnOffSP14Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP14");
    turnOffSP14Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP14");
    turnOffSP14Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP14()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP15Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP14");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP15()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[14] == 'L')
    {
      Serial.println("Watering SP15");
      digitalWrite(solenoidPrimerPins[14], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP15Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP15..");
      turnOffSP15Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP15");
    turnOffSP15Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP15");
    turnOffSP15Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP15()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP16Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP16");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP16()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[15] == 'L')
    {
      Serial.println("Watering SP16");
      digitalWrite(solenoidPrimerPins[15], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP16Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP16..");
      turnOffSP16Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP16");
    turnOffSP16Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP16");
    turnOffSP16Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP16()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP17Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP16");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP17()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[16] == 'L')
    {
      Serial.println("Watering SP17");
      digitalWrite(solenoidPrimerPins[16], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP17Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP17..");
      turnOffSP17Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP17");
    turnOffSP17Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP17");
    turnOffSP17Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP17()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP18Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP17");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP18()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[17] == 'L')
    {
      Serial.println("Watering SP18");
      digitalWrite(solenoidPrimerPins[17], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP18Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP18..");
      turnOffSP18Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP18");
    turnOffSP18Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP18");
    turnOffSP18Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP18()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP19Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP18");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP19()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[18] == 'L')
    {
      Serial.println("Watering SP19");
      digitalWrite(solenoidPrimerPins[18], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP19Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP19..");
      turnOffSP19Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP19");
    turnOffSP19Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP19");
    turnOffSP19Task.setNext(WATERINGMANUALDURATION);
  }
}
void turnOffSP19()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    offMode();
    turnOnSP20Task.setNext(0);
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP19");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}

void turnOnSP20()
{
  // check isWateringModeON is true and soil is low
  if (isWaterModeSchedule)
  {
    if (soilHumidityCode[19] == 'L')
    {
      Serial.println("Watering SP20");
      digitalWrite(solenoidPrimerPins[19], LOW); //ON
      waterMode();

      // next task after WATERINGDURATION
      turnOffSP20Task.setNext(WATERINGDURATION);
    }
    else
    {
      // next task no wait
      Serial.println("Skip, watering SP20..");
      turnOffSP20Task.setNext(0);
    }
  }

  // manual mode for wateing
  if (isWaterMode)
  {
    // water mode true and relay treat mode is OFF
    Serial.println("[Manual] Watering SP20");
    turnOffSP20Task.setNext(WATERINGMANUALDURATION);
  }

  // manual mode for treatment
  if (isTreatMode)
  {
    // treat mode true and relay water mode is OFF
    Serial.println("[Manual] Treatment SP20");
    turnOffSP20Task.setNext(WATERINGMANUALDURATION);
  }
}

void turnOffSP20()
{
  if (isWaterModeSchedule)
  {
    // next sequence
    Serial.println("Finished watering on schedule.");
    offMode();
  }
  else
  {
    // finished
    Serial.println("Turn OFF SP20");
    isWaterMode = false;
    isTreatMode = false;
    offMode();
  }
}