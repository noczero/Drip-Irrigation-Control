/* 
To do :
1. Read soil moist (done)
2. Read all ph (done)
3. Send data to esp8266 
4. Add logic for controling solenoid
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
static const uint8_t soilHumidityAnalogPins[4] = {A0,A1,A2,A3};
static const uint8_t phAnalogPins[12] = {A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};

// define relay pins as array
static const uint8_t solenoidPrimerPins[20] = {R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,R13,R14,R15,R16,R17,R18,R19,R20};


// array value
int soilHumidityValue[20]; // array length 20
float phValue[12]; // ph length 12
boolean solenoidPrimerStatus[20];

// relay command
int relay_command_id = -1; // set init value
String command_status = ""; // this value store command, available command water for watering, treat for treatment. 

I2CScanner scanner;

// ads115 instance
Adafruit_ADS1115 adsFirst, adsSecond, adsThird, adsFourth;     /* Use this for the 12-bit version */

// detect i2c device for ADS115
void i2cScanner(){
	scanner.Init();
	scanner.Scan();
}


// send data to esp8266
unsigned long intervalSendData = 1000; // the time we need to wait 2 minutes
unsigned long previousMillisSendData = 0; // millis() returns an unsigned long.
void sendDataToESP8266(){
	unsigned long currentMillis = millis(); // grab current time
	// check if "interval" time has passed (2000 milliseconds)
	if ((unsigned long)(currentMillis - previousMillisSendData) >= intervalSendData)
	{
		
		previousMillisSendData = millis(); // grab current time as previous millis

		DynamicJsonDocument doc(1024);

		// send solenoid status
		JsonArray solenoidJson = doc.createNestedArray("solenoid");
		for (int i = 0; i < 20; i++)
		{
			solenoidJson.add(solenoidPrimerStatus[i]);
		}
		solenoidJson.add(digitalRead(RSolenoidWater)); // index 20
		solenoidJson.add(digitalRead(RSolenoidTreat)); // index 21
		solenoidJson.add(digitalRead(RPump)); // index 22
		
		// send humidity value
		JsonArray soilHumidityJson = doc.createNestedArray("soil_humidity");
		for (int i = 0; i < 20; i++)
		{
			soilHumidityJson.add(soilHumidityValue[i]);
		}

		// send ph Value
		JsonArray phJson = doc.createNestedArray("ph");
		for (int i = 0; i < 12; i++)
		{
			phJson.add(phValue[i]);
		}
		

		// check Serial 1 available
		Serial.println("send...");
		doc["header"] = "OK"; // cek header
		serializeJson(doc, Serial1);
	}
}

// read data from esp8266
void readDataFromESP8266(){
	StaticJsonDocument<25> doc;

	auto error = deserializeJson(doc, Serial1);
	if (error)
	{
		Serial.print(F("deserializeJson() failed with code "));
		Serial.println(error.c_str());
		return;
	}
	
	String _command_status = doc["cmd_status"]; // water or treat
	command_status = _command_status; // set this

	relay_command_id = doc["relay_cmd_id"];
	
	Serial.println("JSON received and parsed");
	Serial.print("Turn On Relay with ID : ");
	Serial.println(relay_command_id);
}

void setupRelay(){
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


void setupADS1115(){
	adsFourth.begin(0x48);
	adsFirst.begin(0x4B);
	adsSecond.begin(0x4A);
	adsThird.begin(0x49);
}

// read ads
void readSoilHumidity(){	
	int adsCH = 0; // value 0,1,2,3 then reset


	// iterate over 20 soil humidity sensor, ADS and Analog
	for (int i = 0; i < 20; i++){
		
		// reset index channel
		if (adsCH % 4 == 0){
			adsCH = 0;
		}
		
		if ( i < 4) {
			// for first ads, range index 0-3
			soilHumidityValue[i] = adsFirst.readADC_SingleEnded(adsCH);
		} else if (i < 8) {
			// for second ads, range index 4-7
			soilHumidityValue[i] = adsSecond.readADC_SingleEnded(adsCH);
		} else if (i < 12) {
			// for third ads, range index 8-11
			soilHumidityValue[i] = adsThird.readADC_SingleEnded(adsCH);
		} else if (i < 16) {
			// for foruth ads, range index 12-15
			soilHumidityValue[i] = adsFourth.readADC_SingleEnded(adsCH);
		} else {
			// read analog PIN, range index 16-19
			soilHumidityValue[i] = analogRead(soilHumidityAnalogPins[adsCH]);
		}

		adsCH++;
	}
}

void readPH(){
	for (int i = 0; i < 12; i++){
		phValue[i] = analogRead(phAnalogPins[i]);
	}
}

void readRelay(){
	for (int i = 0; i < 20; i++){
		solenoidPrimerStatus[i] = digitalRead(solenoidPrimerPins[i]);
	}
}

unsigned long intervalMonitoring = 100;    // the time we need to wait
unsigned long previousMillisMonitoring = 0; // millis() returns an unsigned long.
void monitoring(){
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

		// send data
		//sendDataToESP8266();
	}
}


unsigned long intervalctrlR1 = 1000*60*2; // the time we need to wait 2 minutes
unsigned long previousMillisctrlR1 = 0; // millis() returns an unsigned long.
void ctrlR1(){
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
	while (!Serial) {};

	// Serial 1, for communication to esp8266
	Serial1.begin(9600);
	while(!Serial1) {};

	// Setup i2c scanner
	i2cScanner();

	// set all relay as output
	setupRelay();

	// setup ads
	setupADS1115();
}

void loop() 
{
	monitoring();

	sendDataToESP8266();

	//readDataFromESP8266();

	debug();
	
	delay(10);
}
