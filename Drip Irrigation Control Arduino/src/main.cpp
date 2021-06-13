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
#include <EasyTransferI2C.h>

// #define DEBUG

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
static const uint8_t solenoidPrimerPins[23] = {R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,R13,R14,R15,R16,R17,R18,R19,R20,RSolenoidWater,RSolenoidTreat,RPump};


// array value
int soilHumidityValue[20]; // array length 20
char soilHumidityCode[20];
int phValue[12]; // ph length 12
boolean solenoidPrimerStatus[23];
char solenoidPrimerStatusCode[23];

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

// i2c data structure
struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int soilHumidityValue[20];
  int phValue[12];
  boolean solenoidPrimerStatus[20];
};

SEND_DATA_STRUCTURE myData;

//create object Easy Transfer
EasyTransferI2C ET; 

// i2c communication address
#define I2CAddressSlave 0x08
#define I2CADdressMaster 4


char dataSend[168];
char format[66] = "OK,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,FIN";
char formatpH[66] = "PH,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,FIN";
char formatSolenoid[66] = "SP,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,FIN";

// Soil, Solenoid, pH
char allDataFormat[168] = "OK,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d";

char formatString[100] = "OK, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s";

int sendID = 3;
void sendToMaster(){
  // Wire.write("OK,HIGH,LOW,LOW,LOW,HIGH,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW");  /*send string on request */
  //Wire.write("OK2,HIGH,LOW,LOW,LOW,HIGH,LOW,LOW,LOW");  /*send string on request */
  //Wire.beginTransmission(I2CAddressSlave);

  if (sendID == 0) {
    sprintf(dataSend, format, 
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
          soilHumidityCode[19]);
    
  } else if (sendID == 1) {
       sprintf(dataSend, formatpH, 
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
  } else if (sendID == 2) {
     sprintf(dataSend, formatSolenoid, 
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
          solenoidPrimerStatusCode[22]);
      
      sendID = -1;
  } else {
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
          phValue[11]
      );
  }

  // sendID++;
  
  //sprintf(dataSend, formatString, "L" , "L" , "L" , "L" , "L" , "L" , "L" , "L" ,  "L" ,"L" ,"L" ,
  //"L" , "L" , "L" , "L" , "L" , "L" ,  "L" ,"L" ,"L" );
  
  // Setup byte variable in the correct size
  //String test = "OK,L,L,L,L,L,L,L,L,L,H,H,H,H,h,H"; // 32 bytes
  //String test = "OK,L,L,L,L,L,L,L,L,L,H,H,H,H,h,H,OK,L,L,L,L,L,L,L,L,L,H,H,H,H,h,H"; // 65 bytes
  //String test = ""
  //int sizeResp = 65;
  //byte response[sizeResp];

  //Serial.println("Bytes " + strlen(test));
  
  // Format answer as array
  //for (byte i=0;i<sizeResp;i++) {
  //  response[i] = (byte)test.charAt(i);
  //}
  
  // Send response back to Master
  Wire.write(dataSend);
  Serial.println(dataSend);
  //String test = "OK, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L";
  // Wire.write("OK");
  // Wire.write("TEST");
  //Wire.endTransmission();
  // delay(10);
  // Wire.write("OK");

	/*
  myData.phValue[0] = 10;
  myData.soilHumidityValue[0] = 1;
  myData.solenoidPrimerStatus[0] = true;
  ET.sendData(I2CAddressSlave);
	*/
}

void readFromMaster(int howMany){
 while (0 <Wire.available()) {
    char c = Wire.read();      /* receive byte as a character */
    Serial.print(c);           /* print the character */
  }
 Serial.println();             /* to newline */
}


void setupI2CCom(){

	// Master
	//Wire.begin();

	// SLave
  	Wire.begin(I2CAddressSlave);
	Wire.setWireTimeout(6000, true);
  	Wire.onReceive(readFromMaster); /* register receive event */
	Wire.onRequest(sendToMaster); /* register request event */

  	//ET.begin(details(myData), &Wire);
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

// convert raw humidity to string
int percentageMoist = 0;
char convertSoilValueToText(int rawValue, boolean ADS){
  //    Convert RAW Analog to discrete (low, normal, high) for soil moist level
  if (ADS){
    // RAW Value ADS Module range 0-32767
    percentageMoist = map(rawValue, 0, 32767, 0, 100);

  } else {
    // using standar Analog, 0-1024
    percentageMoist = map(rawValue, 0, 1024, 0, 100);
  }

  if (25 < percentageMoist && percentageMoist <= 100) {
      return 'H'; // HIGH
  }else if (15 <= percentageMoist && percentageMoist <= 25) {
      return 'N'; // N
  } else if (0 < percentageMoist && percentageMoist < 15) {
      return 'L'; // L
  }
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

      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], true);
		} else if (i < 8) {
			// for second ads, range index 4-7
			soilHumidityValue[i] = adsSecond.readADC_SingleEnded(adsCH);

      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i],true);

		} else if (i < 12) {
			// for third ads, range index 8-11
			soilHumidityValue[i] = adsThird.readADC_SingleEnded(adsCH);
     
      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], true);

		} else if (i < 16) {
			// for foruth ads, range index 12-15
			soilHumidityValue[i] = adsFourth.readADC_SingleEnded(adsCH);
      
      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], true);

		} else {
			// read analog PIN, range index 16-19
			soilHumidityValue[i] = analogRead(soilHumidityAnalogPins[adsCH]);
      
      // convert to discrete
      soilHumidityCode[i] = convertSoilValueToText(soilHumidityValue[i], false);
		}

		adsCH++;
	}
}

void readPH(){
	for (int i = 0; i < 12; i++){
		phValue[i] = analogRead(phAnalogPins[i]);
	}
}


char convertRelayToTxt(int relayStatus){
    if (relayStatus) {
      return '1'; // HIGH
    } else {
      return '0'; // LOW
    }
}

void readRelay(){
	for (int i = 0; i < 23; i++){
		int relayStatus = digitalRead(solenoidPrimerPins[i]);
    solenoidPrimerStatus[i] = relayStatus;
    solenoidPrimerStatusCode[i] = convertRelayToTxt(relayStatus);
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
		//Wire.write("[0,0,0,0,0,0,0,0,0,0,0,0]");
		// sendToSlave();
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

	// SETUP I2C COMMUNICATION
	setupI2CCom();
}

void loop() 
{
	monitoring();

	//sendDataToESP8266();

	//readDataFromESP8266();

	debug();
	
	delay(500);
}
