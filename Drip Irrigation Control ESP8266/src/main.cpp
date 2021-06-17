/* @note

  Change BUFFER_LENGTH to MAXIMUM length of data (136) Wire.h
  Change TWI_BUFFER_LENGTH also
  C:\Users\User\.platformio\packages\framework-arduinoespressif8266\libraries\Wire\Wire.cpp
  C:\Users\User\.platformio\packages\framework-arduinoespressif8266\cores\esp8266\twi.h
*/
#include "DHT.h"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <FirebaseESP8266.h>
#include <ESP8266WiFi.h>
#include <ezTime.h>
#include <Wire.h>
#include <SchedTask.h>			

#define DEBUG // uncomment to debug data

// declare pin
#define DHTPIN D7    // Label : D7
#define TX_serial D5 // to RX arduino
#define RX_serial D6 // to TX arduino

// declare library configuration
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

// instance & variable for DHT
DHT dht(DHTPIN, DHTTYPE);
float temperature, humidity = 0.0;

// configure software serial
SoftwareSerial softSerial(RX_serial, TX_serial);

// configure firebase
#define FIREBASE_HOST "https://drip-irrigation-control-default-rtdb.firebaseio.com/" // url
#define FIREBASE_AUTH "AMudslGhcSwEfWhV79Mqpouzh50lqgJ7MzJb134w"                     // settings/service account/database secret
String node_path = "/device1";

// FirebaseESP8266 instance
FirebaseData firebaseData;
FirebaseJson json;

// wifi configuration
#define ssid "NASTA BAWAH_plus"
#define password "Nasta2b40"

// datetime for automation
Timezone IDTime;
String dateTimeNow = "";
String wateringTimeHours[2] = {"07:00", "17:00"}; // Water time on 7 AM and 5 PM

// I2C ESP8266
#define I2CAddressMaster 0x08
#define pinSDA D1
#define pinSCL D2
#define RECEIVEBYTES 136
#define VALIDROWS 56

// decalare variable used
String soilHumidityValue[20]; // array length 20
int phValue[12];              // ph length 12
String solenoidPrimerStatus[20];
String waterPumpStatus, solenoidWaterStatus, solenoidTreatStatus;

// validation
bool isDataReceivedValid = false;

// mode
bool isWaterModeSchedule, isWaterMode, isTreatMode, isOfflineMode = false;

// Scheduler
void checkTime();
SchedTask checkTimeTask (0, 1000, checkTime); // every minutes check




// -- FUNCTION START --- 

void setupI2CCom()
{

  // Master
  Wire.begin(D1, D2);
  Wire.setTimeout(6000);
}

String cmdCode; // WATER/TREAT,1..20
void sendToSlave()
{
  //cmdCode = "WATER,1";
  //sprintf(commandData, commandFormat, cmdCode);

  // String to array
  char responseChar[13];
  cmdCode.toCharArray(responseChar, 13);

  Wire.beginTransmission(I2CAddressMaster); /* begin with device address 8 */
  Wire.write(responseChar);              /* sends hello string */
  Wire.endTransmission();                   /* stop transmitting */
}

String reverseSolenoidData(String c)
{
  if (c == "1")
  {
    return "OFF";
  }
  else
  {
    return "ON";
  }
}

String reverseSoilHumidityData(String c)
{
  if (c == "L")
  {
    return "LOW";
  }
  else if (c == "N")
  {
    return "NORMAL";
  }
  else
  {
    return "HIGH";
  }
}


void readFromSlave()
{
  int nBytes = Wire.requestFrom(I2CAddressMaster, RECEIVEBYTES); /* request & read data of size 13 from slave */

  String response = "";
  int i = 0;
  if (nBytes == RECEIVEBYTES)
  {
    while (Wire.available())
    {

      char b = Wire.read();
      response += b;
      i++;
    }
  }
  // Serial.println(response);
  // Serial.println(i);
  // Serial.println(nBytes);

  // String to array
  char responseChar[RECEIVEBYTES + 1];
  response.toCharArray(responseChar, RECEIVEBYTES + 1);

  // initialize first part (string, delimiter)
  char *ptr = strtok(responseChar, ",");
  int rows = 0;

  String receivedData[57];
  while (ptr != NULL)
  {
    //Serial.println(ptr);
    receivedData[rows] = ptr;
    // create next part
    ptr = strtok(NULL, ",");
    rows++;
  }

  // Serial.println(rows);

  // validation for parsing
  if (rows == VALIDROWS)
  {
    isDataReceivedValid = true;
    Serial.print("Valid : ");
    Serial.println(receivedData[0]);
    solenoidWaterStatus = reverseSolenoidData(receivedData[41]);
    solenoidTreatStatus = reverseSolenoidData(receivedData[42]);
    waterPumpStatus = reverseSolenoidData(receivedData[43]);

    int dataIndex = 0;
    for (int i = 1; i < 56; i++)
    {
      // INDEX 1 - 20 SOIL, Solenoid: 21 - 43 (41 [water],42[treat],43[pump]), pH : 44 - 45
      //Serial.print(i);
      //Serial.print("-");
      //Serial.println(receivedData[i]);

      //if (i == 0){
      //  continue;
      //}

      if (i == 21)
        dataIndex = 0;

      if (i == 44)
        dataIndex = 0;

      // parseing recevideData to each variable
      if (i > 0 && i <= 20)
      {
        soilHumidityValue[dataIndex] = reverseSoilHumidityData(receivedData[i]);
        //Serial.println(soilHumidityValue[dataIndex]);
      }
      else if (i > 20 && i <= 40)
      {
        solenoidPrimerStatus[dataIndex] = reverseSolenoidData(receivedData[i]);
        //Serial.println(solenoidPrimerStatus[dataIndex]);
      }
      else if (i == 41)
      {
        solenoidWaterStatus = reverseSolenoidData(receivedData[i]);
        //Serial.println(solenoidWaterStatus);
      }
      else if (i == 42)
      {
        solenoidTreatStatus = reverseSolenoidData(receivedData[i]);
        //Serial.println(solenoidTreatStatus);
      }
      else if (i == 43)
      {
        waterPumpStatus = reverseSolenoidData(receivedData[i]);
        //Serial.println(waterPumpStatus);
      }
      else if (i > 43 && i <= 55)
      {
        phValue[dataIndex] = receivedData[i].toInt();
        //Serial.println(phValue[dataIndex]);
      }

      dataIndex++;
    }
  }

  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);      // print the character
  }
  int x = Wire.read(); // receive byte as an integer
  Serial.println(x);   // print the integer
}

// Read DHT 22 with interval 2 seconds
unsigned long intervalDHT = 2000;    // the time we need to wait
unsigned long previousMillisDHT = 0; // millis() returns an unsigned long.
void readDHT22()
{
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (2000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisDHT) >= intervalDHT)
  {
    previousMillisDHT = millis(); // grab current time as previous millis

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float raw_humidity = dht.readHumidity();

    // Read temperature as Celsius (the default)
    float raw_temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    else
    {
      // if not NaN
      humidity = raw_humidity;
      temperature = raw_temperature;
    }
  }
}

// setup time
void setupTime()
{
  Serial.println("Setup NTP Time...");

  events(); // date
  waitForSync();
  IDTime.setLocation("Asia/Jakarta");
  dateTimeNow = IDTime.dateTime();
  Serial.println(dateTimeNow);
}

// setup for wifi
void setupWiFI()
{
  // Let us connect to WiFi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println(".......");
  Serial.println("WiFi Connected....IP Address:");
  Serial.println(WiFi.localIP());
}

// setup firebase
void setupFirebase()
{
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}

// send data to firebase
unsigned long intervalSendDataFirebase = 1000;    // the time we need to wait
unsigned long previousMillisSendDataFirebase = 0; // millis() returns an unsigned long.
void sendDataToFirebase()
{
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (1000 milliseconds)
  if (isDataReceivedValid && (unsigned long)(currentMillis - previousMillisSendDataFirebase) >= intervalSendDataFirebase)
  {
    previousMillisSendDataFirebase = millis(); // grab current time as previous millis

    // update temperature
    json.clear();
    json.add("temperature", temperature);
    boolean status_temperature = Firebase.updateNode(firebaseData, node_path, json);

    // update humidity
    json.clear();
    json.add("humidity", humidity);
    boolean status_humidity = Firebase.updateNode(firebaseData, node_path, json);

    // clear json
    json.clear();

    // update soil humidity
    for (int i = 0; i < 20; i++)
    {
      /* code */
      String key = "TA" + String(i + 1);
      json.add(key, soilHumidityValue[i]);
    }
    // bulk update node
    boolean status_soil = Firebase.updateNode(firebaseData, node_path + "/soil_humidity", json);

    json.clear(); // reset json

    // update ph value
    for (int i = 0; i < 12; i++)
    {
      /* code */
      String key = "TA" + String(i + 1);
      json.add(key, phValue[i]);
    }
    // bulk update node
    boolean status_ph = Firebase.updateNode(firebaseData, node_path + "/ph", json);

    json.clear(); // reset json

    // update solenoid value
    for (int i = 0; i < 20; i++)
    {
      /* code */
      String key = "TA" + String(i + 1);
      json.add(key, solenoidPrimerStatus[i]);
    }

    // rest of solenoid
    json.add("solenoid_water", solenoidWaterStatus);
    json.add("solenoid_treat", solenoidTreatStatus);
    json.add("water_pump", waterPumpStatus);

    // bulk update node
    boolean status_solenoid = Firebase.updateNode(firebaseData, node_path + "/solenoid", json);

    if (status_ph && status_solenoid && status_soil && status_temperature && status_humidity)
    {
      Serial.println("Success, update to firebase");
    }
  }
}


// send data to firebase
unsigned long intervalGetDataFromFirebase = 1000;    // the time we need to wait
unsigned long previousMillisGetDataFromFirebase = 0; // millis() returns an unsigned long.
void getDataFromFirebase()
{
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (1000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisGetDataFromFirebase) >= intervalGetDataFromFirebase)
  {
    previousMillisGetDataFromFirebase = millis(); // grab current time as previous millis

    if (Firebase.getString(firebaseData, node_path + "/CMD")){
      if (firebaseData.dataType() == "string") {
        cmdCode = firebaseData.stringData();
        Serial.println(cmdCode);
      }
    } 
  }
}


// debug mode, print information to serial
unsigned long intervalDEBUG = 1000;    // the time we need to wait
unsigned long previousMillisDEBUG = 0; // millis() returns an unsigned long.
void debug()
{
#ifdef DEBUG
  //sendDataToFirebase();
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (2000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisDEBUG) >= intervalDEBUG)
  {
    previousMillisDEBUG = millis(); // grab current time as previous millis

    // execute if DEBUG mode is defined
    Serial.print("Humidity (%): ");
    Serial.print(humidity);
    Serial.print(", Temperature (C): ");
    Serial.print(temperature);
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
    Serial.print(solenoidWaterStatus);

    Serial.print(", Solenoid Treat : ");
    Serial.print(solenoidTreatStatus);

    Serial.print(", Water Pump : ");
    Serial.print(waterPumpStatus);

    Serial.println();
  }

#endif
}

void setup()
{
  delay(1000);

  // setup i2c
  setupI2CCom();

  // setup serial default
  Serial.begin(9600);
  while (!Serial)
  {
  };
  Serial.println("ESP8266 Drip Irrigation Control Ver. 1.0");

  // setup software serial
  softSerial.begin(9600);
  while (!softSerial)
  {
  };

  // dht begin
  dht.begin();

  // setup WiFI
  setupWiFI();

  // setup Time
  setupTime();

  // setup firebase
  setupFirebase();
}

void loop()
{

  // invoke read DHT
  readDHT22();

  // invoke debug
  debug();

  // send data to firebase
  sendDataToFirebase();

  // get data from firebase
  getDataFromFirebase();

  // i2ccommunication
  sendToSlave();
  readFromSlave();

  // eztime
  events();


  // Schedule Task
  SchedBase::dispatcher();										// dispatch any tasks due

  delay(500);
}

void checkTime(){
  // check time
  String timeNow = IDTime.dateTime("H:i");  // 17:00, 18:00
  Serial.println(timeNow);
   
  // compare to array wateringTimeHours
  for (int i = 0; i < 2; i++ ){
    if (!isWaterModeSchedule && timeNow == wateringTimeHours[i]){
      Serial.println("On Time");
      isWaterModeSchedule = true; // set mode to true.
      // String to array
      String scheduleCmd = "ON,SCHED," + IDTime.dateTime("H");
      char responseChar[13];
      scheduleCmd.toCharArray(responseChar, 13);

      Wire.beginTransmission(I2CAddressMaster); /* begin with device address 8 */

      Wire.write(responseChar);              /* sends hello string */
      Wire.endTransmission();                   /* stop transmitting */
    
    }
  }
}

