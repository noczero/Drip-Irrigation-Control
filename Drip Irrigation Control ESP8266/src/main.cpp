#include "DHT.h"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <FirebaseESP8266.h>
#include <ESP8266WiFi.h>
#include <ezTime.h>


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
#define FIREBASE_AUTH  "AMudslGhcSwEfWhV79Mqpouzh50lqgJ7MzJb134w" // settings/service account/database secret
String node_path = "/device1";

// FirebaseESP8266 instance
FirebaseData firebaseData;
FirebaseJson json;

// wifi configuration 
#define ssid "kawung ece"
#define password "tuti17745"

// datetime for automation
Timezone IDTime;
String dateTimeNow = "";


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
void setupTime(){
  Serial.println("Setup NTP Time...");

  events(); // date
  waitForSync();
  IDTime.setLocation("Asia/Jakarta");
  dateTimeNow = IDTime.dateTime();
  Serial.println(dateTimeNow);
}

// setup for wifi
void setupWiFI() {
  // Let us connect to WiFi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(".......");
  Serial.println("WiFi Connected....IP Address:");
  Serial.println(WiFi.localIP());
}

// setup firebase
void setupFirebase(){
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");


}

// send data to firebase 
int soilHumidity[20]; // make array soil humidity, length is 20
int solenoidSensor[22];
int numberPHSensor[12];
void sendDataToFirebase(){
  // clear json
  json.clear();

  // update soil humidity
  for (int i = 0; i < 20; i++)
  {
    /* code */
    String key = "TA" + String(i+1);
    json.add(key, soilHumidity[i]);
  }
  // bulk update node
  boolean status_soil = Firebase.updateNode(firebaseData, node_path + "/soil_humidity", json);
  
  json.clear(); // reset json

}

// read JSON
void readSerialDataReceived()
{
  StaticJsonDocument<256> doc;
  //JsonObject root = doc.parseObject(softSerial);

  auto error = deserializeJson(doc, softSerial);
  if (error)
  {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(error.c_str());
    return;
  }
  Serial.println("JSON received and parsed");
  String relay_status = doc["relay_status"];

  Serial.println(relay_status);
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

    // execute if DEBUG mode is defined
    Serial.print("Humidity (%): ");
    Serial.print(humidity);
    Serial.print(", Temperature (C): ");
    Serial.print(temperature);

    Serial.println();
  }
#endif
}

void setup()
{
  delay(1000);
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
  // setupTime();

  // setup firebase
  setupFirebase();
}

void loop()
{

  // invoke read DHT
  readDHT22();

  // read data recevied serial
  readSerialDataReceived();

  // invoke debug
  //debug();

  // send data to firebase
 // sendDataToFirebase();

  // eztime
  // events(); 

  delay(10);
}