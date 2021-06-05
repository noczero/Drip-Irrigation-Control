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
#define ssid "TP-Link_17D0"
#define password "32118281123"

// datetime for automation
Timezone IDTime;
String dateTimeNow = "";


// decalare variable used
int soilHumidityValue[20]; // array length 20
float phValue[12]; // ph length 12
boolean solenoidPrimerStatus[20];
boolean waterPumpStatus, solenoidWaterStatus, solenoidTreatStatus;

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
unsigned long intervalSendDataFirebase = 1000;    // the time we need to wait
unsigned long previousMillisSendDataFirebase = 0; // millis() returns an unsigned long.
void sendDataToFirebase(){
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (1000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisSendDataFirebase) >= intervalSendDataFirebase)
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
      String key = "TA" + String(i+1);
      json.add(key, soilHumidityValue[i]);
    }
    // bulk update node
    boolean status_soil = Firebase.updateNode(firebaseData, node_path + "/soil_humidity", json);
    
    json.clear(); // reset json

    // update ph value
    for (int i = 0; i < 20; i++)
    {
      /* code */
      String key = "TA" + String(i+1);
      json.add(key, phValue[i]);
    }
    // bulk update node
    boolean status_ph = Firebase.updateNode(firebaseData, node_path + "/ph", json);

    
    json.clear(); // reset json

    // update solenoid value
    for (int i = 0; i < 20; i++)
    {
      /* code */
      String key = "TA" + String(i+1);
      json.add(key, solenoidPrimerStatus[i]);
    }

    // rest of solenoid
    json.add("solenoid_water" , solenoidWaterStatus );
    json.add("solenoid_treat" , solenoidTreatStatus );
    json.add("water_pump" , waterPumpStatus );
    
    // bulk update node
    boolean status_solenoid = Firebase.updateNode(firebaseData, node_path + "/solenoid", json);

  
    if (status_ph && status_solenoid && status_soil && status_temperature && status_humidity){
      Serial.println("Success, update to firebase");
    }

  }
}

// read JSON
unsigned long intervalReadSerial = 500;    // the time we need to wait
unsigned long previousMillisReadSerial = 0; // millis() returns an unsigned long.
void readSerialDataReceived()
{
  unsigned long currentMillis = millis(); // grab current time
  // check if "interval" time has passed (200 milliseconds)
  if ((unsigned long)(currentMillis - previousMillisReadSerial) >= intervalReadSerial)
  {
    previousMillisReadSerial = millis(); // grab current time as previous millis

    StaticJsonDocument<1024> doc;
    //JsonObject root = doc.parseObject(softSerial);

    auto error = deserializeJson(doc, softSerial);
    if (error)
    {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(error.c_str());
      return;
    }
 
    //String relay_status = doc["solenoid"];
    String header = doc["header"];
    if (header.isEmpty() || header == NULL || header == "null") {
      return;
    } else {
      Serial.println(header);
      Serial.println("JSON received and parsed");
    }

    // parse soil humidty
    for (int i = 0; i < 20; i++)
    {
      /* code */
      soilHumidityValue[i] = doc["soil_humidity"][i];
    }
    
    
    // parse ph value
    for (int i = 0; i < 20; i++)
    {
      /* code */
      phValue[i] = doc["ph"][i];
    }

    // parse solenoid primer
    
    // parse soil humidty
    for (int i = 0; i < 20; i++)
    {
      /* code */
      solenoidPrimerStatus[i] = doc["solenoid"][i];
    }

    // the rest of solenoid
    waterPumpStatus = doc["solenoid"][22];
    solenoidTreatStatus = doc["solenoid"][21];
    solenoidWaterStatus = doc["solenoid"][20];

  
   
  }
}

// reeived


// debug mode, print information to serial
unsigned long intervalDEBUG = 1000;     // the time we need to wait
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
  debug();

  // send data to firebase
  sendDataToFirebase();

  // eztime
  // events(); 

  delay(10);
}