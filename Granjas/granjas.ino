#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>
#include "DHT.h" // Include DHT library

#include <HardwareSerial.h>
#include <TinyGPS++.h>

#define DHT_PIN 22     // Defines pin number to which the sensor is connected
#define DHT_TYPE DHT11 // Defines the sensor type. It can be DHT11 or DHT22

#define RX_PIN 16 // Pinout RX of ESP32
#define TX_PIN 17 // Pinout TX of ESP32
#define REFRESH_RATE 5000 // Defined in miliseconds

HardwareSerial SerialGPS(1);

TinyGPSPlus gps;

DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor


// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "***";
const char *WIFI_PASSWORD = "*****";
char macAddress[18];


//const char *MQTT_BROKER_IP = "iiot-upc.gleeze.com";
const char *MQTT_BROKER_IP = "linvirtualazure01.northeurope.cloudapp.azure.com"; //23.100.54.173
const int MQTT_PORT = 1883;
//const char *MQTT_USER = "iiot-upc";
//const char *MQTT_PASSWORD = "cim2020";
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";
const bool RETAINED = true;



float randNumberLong;
float randNumberLat;
int Latido;
int LitroLeche;
int walking;

static float humidity;
static float temperature;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);


void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");

  mqttClient.setServer(MQTT_BROKER_IP,
                       MQTT_PORT); // Connect the configured mqtt broker

  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker

  dhtSensor.begin(); // Starts sensor communication

  SerialGPS.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Starts gps communication with UART

  TimerHandle_t xTimer = xTimerCreate("printGpsReadings", REFRESH_RATE, pdTRUE, (void *) 0, printGpsReadings);
  xTimerStart(xTimer, 0);
  
}

void loop() {
  checkConnections(); // We check the connection every time

  // Publish every 2 seconds
  static int nowTime = millis();
  static int startTime = 0;
  static int elapsedTime = 0;
  nowTime = millis();
  elapsedTime = nowTime - startTime;

  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
  
  if (elapsedTime >= 2000) {
    //publishIntNumber();   // Publishes an int number
    //publishFloatNumber(); // Publishes a float number
    //publishString();      // Publishes string
    publishSmallJsonVaca();   // Publishes a small json
    publishSmallJsonGranja();   // Publishes a small json
    //publishBigJson();     // Publishes a big json
    publishTemperature(); //Publishes temp
    publishHumidity();   //Publishes Hum
    //publishGPS();   //Publishes GPS
    startTime = nowTime;
  }
}

/* Additional functions */
void publishIntNumber() {
  static int counter = 0;
  static const String topicStr = createTopic("int_number");
  static const char *topic = topicStr.c_str();

  counter++;

  mqttClient.publish(topic, String(counter).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(counter));
}

void publishFloatNumber() {
  static float counter = 0;
  static const String topicStr = createTopic("float_number");
  static const char *topic = topicStr.c_str();

  counter = counter + 0.1;

  mqttClient.publish(topic, String(counter).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(counter));
}

void publishTemperature() {

  static const String topicStr = createTopic("temperature");
  static const char *topic = topicStr.c_str();   
   
  temperature = dhtSensor.readTemperature(); // Reads the temperature, it takes
                                             // about 250 milliseconds

                                             


  mqttClient.publish(topic, String(temperature).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(temperature));

  delay(1000); // Freezes the loop for 1000 milliseconds

                                  
  //Serial.println("Temperature: " + String(temperature) + "°C"); // Prints in a new line the result                                     

  //Serial.println("Humidity: " + String(humidity) +" %"); // Prints in a new line the result

 
}

void publishHumidity() {

  static const String topicStr = createTopic("humidity");
  static const char *topic = topicStr.c_str();   
   // Variable that will store the last humidity value
  humidity = dhtSensor.readHumidity(); // Reads the humidity, it takes about 250
                                       // milliseconds
                                           


  mqttClient.publish(topic, String(humidity).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(humidity));

  delay(1000); // Freezes the loop for 1000 milliseconds

                                  
  //Serial.println("Temperature: " + String(temperature) + "°C"); // Prints in a new line the result                                     

  //Serial.println("Humidity: " + String(humidity) +" %"); // Prints in a new line the result

 
}


void publishString() {
  static int counter = 0;
  static const String topicStr = createTopic("string");
  static const char *topic = topicStr.c_str();

  counter++;
  String text = "Rai " + String(counter);

  mqttClient.publish(topic, text.c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + text);
}

void publishSmallJsonVaca() {
  static const String topicStr = createTopic("Vaca");
  static const char *topic = topicStr.c_str();

  StaticJsonDocument<256> doc; // Create JSON document of 128 bytes
  char buffer[256]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  JsonObject values1 =
      doc.createNestedObject("values1"); // We can add another Object
  
  randNumberLong = random(4000, 4100);
  randNumberLong  =randNumberLong/100;
  values1["long"] = randNumberLong;
  
  randNumberLat = random(30, 40);
  randNumberLat  =randNumberLat/100;
  values1["lat"] = randNumberLat;

  Latido = random(58, 72);
  values1["heart"] = Latido;

  walking = random(6, 10);
  values1["walk"] = walking;

  LitroLeche = random(1, 100);
  values1["milk"] = LitroLeche;

  values1["temperature"] = temperature;
  values1["humidity"] = humidity;
  
  // Serialize the JSON document to a buffer in order to publish it
  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

void publishSmallJsonGranja() {
  static const String topicStr = createTopic("Granja");
  static const char *topic = topicStr.c_str();

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  JsonObject values1 =
      doc.createNestedObject("values1"); // We can add another Object
  
  values1["temperature"] = temperature;
  values1["humidity"] = humidity;
  
  // Serialize the JSON document to a buffer in order to publish it
  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

void publishBigJson() {
  static const String topicStr = createTopic("big_json");
  static const char *topic = topicStr.c_str();

  DynamicJsonDocument doc(2048); // Create JSON document
  char buffer[2048]; // Create the buffer where we will print the JSON document
                     // to publish through MQTT

  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  JsonObject values1 = doc.createNestedObject("values1"); // Add another Object
  values1["t"] = 19.30;
  values1["h"] = 78;

  JsonArray values2 = doc.createNestedArray("values2"); // We can add an Array
  values2.add(1); // Inside the array we can add new values to "values"
  values2.add(2);
  values2.add(3);
  values2.add(4);
  values2.add(5);
  values2.add(6);
  values2.add(7);
  values2.add(8);
  values2.add(9);
  values2.add(10);
  values2.add(11);
  values2.add(12);

  // Serialize the JSON document to a buffer in order to publish it
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish_P(topic, buffer, n); // No RETAINED option
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

String createTopic(char *topic) {
  String topicStr = String(macAddress) + "/" + topic;
  return topicStr;
}

void printGpsReadings(TimerHandle_t xTimer){
  Serial.print("LAT=");   Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
  Serial.print("LONG=");  Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
  Serial.print("ALT=");   Serial.println(gps.altitude.meters());  // Altitude in meters (double)
  Serial.print("SATS=");  Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
  Serial.print("YEAR=");  Serial.println(gps.date.year()); // Year (2000+) (u16)
  Serial.print("MONTH="); Serial.println(gps.date.month()); // Month (1-12) (u8)
  Serial.print("DAY=");   Serial.println(gps.date.day()); // Day (1-31) (u8)
  Serial.print("HOUR=");  Serial.println(gps.time.hour()); // Hour (0-23) (u8)
  Serial.print("MIN=");   Serial.println(gps.time.minute()); // Minute (0-59) (u8)
  Serial.print("SEC=");   Serial.println(gps.time.second()); // Second (0-59) (u8)
}


void connectToWiFiNetwork() {
  Serial.print(
      "Connecting with Wi-Fi: " +
      String(WIFI_SSID)); // Print the network which you want to connect
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".."); // Connecting effect
  }
  Serial.print("..connected!  (ip: "); // After being connected to a network,
                                       // our ESP32 should have a IP
  Serial.print(WiFi.localIP());
  Serial.println(")");
  String macAddressStr = WiFi.macAddress().c_str();
  strcpy(macAddress, macAddressStr.c_str());
}

void connectToMqttBroker() {
  Serial.print(
      "Connecting with MQTT Broker:" +
      String(MQTT_BROKER_IP));    // Print the broker which you want to connect
  mqttClient.connect(macAddress, MQTT_USER, MQTT_PASSWORD);// Using unique mac address from ESP32
  while (!mqttClient.connected()) {
    delay(500);
    Serial.print("..");             // Connecting effect
    mqttClient.connect(macAddress); // Using unique mac address from ESP32
  }
  Serial.println("..connected! (ClientID: " + String(macAddress) + ")");
}

void checkConnections() {
  if (mqttClient.connected()) {
    mqttClient.loop();
  } else { // Try to reconnect
    Serial.println("Connection has been lost with MQTT Broker");
    if (WiFi.status() != WL_CONNECTED) { // Check wifi connection
      Serial.println("Connection has been lost with Wi-Fi");
      connectToWiFiNetwork(); // Reconnect Wifi
    }
    connectToMqttBroker(); // Reconnect Server MQTT Broker
  }
}
