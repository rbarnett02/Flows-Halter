#include <HardwareSerial.h>
#include <TinyGPS++.h>

#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>
#include "DHT.h" // Include DHT library

#define RX_PIN 16 // Pinout RX of ESP32
#define TX_PIN 17 // Pinout TX of ESP32
#define REFRESH_RATE 5000 // Defined in miliseconds

HardwareSerial SerialGPS(1);

TinyGPSPlus gps;

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "AulaAutomatica";
const char *WIFI_PASSWORD = "ticsFcim";
char macAddress[18];

const char *MQTT_BROKER_IP = "iiot-upc.gleeze.com";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "iiot-upc";
const char *MQTT_PASSWORD = "cim2020";
const bool RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);


void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");

  
  mqttClient.setServer(MQTT_BROKER_IP,
                       MQTT_PORT); // Connect the configured mqtt broker

  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker
  
  SerialGPS.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Starts gps communication with UART

  TimerHandle_t xTimer = xTimerCreate("printGpsReadings", REFRESH_RATE, pdTRUE, (void *) 0, printGpsReadings);
  xTimerStart(xTimer, 0);
}

void loop() {
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
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

  static const String topicStr = createTopic("SEC");
  static const char *topic = topicStr.c_str();   
  static float SEC; // Variable that will store the last humidity value
  SEC = gps.time.second(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topic, String(SEC).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(SEC));

    static const String topicStrM = createTopic("MIN");
  static const char *topicM = topicStrM.c_str();   
  static float MIN; // Variable that will store the last humidity value
  MIN = gps.time.minute(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicM, String(MIN).c_str(), RETAINED);
  Serial.println(" <= " + String(topicM) + ": " + String(MIN));

    static const String topicStrH = createTopic("HOUR");
  static const char *topicH = topicStrH.c_str();   
  static float HOUR; // Variable that will store the last humidity value
  HOUR = gps.time.hour(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicH, String(HOUR).c_str(), RETAINED);
  Serial.println(" <= " + String(topicH) + ": " + String(HOUR));

      static const String topicStrY = createTopic("YEAR");
  static const char *topicY = topicStrY.c_str();   
  static float YEAR; // Variable that will store the last humidity value
  YEAR = gps.date.year(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicY, String(YEAR).c_str(), RETAINED);
  Serial.println(" <= " + String(topicY) + ": " + String(YEAR));

   static const String topicStrMH = createTopic("MONTH");
  static const char *topicMH = topicStrMH.c_str();   
  static float MONTH; // Variable that will store the last humidity value
  MONTH = gps.date.month(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicMH, String(MONTH).c_str(), RETAINED);
  Serial.println(" <= " + String(topicMH) + ": " + String(MONTH));

     static const String topicStrS = createTopic("SATS");
  static const char *topicS = topicStrS.c_str();   
  static float SATS; // Variable that will store the last humidity value
  SATS = gps.satellites.value(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicS, String(SATS).c_str(), RETAINED);
  Serial.println(" <= " + String(topicS) + ": " + String(SATS));

     static const String topicStrA = createTopic("ALT");
  static const char *topicA = topicStrA.c_str();   
  static float ALT; // Variable that will store the last humidity value
  ALT = gps.altitude.meters(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicA, String(ALT).c_str(), RETAINED);
  Serial.println(" <= " + String(topicA) + ": " + String(ALT));

       static const String topicStrL = createTopic("LONG");
  static const char *topicL = topicStrL.c_str();   
  static float LONG; // Variable that will store the last humidity value
  LONG = gps.location.lng(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicL, String(LONG).c_str(), RETAINED);
  Serial.println(" <= " + String(topicL) + ": " + String(LONG));

       static const String topicStrLA = createTopic("LAT");
  static const char *topicLA = topicStrLA.c_str();   
  static float LAT; // Variable that will store the last humidity value
  LAT = gps.location.lat(); // Reads the humidity, it takes about 250
                                       // milliseconds                                       
  mqttClient.publish(topicLA, String(LAT).c_str(), RETAINED);
  Serial.println(" <= " + String(topicLA) + ": " + String(LAT));
  
  delay(1000); // Freezes the loop for 1000 milliseconds


}

/* More GPS functions examples */
//  Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
//  Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
//  Serial.print(gps.location.rawLat().negative ? "-" : "+");
//  Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
//  Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
//  Serial.print(gps.location.rawLng().negative ? "-" : "+");
//  Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
//  Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
//  Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
//  Serial.println(gps.date.year()); // Year (2000+) (u16)
//  Serial.println(gps.date.month()); // Month (1-12) (u8)
//  Serial.println(gps.date.day()); // Day (1-31) (u8)
//  Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
//  Serial.println(gps.time.hour()); // Hour (0-23) (u8)
//  Serial.println(gps.time.minute()); // Minute (0-59) (u8)
//  Serial.println(gps.time.second()); // Second (0-59) (u8)
//  Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
//  Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
//  Serial.println(gps.speed.knots()); // Speed in knots (double)
//  Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
//  Serial.println(gps.speed.mps()); // Speed in meters per second (double)
//  Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
//  Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
//  Serial.println(gps.course.deg()); // Course in degrees (double)
//  Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
//  Serial.println(gps.altitude.meters()); // Altitude in meters (double)
//  Serial.println(gps.altitude.miles()); // Altitude in miles (double)
//  Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
//  Serial.println(gps.altitude.feet()); // Altitude in feet (double)
//  Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
//  Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)


String createTopic(char *topic) {
  String topicStr = String(macAddress) + "/" + topic;
  return topicStr;
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
