#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>  // https://arduinojson.org/
#include <Wire.h>
#include "DHT.h" // Include DHT library

// Hook up HC-SR04 with Trig to Arduino Pin 12, Echo to Arduino pin 13
#define trigPin 13
#define echoPin 12

#define DHT_PIN 22     // Defines pin number to which the sensor is connected
#define DHT_TYPE DHT11 // Defines the sensor type. It can be DHT11 or DHT22

DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor dht

// float parameters are duration, distance and level
float duration, distance, level, temperature;

const char* ssid = "MiFibra-CEF0";
const char* password = "tMkR7SKw";

//const char *MQTT_BROKER_IP = "10.20.60.5";
const char *MQTT_BROKER_IP = "iiot-upc.gleeze.com";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "iiot-upc";
const char *MQTT_PASSWORD = "cim2020";
const bool RETAINED = true;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
char macAddress[18];

static const String topicStr;
static const char *topic;
  
static const String topicStrT;
static const char *topicT;
  
long lastMsg = 0;
char msg[50];
int value = 0;

long lastMsgT = 0;
char msgT[50];
int valueT = 0;

// wifi and ultrasonic setup
void setup() {
    Serial.begin (115200);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    delay(10);
    dhtSensor.begin();
    delay(10);
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
    mqttClient.setServer(MQTT_BROKER_IP, 1883);
    mqttClient.setCallback(callback);
}
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, HIGH);   // on-line led status
  }

}
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(),MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //mqttClient.publish("fuelmonitor", "device on-line");
      // ... and resubscribe
      mqttClient.subscribe("fuelreaction");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
   if (!mqttClient.connected()) {
   reconnect();
  }
  mqttClient.loop();
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    char buffer[12];
    dtostrf(level, 7, 3, buffer);
    snprintf (msg, 50, buffer, value);
    Serial.print("Level: ");
    Serial.println(msg);

  static const String topicStr = createTopic("Camion1");
  static const char *topic = topicStr.c_str();
  //mqttClient.publish(topic, msg);
  
  ++valueT;
  char bufferT[12];
  dtostrf(temperature, 7, 3, bufferT);
  snprintf (msgT, 50, bufferT, valueT);
  temperature = dhtSensor.readTemperature(); // Reads the temperature, it takes
                                            // about 250 milliseconds
  Serial.println("Temperature: " + String(msgT) + "°C"); // Prints in a new line the result
  //static const String topicStrT = createTopic("Temp");
  //static const char *topicT = topicStrT.c_str();
  //mqttClient.publish(topic, msgT);


  static const String topicStrJ = createTopic("Camion1");
  static const char *topicJ = topicStrJ.c_str();
  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char bufferJ[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["t"] = msgT; // Add names and values to the JSON document
  doc["l"] = msg;
  serializeJson(doc, bufferJ);
  mqttClient.publish(topic, bufferJ, RETAINED);
   
  }

 
  // Write a pulse to the HC-SR04 Trigger Pin
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the response from the HC-SR04 Echo Pin
 
  duration = pulseIn(echoPin, HIGH);
  
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  
  distance = (duration / 2) * 0.0343;
  
  // Send results to Serial Monitor

  Serial.print("Distance = ");
  if (distance >= 144 || distance <= 0) {
     level = 99999 ;
     Serial.println("/");
     delay(500);
  }
  else 
  if(  (distance > 141) && (distance <= 143)   ) 
{
  level = 0;
  Serial.print ("0"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 136) && (distance <= 140)   ) 
{
  level = 40;
  Serial.print ("40"); 
  Serial.println("L");
  delay(500);
  
}  else
   if(  (distance > 131) && (distance <= 135)   ) 
{
  level = 112;
  Serial.print ("112"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 126) && (distance <= 130)   ) 
{
  level = 204;
  Serial.print ("204"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 121) && (distance <= 125)   ) 
{
  level = 309;
  Serial.print ("309"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 116) && (distance <= 120)   ) 
{
  level = 426;
  Serial.print ("426"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 111) && (distance <= 115)   ) 
{
  level = 553;
  Serial.print ("553"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 106) && (distance <= 110)   ) 
{
  level = 686;
  Serial.print ("686"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 101) && (distance <= 105)   ) 
{
  level = 826;
  Serial.print ("826"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 96) && (distance <= 100)   ) 
{
  level = 970;
  Serial.print ("970"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 91) && (distance <= 95)   ) 
{
  level = 1118;
  Serial.print ("1118"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 86) && (distance <= 90)   ) 
{
  level = 1269;
  Serial.print ("1269"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 81) && (distance <= 85)   ) 
{
  level = 1421;
  Serial.print ("1421"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 76) && (distance <= 80)   ) 
{
  level = 1573;
  Serial.print ("1573"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 71) && (distance <= 75)   ) 
{
  level = 1725;
  Serial.print ("1725"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 66) && (distance <= 70)   ) 
{
  level = 1876;
  Serial.print ("1876"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 61) && (distance <= 65)   ) 
{
  level = 2024;
  Serial.print ("2024"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 56) && (distance <= 60)   ) 
{
  level = 2138;
  Serial.print ("2138"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 51) && (distance <= 55)   ) 
{
  level = 2308;
  Serial.print ("2308"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 46) && (distance <= 50)   ) 
{
  level = 2442;
  Serial.print ("2442"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 41) && (distance <= 45)   ) 
{
  level = 2568;
  Serial.print ("2568"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 36) && (distance <= 40)   ) 
{
  level = 2685;
  Serial.print ("2685"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 31) && (distance <= 35)   ) 
{
  level = 2791;
  Serial.print ("2791"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 26) && (distance <= 30)   ) 
{
  level = 2882;
  Serial.print ("2882"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 21) && (distance <= 25)   ) 
{
  level = 2954;
  Serial.print ("2954"); 
  Serial.println("L");
  delay(500);
  
} else
   if(  (distance > 1) && (distance <= 20)   ) 
{
  level = 3000;
  Serial.print ("3000"); 
  Serial.println("L");
  delay(500);
}
}

String createTopic(char *topic) {
  String topicStr = String(macAddress) + "/" + topic;
  return topicStr;
}
