#include "DHT.h" // Libreria para el sensor de temperatura
#include <ArduinoJson.h>  //Libreria para Json
#include <PubSubClient.h> // Libreria para MQTT
#include <WiFi.h> //Libreria para Wifi
#include <Wire.h> //
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#define RX_PIN 16 // Pinout RX of ESP32
#define TX_PIN 17 // Pinout TX of ESP32
#define REFRESH_RATE 5000 // Defined in miliseconds

// Configuramos los pines del sensor Trigger y Echo
const int PinTrig = 13;
const int PinEcho = 12;

// Configuramos los pines del sensor de temperatura DHT11
#define DHT_PIN 22    
#define DHT_TYPE DHT11 

DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor dht

// Constante velocidad sonido en cm/s
const float VelSon = 34000.0;

// Número de muestras
const int numLecturas = 100;

// Distancia a los 10000 l y vacío para el sensor de nivel
const float distancia10000 = 2.15;
const float distanciaVacio = 11.41;

float lecturas[numLecturas]; // Array para almacenar lecturas para el sensor de nivel
int lecturaActual = 0; // Lectura por la que vamos para el sensor de nivel
float total = 0; // Total de las que llevamos para el sensor de nivel
float media = 0; // Media de las medidas para el sensor de nivel
bool primeraMedia = false; // Para saber que ya hemos calculado por lo menos una
float distanciaLleno = 0;
float cantidadLiquido = 0;

 
//Temperatura para el sensor DHT11
static float temperatura;

//Variables del GPS
static float LAT_GPS;
static float LONG_GPS;

HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

// Configuración del Wifi
const char *WIFI_SSID = "***";
const char *WIFI_PASSWORD = "****";
char macAddress[18];

//const char *MQTT_BROKER_IP = "iiot-upc.gleeze.com";
const char *MQTT_BROKER_IP = "linvirtualazure01.northeurope.cloudapp.azure.com"; //23.100.54.173
const int MQTT_PORT = 1883;
//const char *MQTT_USER = "iiot-upc";
//const char *MQTT_PASSWORD = "cim2020";
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "";
const bool RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup()
{
  // Iniciamos el monitor serie para mostrar el resultado
  Serial.begin(9600);
  // Ponemos el pin Trig en modo salida del sensor de nivel
  pinMode(PinTrig, OUTPUT);
  // Ponemos el pin Echo en modo entrada del sensor de nivel
  pinMode(PinEcho, INPUT);

  // Inicializamos el array del sensor de nivel
  for (int i = 0; i < numLecturas; i++)
  {
    lecturas[i] = 0;
  }

  //Inicio de las comunicaciones del sensor de temperatura
  dhtSensor.begin(); // Starts sensor communication

  //Configuración del GPS, el RX y TX del GPS van cruzados con el ESP32, paciencia porque tarda
  SerialGPS.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Starts gps communication with UART

  TimerHandle_t xTimer = xTimerCreate("printGpsReadings", REFRESH_RATE, pdTRUE, (void *) 0, printGpsReadings);
  xTimerStart(xTimer, 0);
  
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT); // Conectar el broker mqtt configurado

  connectToWiFiNetwork(); // Se conecta a la red configurada
  connectToMqttBroker();  // Se conecta al broker mqtt configurado
 
   
}
void loop()
{
  checkConnections(); // Comprobamos la conexión cada vez

 if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
  
 // Publicar cada 2 segundos
  static int nowTime = millis();
  static int startTime = 0;
  static int elapsedTime = 0;
  nowTime = millis();
  elapsedTime = nowTime - startTime;
  if (elapsedTime >= 2000) {
    LecturaNivel();
    LecturaTemperatura();
    publishSmallJson();   // Publica un pequeño json
    startTime = nowTime;
  }
  

 
}

// Método que inicia la secuencia del Trigger para comenzar a medir
void iniciarTrigger()
{
  // Ponemos el Triiger en estado bajo y esperamos 2 ms
  digitalWrite(PinTrig, LOW);
  delayMicroseconds(2);

  // Ponemos el pin Trigger a estado alto y esperamos 10 ms
  digitalWrite(PinTrig, HIGH);
  delayMicroseconds(10);

  // Comenzamos poniendo el pin Trigger en estado bajo
  digitalWrite(PinTrig, LOW);
}

void LecturaNivel() {

  // Eliminamos la última medida
  total = total - lecturas[lecturaActual];

  iniciarTrigger();

  // La función pulseIn obtiene el tiempo que tarda en cambiar entre estados, en este caso a HIGH
  unsigned long tiempo = pulseIn(PinEcho, HIGH);

  // Obtenemos la distancia en cm, hay que convertir el tiempo en segudos ya que está en microsegundos
  // por eso se multiplica por 0.000001
  float distancia = tiempo * 0.000001 * VelSon / 2.0;

  // Almacenamos la distancia en el array
  lecturas[lecturaActual] = distancia;

  // Añadimos la lectura al total
  total = total + lecturas[lecturaActual];

  // Avanzamos a la siguiente posición del array
  lecturaActual = lecturaActual + 1;

  // Comprobamos si hemos llegado al final del array
  if (lecturaActual >= numLecturas)
  {
    primeraMedia = true;
    lecturaActual = 0;
  }

  // Calculamos la media
  media = total / numLecturas;

  // Solo mostramos si hemos calculado por lo menos una media
  if (primeraMedia)
  {
    distanciaLleno = distanciaVacio - media;
    cantidadLiquido = distanciaLleno * 100 / distancia10000;

    Serial.print(media);
    Serial.println(" cm");
    
    Serial.print(cantidadLiquido);
    Serial.println(" l");
  }
    
}

void LecturaTemperatura() {
 
  temperatura = dhtSensor.readTemperature(); // Lectura de la temperatura
                                            
  Serial.println("Temperatura: " + String(temperatura) + "°C"); // Print por pantalla

  //delay(500); // Congela el bucle durante 500 milisegundos
}

void printGpsReadings(TimerHandle_t xTimer){

  LAT_GPS = gps.location.lat();
  LONG_GPS = gps.location.lng();
  
  Serial.print("LAT=");   Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
  Serial.print("LONG=");  Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
 
}


void publishSmallJson() {
  static const String topicStr = createTopic("Camion 2456-FCM");
  static const char *topic = topicStr.c_str();

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT11-HCSR04-NEO6MV2";
  JsonObject values =
      doc.createNestedObject("values"); // We can add another Object
  values["ID"] = 1;
  values["t"] = temperatura;
  values["l"] = cantidadLiquido;
  values["GPSLAT"] = LAT_GPS;
  values["GPSLONG"] = LONG_GPS;
  

  // Serialize the JSON document to a buffer in order to publish it
  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));

}

String createTopic(char *topic) {
  String topicStr = String(macAddress)  + " HALTER"+ "/" + topic;
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
