#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//--------------------------------------------------------------------------------------
// Pin Declaration
//--------------------------------------------------------------------------------------
#define M1 23 // Water pump 1
#define M2 19 // Water pump 2
#define M3 4  // Water pump 3
#define M4 2  // Water pump 4

#define V1 18 // Solenoid Valve 1
#define V2 5  // Solenoid Valve 2
#define V3 17 // Solenoid Valve 3
#define V4 16 // Solenoid Valve 4

#define TRIG_AB_PIN_AB_TANK 32 // Ultrasonic as Water Level AB Tank
#define ECHO_AB_PIN_AB_TANK 33
#define TRIG_AB_PIN_TANK_MIX 25 // Ultrasonic as Water Level Mix Tank
#define ECHO_AB_PIN_TANK_MIX 26

#define SOIL_MOISTURE_PIN 34

//--------------------------------------------------------------------------------------
// Project Configuration
//--------------------------------------------------------------------------------------
// int projectID = 8;
int projectID = 42;
//--------------------------------------------------------------------------------------
// Internet connection Configuration
//--------------------------------------------------------------------------------------
// Wi-Fi credentials
// const char *ssid = "Redmi Note 10S";
// const char *password = "1234567890";

const char *ssid = "13_6-2G";
const char *password = "01110292882abc";

// Function to connect to Wi-Fi
void connectToWiFi()
{
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}
//--------------------------------------------------------------------------------------
// MQTT Broker Configuration
//--------------------------------------------------------------------------------------
const char *mqtt_broker = "157.245.158.126"; // Digital Ocean
// const char *mqtt_broker = "broker.mqttdashboard.com"; // HiveMQ Broker
// const char* mqtt_broker = "test.mosquitto.org"; // Mosquitto Broker
// const char* mqtt_broker = 127.0.0.1; // Localhost
const char *mqtt_username = "mqtt_user";
const char *mqtt_password = "";
const int mqtt_port = 1883;
// const char *mqtt_client_id = "1234567"; // Define the MQTT client ID

// MQTT topic
const char *topic_pub = "esp32/data";
const char *topic_sub = "switch-button"; // Topic for receiving data from Laravel

// Variables for timing
unsigned long previousMillis = 0;
const long interval = 5000; // Interval to send data (in milliseconds)

// Variables for timer
unsigned long startMillis = 0;
unsigned long durationMillis = 0;
bool timerActive = false;
int currentInputId = 0;    // Variable to store current input ID
int currentScheduleId = 0; // Variable to store current schedule ID

// Client setup
WiFiClient espClient;
PubSubClient client(espClient);

void connectToMQTTBroker()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password))
    {
      Serial.println("Connected to MQTT broker");
      client.subscribe(topic_sub); // Subscribe to the topic
    }
    else
    {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

// Callback function when a message is received
void callback(char *topic, byte *payload, unsigned int length)
{
  StaticJsonDocument<256> jsonDoc;
  deserializeJson(jsonDoc, payload, length);

  int currentProjectId = jsonDoc["projectId"];

  if (currentProjectId == projectID)
  {
    int inputId = jsonDoc["inputId"];
    String slug = jsonDoc["slug"];
    bool status = jsonDoc["status"];
    int duration = jsonDoc["duration"]; // in minutes
    int limitSensor = jsonDoc["limitSensor"];
    int scheduleId = jsonDoc["scheduleId"];

    Serial.println("Message received:");
    Serial.print("Project ID: ");
    Serial.println(currentProjectId);
    Serial.print("Input ID: ");
    Serial.println(inputId);
    Serial.print("Input Slug: ");
    Serial.println(slug);
    Serial.print("Status: ");
    Serial.println(status);
    Serial.print("Duration: ");
    Serial.println(duration);
    Serial.print("Limit Sensor: ");
    Serial.println(limitSensor);
    Serial.print("Schedule ID: ");
    Serial.println(scheduleId);

    // Check if status is true to start the timer
    if (status)
    // if (status)
    {
      currentScheduleId = scheduleId;
      currentInputId = inputId;
      startMillis = millis();
      durationMillis = duration * 60000; // Convert minutes to milliseconds
      timerActive = true;
      Serial.println("Timer started.");
    }
    else
    {
      // If status is false, stop the timer immediately
      timerActive = false;
      Serial.println("Timer stopped manually.");
    }
  }
}

// List Sensor ID
// 1. EC Sensor
// 2. Float AB Tank Sensor
// 3. Float Mix Tank Sensor (%)
// 4. Water Temperature Sensor
// 5. Soil Sensor 1
// 6. Soil Sensor 2
// 7. Soil Sensor 3

// Sensor data (simulate data here)
// float temperature = 0;
// float humidity = 0;
// int lightLevel = 0;

//--------------------------------------------------------------------------------------
// Ultrasonic Sensor Configuration (Water Level)
//--------------------------------------------------------------------------------------
// Define variables for duration and distance for each sensor
long durationTank;
int distanceTank;

long durationABTank;
int distanceABTank;
const int ABTankHeight = 111; // Height of the water tank in mm (8.5 cm)

long durationMixTank;
int distanceMixTank;
const int mixTankHeight = 72; // Height of the water tank in mm (8.5 cm)
// const int mixTankHeight = 90; // Height of the water tank in mm (8.5 cm)
// const int mixTankHeight = 65; // Height of the water tank in mm (8.5 cm)

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// Function to calculate water level percentage
int waterLevel(int TRIG, int ECHO, int TankHeight)
{
  // Trigger ultrasonic sensor to send a pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Read the echoPin and calculate the duration of the pulse
  long duration = pulseIn(ECHO, HIGH);
  // Calculate the distance (duration / 2) / 29.1 = cm, convert to mm
  int distance = (duration / 2) / 29.1 * 10;
  // float  distanceCm = duration * SOUND_SPEED / 2;
  // float  distanceInch = distanceCm * CM_TO_INCH;

  // Calculate water level in mm and percentage
  int waterLevel = TankHeight - distance;           // Water level in mm
  int percentage = (waterLevel * 100) / TankHeight; // Water level percentage

  // Ensure percentage is within 0-100 range
  if (percentage > 100)
  {
    return 100;
  }
  else if (percentage < 0)
  {
    return 0;
  }
  return percentage;
}

//--------------------------------------------------------------------------------------
// Soil Moisture Sensor Configuration
//--------------------------------------------------------------------------------------
String soilMoisture(int soilMoistureValue)
{
  if (soilMoistureValue < 1)
  {
    return "WET";
  }
  else
  {
    return "DRY";
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(TRIG_AB_PIN_AB_TANK, OUTPUT);
  pinMode(ECHO_AB_PIN_AB_TANK, INPUT);
  pinMode(TRIG_AB_PIN_TANK_MIX, OUTPUT);
  pinMode(ECHO_AB_PIN_TANK_MIX, INPUT);

  pinMode(SOIL_MOISTURE_PIN, INPUT);

  connectToWiFi();

  // Connect to MQTT broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  connectToMQTTBroker();
}

void loop()
{
  if (!client.connected())
  {
    connectToMQTTBroker();
  }
  client.loop();

  //--------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------
  // Check if it's time to send data
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    int soilMoistureValue = digitalRead(SOIL_MOISTURE_PIN);

    // Calculate water levels
    int AB_Percentage = waterLevel(TRIG_AB_PIN_AB_TANK, ECHO_AB_PIN_AB_TANK, ABTankHeight);
    int Mix_Percentage = waterLevel(TRIG_AB_PIN_TANK_MIX, ECHO_AB_PIN_TANK_MIX, mixTankHeight);

    Serial.print("AB Water Level: ");
    Serial.print(AB_Percentage);
    Serial.println(" %");

    Serial.print("Mix Water Level: ");
    Serial.print(Mix_Percentage);
    Serial.println(" %");

    // Print the soil moisture value to the Serial Monitor
    Serial.print("Soil Moisture Value: ");
    Serial.println(soilMoistureValue);

    // Determine the status based on the soil moisture value
    Serial.println("Status: " + soilMoisture(soilMoistureValue));

    // Prepare sensor data
    StaticJsonDocument<300> jsonDoc;
    jsonDoc["project_id"] = projectID; // Send project_id only once
    jsonDoc["status"] = true;          // ESP32 is ON

    JsonArray sensorData = jsonDoc.createNestedArray("sensor_data");

    JsonObject sensor1 = sensorData.createNestedObject();
    sensor1["sensor_id"] = 1;          // EC Sensor
    sensor1["value"] = random(0, 100); // Example value for sensor 1

    JsonObject sensor2 = sensorData.createNestedObject();
    sensor2["sensor_id"] = 3;          // Float AB Sensor (%)
    sensor2["value"] = random(0, 100); // Example value for sensor 3

    JsonObject sensor3 = sensorData.createNestedObject();
    sensor3["sensor_id"] = 4;          // Water Temperature Sensor
    sensor3["value"] = random(0, 100); // Example value for sensor 4

    JsonObject sensor4 = sensorData.createNestedObject();
    sensor4["sensor_id"] = 5;          // Soil Sensor - 1
    sensor4["value"] = random(0, 100); // Example value for sensor 5

    // Serialize JSON object to string
    char jsonBuffer[512];
    serializeJson(jsonDoc, jsonBuffer);

    // Publish data to MQTT
    client.publish(topic_pub, jsonBuffer);
    Serial.println("Data sent:");
    Serial.println(jsonBuffer);
  }

  // Check if timer is active and duration has passed
  if (timerActive && millis() - startMillis >= durationMillis)
  {
    // Prepare JSON to publish status off
    StaticJsonDocument<128> jsonDoc;
    JsonArray statusInput = jsonDoc.createNestedArray("status_input");
    JsonObject input = statusInput.createNestedObject();
    input["inputId"] = currentInputId;
    input["status"] = 0;

    if (currentScheduleId != 0)
    {
      input["scheduleId"] = currentScheduleId;
    }

    // Serialize JSON to string
    char jsonBuffer[128];
    serializeJson(jsonDoc, jsonBuffer);

    // Publish the data
    client.publish(topic_pub, jsonBuffer);
    Serial.println("Timer expired, status set to false:");
    Serial.println(jsonBuffer);

    // Reset timer
    timerActive = false;
  }
}