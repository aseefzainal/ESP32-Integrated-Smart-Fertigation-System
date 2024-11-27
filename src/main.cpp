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
String projectSlug = "@year_1555"; // Replace with your project slug

//--------------------------------------------------------------------------------------
// Internet connection Configuration
//--------------------------------------------------------------------------------------
const char *ssid = "13_6-2G";
const char *password = "01110292882abc";

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

//--------------------------------------------------------------------------------------
// MQTT Broker Configuration
//--------------------------------------------------------------------------------------
const char *mqtt_broker = "157.245.158.126";
const char *mqtt_username = "mqtt_user";
const char *mqtt_password = "";
const int mqtt_port = 1883;

const char *topic_pub = "esp32/data";
const char *topic_sub = "switch-button";

WiFiClient espClient;
PubSubClient client(espClient);

// Timing Variables
unsigned long previousMillis = 0;
const long interval = 5000; // 5 seconds

// Timer Variables
unsigned long startMillis = 0;
unsigned long durationMillis = 0;
bool timerActive = false;
int currentInputId = 0;
int currentScheduleId = 0;

String inputSlug = "";
bool inputStatus = false;

void connectToMQTTBroker()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password))
    {
      Serial.println("Connected to MQTT broker");
      client.subscribe(topic_sub);
    }
    else
    {
      Serial.print("Failed to connect, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  StaticJsonDocument<256> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, payload, length);

  if (error)
  {
    Serial.print("JSON deserialization failed: ");
    Serial.println(error.c_str());
    return;
  }

  String currentProjectSlug = jsonDoc["projectSlug"];
  if (currentProjectSlug == projectSlug)
  {
    int inputId = jsonDoc["inputId"];
    // String inputSlug = jsonDoc["slug"];
    inputSlug = jsonDoc["slug"].as<String>();
    inputStatus = jsonDoc["status"];
    int duration = jsonDoc["duration"];
    int limitSensor = jsonDoc["limitSensor"];
    int scheduleId = jsonDoc["scheduleId"];

    Serial.println("Message received:");
    Serial.println("Project Slug: " + currentProjectSlug);
    Serial.println("Input ID: " + String(inputId));
    Serial.println("Input Slug: " + String(inputSlug));
    Serial.println("Status: " + String(inputStatus));
    Serial.println("Duration: " + String(duration));
    Serial.println("Limit EC Sensor: " + String(limitSensor));
    Serial.println("Schedule ID: " + String(scheduleId));

    if (inputStatus)
    {
      currentScheduleId = scheduleId;
      currentInputId = inputId;
      startMillis = millis();
      durationMillis = duration * 60000;
      timerActive = true;
      Serial.println("Timer started.");
    }
    else
    {
      timerActive = false;
      Serial.println("Timer stopped manually.");
      // inputSlug = "";
    }
  }
}

int waterLevel(int TRIG, int ECHO, int TankHeight)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH);
  int distance = (duration / 2) / 29.1 * 10;
  int waterLevel = TankHeight - distance;
  int percentage = (waterLevel * 100) / TankHeight;

  return constrain(percentage, 0, 100);
}

String soilMoisture(int soilMoistureValue)
{
  return soilMoistureValue < 1 ? "WET" : "DRY";
}

void fertilizerIrrigation(bool status)
{
}

void waterIrrigation(bool status)
{
  digitalWrite(M2, !status);
  digitalWrite(V4, status);
}

void setup()
{
  Serial.begin(115200);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);

  pinMode(V1, OUTPUT);
  pinMode(V2, OUTPUT);
  pinMode(V3, OUTPUT);
  pinMode(V4, OUTPUT);

  pinMode(TRIG_AB_PIN_AB_TANK, OUTPUT);
  pinMode(ECHO_AB_PIN_AB_TANK, INPUT);
  pinMode(TRIG_AB_PIN_TANK_MIX, OUTPUT);
  pinMode(ECHO_AB_PIN_TANK_MIX, INPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);

  // set all water pump is off by default
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  digitalWrite(M3, HIGH);
  digitalWrite(M4, HIGH);

  connectToWiFi();

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

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    int soilMoistureValue = digitalRead(SOIL_MOISTURE_PIN);
    int AB_Percentage = waterLevel(TRIG_AB_PIN_AB_TANK, ECHO_AB_PIN_AB_TANK, 111);
    int Mix_Percentage = waterLevel(TRIG_AB_PIN_TANK_MIX, ECHO_AB_PIN_TANK_MIX, 72);

    StaticJsonDocument<1024> jsonDoc;
    jsonDoc["project_slug"] = projectSlug;
    jsonDoc["status"] = true;

    JsonArray sensorData = jsonDoc.createNestedArray("sensor_data");

    JsonObject sensor1 = sensorData.createNestedObject();
    sensor1["sensor_slug"] = "ec";
    sensor1["value"] = random(0, 100);

    // JsonObject sensor2 = sensorData.createNestedObject();
    // sensor2["sensor_slug"] = "water-temperature";
    // sensor2["value"] = random(0, 100);

    JsonObject sensor3 = sensorData.createNestedObject();
    sensor3["sensor_slug"] = "float-ab-percentage";
    sensor3["value"] = AB_Percentage;

    JsonObject sensor4 = sensorData.createNestedObject();
    sensor4["sensor_slug"] = "float-mix-percentage";
    sensor4["value"] = Mix_Percentage;

    JsonObject sensor5 = sensorData.createNestedObject();
    sensor5["sensor_slug"] = "soil-1";
    sensor5["value"] = soilMoistureValue;
    // sensor4["value"] = soilMoisture(soilMoistureValue);

    char jsonBuffer[1024];
    serializeJson(jsonDoc, jsonBuffer);

    client.publish(topic_pub, jsonBuffer);
    Serial.println("Data sent:");
    Serial.println(jsonBuffer);
  }

  if (inputSlug == "m1")
  {
    Serial.println(inputSlug);
    digitalWrite(M1, !inputStatus);
  }
  else if (inputSlug == "m2")
  {
    Serial.println(inputSlug);
    digitalWrite(M2, !inputStatus);
  }
  else if (inputSlug == "m3")
  {
    Serial.println(inputSlug);
    digitalWrite(M3, !inputStatus);
  }
  else if (inputSlug == "m4")
  {
    Serial.println(inputSlug);
    digitalWrite(M4, !inputStatus);
  }
  else if (inputSlug == "v2")
  {
    Serial.println(inputSlug);
    digitalWrite(V1, inputStatus);
  }
  else if (inputSlug == "v3")
  {
    Serial.println(inputSlug);
    digitalWrite(V2, inputStatus);
  }
  else if (inputSlug == "v4")
  {
    Serial.println(inputSlug);
    digitalWrite(V3, inputStatus);
  }
  else if (inputSlug == "v5")
  {
    Serial.println(inputSlug);
    digitalWrite(V4, inputStatus);
  }

  if (inputSlug == "water-irrigation")
  {
    Serial.println(inputSlug);
    waterIrrigation(inputStatus);
  }
  else if (inputSlug == "fertilizer-irrigation")
  {
    Serial.println(inputSlug);
    fertilizerIrrigation(inputStatus);
  }

  Serial.println(inputSlug);
  Serial.println(timerActive);
  delay(500);

  if (timerActive && durationMillis > 0 && millis() - startMillis >= durationMillis)
  {
    StaticJsonDocument<128> jsonDoc;
    JsonArray statusInput = jsonDoc.createNestedArray("status_input");
    JsonObject input = statusInput.createNestedObject();
    input["inputId"] = currentInputId;
    input["status"] = 0;
    if (currentScheduleId != 0)
    {
      input["scheduleId"] = currentScheduleId;
    }

    if (inputSlug == "water-irrigation")
    {
      Serial.println(inputSlug);
      waterIrrigation(!inputStatus);
    }
    else if (inputSlug == "fertilizer-irrigation")
    {
      Serial.println(inputSlug);
      fertilizerIrrigation(!inputStatus);
    }

    char jsonBuffer[128];
    serializeJson(jsonDoc, jsonBuffer);

    client.publish(topic_pub, jsonBuffer);
    Serial.println("Timer expired, status set to false:");
    Serial.println(jsonBuffer);

    timerActive = false;
    inputSlug = "";
  }
}
