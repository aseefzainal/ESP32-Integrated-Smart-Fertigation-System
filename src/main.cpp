#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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
const char *mqtt_broker = "64.227.7.210"; // Digital Ocean
// const char *mqtt_broker = "broker.mqttdashboard.com"; // HiveMQ Broker
// const char* mqtt_broker = "test.mosquitto.org"; // Mosquitto Broker
// const char* mqtt_broker = 127.0.0.1; // Localhost
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;
// const char *mqtt_client_id = "1234567"; // Define the MQTT client ID

// MQTT topic
const char *topic_pub = "esp32/data";

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
    }
    else
    {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

//--------------------------------------------------------------------------------------
// Project Configuration
//--------------------------------------------------------------------------------------
// int projectID = 6;
int projectID = 42;

// List Sensor ID
// 1. EC Sensor
// 2. Float AB Sensor
// 3. Float AB Sensor (%)
// 4. Water Temperature Sensor
// 5. Soil Sensor 1
// 6. Soil Sensor 2
// 7. Soil Sensor 3

// Sensor data (simulate data here)
// float temperature = 0;
// float humidity = 0;
// int lightLevel = 0;

void setup()
{
  Serial.begin(115200);
  connectToWiFi();

  // Connect to MQTT broker
  client.setServer(mqtt_broker, mqtt_port);
  connectToMQTTBroker();
}

void loop()
{
  if (!client.connected())
  {
    connectToMQTTBroker();
  }
  client.loop();

  // // Simulate reading from sensors
  // temperature = random(20, 30); // Example temperature in °C
  // humidity = random(40, 60);    // Example humidity in %
  // lightLevel = random(0, 1024); // Example light level (0-1024)

  // // Prepare JSON object with sensor data
  // StaticJsonDocument<200> jsonDoc;
  // jsonDoc["temperature"] = temperature;
  // jsonDoc["humidity"] = humidity;
  // jsonDoc["lightLevel"] = lightLevel;

  // // Serialize JSON object to string
  // char jsonBuffer[200];
  // serializeJson(jsonDoc, jsonBuffer);

  // // Publish data to MQTT topic
  // if (client.publish(topic_pub, jsonBuffer))
  // {
  //   Serial.println("Data sent successfully:");
  //   Serial.println(jsonBuffer);
  // }
  // else
  // {
  //   Serial.println("Failed to send data");
  // }

  // Prepare sensor data
  StaticJsonDocument<300> jsonDoc;
  JsonArray sensorData = jsonDoc.createNestedArray("sensor_data");

  JsonObject sensor1 = sensorData.createNestedObject();
  sensor1["project_id"] = projectID;
  sensor1["sensor_id"] = 1;          // EC Sensor
  sensor1["value"] = random(0, 100); // Example value for sensor 1

  JsonObject sensor2 = sensorData.createNestedObject();
  sensor2["project_id"] = projectID;
  sensor2["sensor_id"] = 3;          // Float AB Sensor (%)
  sensor2["value"] = random(0, 100); // Example value for sensor 3

  JsonObject sensor3 = sensorData.createNestedObject();
  sensor3["project_id"] = projectID;
  sensor3["sensor_id"] = 4;          // Water Temperature Sensor
  sensor3["value"] = random(0, 100); // Example value for sensor 4

  JsonObject sensor4 = sensorData.createNestedObject();
  sensor4["project_id"] = projectID;
  sensor4["sensor_id"] = 5;          // Soil Sensor - 1
  sensor4["value"] = random(0, 100); // Example value for sensor 5

  // Serialize JSON object to string
  char jsonBuffer[512];
  serializeJson(jsonDoc, jsonBuffer);

  // Publish data to MQTT
  client.publish(topic_pub, jsonBuffer);
  Serial.println("Data sent:");
  Serial.println(jsonBuffer);

  delay(5000); // Delay before sending the next data
}