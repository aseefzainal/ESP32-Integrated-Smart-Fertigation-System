#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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
#define TDS_SENSOR_PIN 35
const int oneWireBus = 27;

//--------------------------------------------------------------------------------------
// Project Configuration
//--------------------------------------------------------------------------------------
String projectSlug = "@year_2435"; // Replace with your project slug
// String projectSlug = "@year_1555"; // Replace with your project slug
// String projectSlug = "kandis-site"; // Replace with your project slug

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

//--------------------------------------------------------------------------------------
// Water Temperature Sensor (DS18B20)
//--------------------------------------------------------------------------------------
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

float temperatureC = 0;
float temperatureF = 0;
//--------------------------------------------------------------------------------------
// TDS Sensor
//--------------------------------------------------------------------------------------
#define VREF 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
// int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
// float temperature = 29; // current temperature for compensation

float conversionFactor = 0.5;                // Typical conversion factor for TDS to EC
float ecValue = tdsValue / conversionFactor; // EC in μS/cm

// float conversionFactor = 0.5;                // Typical conversion factor for TDS to EC
// float ecValue = tdsValue / conversionFactor; // EC in μS/cm

float getMedianNum(float *buffer, int size)
{
  float temp;
  for (int i = 0; i < size - 1; i++)
  {
    for (int j = i + 1; j < size; j++)
    {
      if (buffer[i] > buffer[j])
      {
        temp = buffer[i];
        buffer[i] = buffer[j];
        buffer[j] = temp;
      }
    }
  }
  if (size % 2 == 0)
  {
    return (buffer[size / 2] + buffer[size / 2 - 1]) / 2.0;
  }
  else
  {
    return buffer[size / 2];
  }
}

void readDS18B20()
{
  sensors.requestTemperatures();
  temperatureC = sensors.getTempCByIndex(0);
  temperatureF = sensors.getTempFByIndex(0);
  // Serial.print(temperatureC);
  // Serial.println("ºC");
  // Serial.print(temperatureF);
  // Serial.println("ºF");
}

void readTdsSensor()
{
  float analogBufferTemp[SCOUNT];
  for (int i = 0; i < SCOUNT; i++)
  {
    analogBufferTemp[i] = analogBuffer[i];
  }

  averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
  float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;

  tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

  float conversionFactor = 0.5; // Conversion factor for TDS to EC
  ecValue = tdsValue / conversionFactor;
}

//--------------------------------------------------------------------------------------
// CallBack Function and configuration
//--------------------------------------------------------------------------------------
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
int limitSensor = 0;

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
    limitSensor = jsonDoc["limitSensor"];
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
//--------------------------------------------------------------------------------------
// Water Level (Ultrasonic Sensor)
//--------------------------------------------------------------------------------------
int AB_Percentage = 0;
int Mix_Percentage = 0;

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
//--------------------------------------------------------------------------------------
// Soil Moisture Sensor
//--------------------------------------------------------------------------------------
String soilMoisture(int soilMoistureValue)
{
  return soilMoistureValue < 1 ? "WET" : "DRY";
}
//--------------------------------------------------------------------------------------
// Irrigation Configuration
//--------------------------------------------------------------------------------------
void fertilizerIrrigation(bool status, int mixPercentage, int currentEcSensor)
{
  Serial.println("Current EC Sensor: " + String(currentEcSensor));
  Serial.println("Limit EC Sensor: " + String(limitSensor));
  Serial.println("Mix Percentage: " + String(mixPercentage));

  if (status == 1)
  {
    // Fill the mix tank with water until it is at least 30% full
    if (mixPercentage < 30)
    {
      // on the water pump and solenoid valve
      Serial.println("Fill the mix tank with water until it is at least 30 percent full");
      Serial.println("ON");
      digitalWrite(M2, !status);
      digitalWrite(V3, !status);
    }
    else
    {
      // off the water pump and solenoid valve
      Serial.println("Fill the mix tank with water until it is at least 30 percent full");
      Serial.println("OFF");
      digitalWrite(M2, status);
      digitalWrite(V3, status);

      // Add AB Solution to the mix tank when the EC Sensor reaches the limit
      if (currentEcSensor < limitSensor)
      {
        Serial.println("Add AB Solution to the mix tank when the EC Sensor reaches the limit");
        Serial.println("ON");
        digitalWrite(M3, !status);
        //
        digitalWrite(M1, !status);
        digitalWrite(V1, !status);
      }
      else if (currentEcSensor >= limitSensor)
      {
        Serial.println("Add AB Solution to the mix tank when the EC Sensor reaches the limit");
        Serial.println("OFF");
        digitalWrite(M3, status);
        //
        // digitalWrite(M1, status);
        digitalWrite(V1, status);

        // Send water from the mix tank to the plant
        Serial.println("Send water from the mix tank to the plant");
        Serial.println("ON");
        digitalWrite(M1, !status);
        digitalWrite(V2, !status);
      }
    }
  }
  else
  {
    digitalWrite(M1, !status);
    digitalWrite(M2, !status);
    digitalWrite(M3, !status);
    digitalWrite(V1, !status);
    digitalWrite(V2, !status);
    digitalWrite(V3, !status);
  }
}

void waterIrrigation(bool status)
{
  digitalWrite(M2, !status);
  digitalWrite(V4, !status);
}
//--------------------------------------------------------------------------------------
// Setup Function
//--------------------------------------------------------------------------------------
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
  pinMode(TDS_SENSOR_PIN, INPUT);

  // set all solenoid valve is off by default
  digitalWrite(V1, HIGH);
  digitalWrite(V2, HIGH);
  digitalWrite(V3, HIGH);
  digitalWrite(V4, HIGH);

  // set all water pump is off by default
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  digitalWrite(M3, HIGH);
  digitalWrite(M4, HIGH);

  // Start the DS18B20 sensor
  sensors.begin();

  connectToWiFi();

  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  connectToMQTTBroker();
}
//--------------------------------------------------------------------------------------
// Loop Function
//--------------------------------------------------------------------------------------
void loop()
{
  if (!client.connected())
  {
    connectToMQTTBroker();
  }
  client.loop();

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)
  { // Every 40 milliseconds, read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_SENSOR_PIN); // Read the analog value and store it into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
    {
      analogBufferIndex = 0;
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    int soilMoistureValue = digitalRead(SOIL_MOISTURE_PIN);
    AB_Percentage = waterLevel(TRIG_AB_PIN_AB_TANK, ECHO_AB_PIN_AB_TANK, 111);
    Mix_Percentage = waterLevel(TRIG_AB_PIN_TANK_MIX, ECHO_AB_PIN_TANK_MIX, 72);

    readTdsSensor(); // Get EC and PPM values
    readDS18B20();

    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.print(" ppm   ");
    Serial.print("EC Value: ");
    Serial.print(ecValue, 2);
    Serial.println(" μS/cm");

    StaticJsonDocument<1024> jsonDoc;
    jsonDoc["project_slug"] = projectSlug;
    jsonDoc["status"] = true;

    JsonArray sensorData = jsonDoc.createNestedArray("sensor_data");

    JsonObject sensor1 = sensorData.createNestedObject();
    sensor1["sensor_slug"] = "ec";
    sensor1["value"] = ecValue;

    // JsonObject sensor2 = sensorData.createNestedObject();
    // sensor2["sensor_slug"] = "ppm";
    // sensor2["value"] = tdsValue; // PPM value

    // JsonObject sensor3 = sensorData.createNestedObject();
    // sensor3["sensor_slug"] = "water-temperature";
    // sensor3["value"] = temperatureC;
    // sensor3["value"] = random(0, 100);

    JsonObject sensor4 = sensorData.createNestedObject();
    sensor4["sensor_slug"] = "float-ab-percentage";
    sensor4["value"] = AB_Percentage;

    JsonObject sensor5 = sensorData.createNestedObject();
    sensor5["sensor_slug"] = "float-mix-percentage";
    sensor5["value"] = Mix_Percentage;

    JsonObject sensor6 = sensorData.createNestedObject();
    sensor6["sensor_slug"] = "soil-1";
    sensor6["value"] = soilMoistureValue;
    // sensor6["value"] = soilMoisture(soilMoistureValue);

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
    digitalWrite(V1, !inputStatus);
  }
  else if (inputSlug == "v3")
  {
    Serial.println(inputSlug);
    digitalWrite(V2, !inputStatus);
  }
  else if (inputSlug == "v4")
  {
    Serial.println(inputSlug);
    digitalWrite(V3, !inputStatus);
  }
  else if (inputSlug == "v5")
  {
    Serial.println(inputSlug);
    digitalWrite(V4, !inputStatus);
  }

  if (inputSlug == "water-irrigation")
  {
    Serial.println(inputSlug);
    waterIrrigation(inputStatus);
  }
  else if (inputSlug == "fertilizer-irrigation")
  {
    Serial.println(inputSlug);
    fertilizerIrrigation(inputStatus, Mix_Percentage, ecValue);
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
      fertilizerIrrigation(!inputStatus, Mix_Percentage, ecValue);
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