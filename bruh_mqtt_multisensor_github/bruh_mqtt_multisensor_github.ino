/*

  Based on https://github.com/bruhautomation/ESP-MQTT-JSON-Multisensor with a lot of insperation from https://github.com/ShawnCorey/ESP-MQTT-Multisensor-Authdiscovery

  I have added a working auto discovery functionality for Home Assistant. And shorten the fade time to 255 steps

  Thanks much to @corbanmailloux for providing a great framework for implementing flash/fade with HomeAssistant https://github.com/corbanmailloux/esp-mqtt-rgb-led

  To use this code you will need the following dependancies: 
  
  - Support for the ESP8266 boards. 
        - You can add it to the board manager by going to File -> Preference and pasting http://arduino.esp8266.com/stable/package_esp8266com_index.json into the Additional Board Managers URL field.
        - Next, download the ESP8266 dependancies by going to Tools -> Board -> Board Manager and searching for ESP8266 and installing it.
  
  - You will also need to download the follow libraries by going to Sketch -> Include Libraries -> Manage Libraries
      - DHT sensor library 
      - Adafruit unified sensor
      - PubSubClient
      - ArduinoJSON

  TODO:
  send command to the unit to remove the configurations from Home Assistant and to restart etc... inplemented nicely in https://github.com/ShawnCorey/ESP-MQTT-Multisensor-Authdiscovery
  
*/

#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Arduino_JSON.h>

/************ TEMP SETTINGS (CHANGE THIS FOR YOUR SETUP) *******************************/
#define IsFahrenheit false //to use celsius change to false

/************ WIFI and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
#define wifi_ssid "YourSSID" //type your WIFI information inside the quotes
#define wifi_password "YourWIFIpassword"
#define mqtt_server "your.mqtt.server.ip"
#define mqtt_user "yourMQTTusername" 
#define mqtt_password "yourMQTTpassword"
#define mqtt_port 1883

/**************************** PIN DEFINITIONS ********************************************/
const int redPin = D1;
const int greenPin = D2;
const int bluePin = D3;
#define PIRPIN D5
#define DHTPIN D7
#define DHTTYPE DHT22
#define LDRPIN A0


//  Home Assistant discovery prefix normaly set to discovery_prefix: homeassistant
#define HA_DISCOVERY_PREFIX "homeassistant"
#define DEVICE_NAME "multisensor1"
#define DEVICE_FRIENDLY_NAME "nodeMCU Vardagsrum"

/**************************** FOR OTA **************************************************/
#define SENSORNAME "sensornode1"
#define OTApassword "YouPassword" // change this to whatever password you want to use when you upload OTA
int OTAport = 8266;

//could be set to "" if you don't want to have the nodemcu in the path
#define TOPIC_PREFIX "/nodemcu"

/************* MQTT COMMAND /ON/OFF (change these topics as you wish, but not if you want to use with Home Assistant)  **************************/
const char *on_cmd = "ON";
const char *off_cmd = "OFF";

// Should not need to set anything below this, unless you have changed the MQTT path for HA discovery

#if IsFahrenheit
#define UNIT_OF_MEASUREMENT "°F"
#else
#define UNIT_OF_MEASUREMENT "°C"
#endif

//
// MQTT TOPICS (to maintain HomeAssitant discovery ability do not edit unless you know what you are doing)
//

// Topics for general device management, this if for actions like forcing re/un-registration w/ discovery, rebooting, etc
// Not implemented yet!
#define DEVICE_DEVICE_COMMAND_TOPIC HA_DISCOVERY_PREFIX DEVICE_NAME "/set"
#define DEVICE_DEVICE_STATE_TOPIC HA_DISCOVERY_PREFIX TOPIC_PREFIX "/" DEVICE_NAME "/state"

// Topics for PIR (motion sensor)
#define DEVICE_PIR_TOPIC_ROOT HA_DISCOVERY_PREFIX "/binary_sensor" TOPIC_PREFIX "/" DEVICE_NAME "_pir"
#define DEVICE_PIR_DISCOVERY_TOPIC          DEVICE_PIR_TOPIC_ROOT "/config"
//#define DEVICE_PIR_STATE_TOPIC              DEVICE_PIR_TOPIC_ROOT "/state"

// Topics for temperature sensor
#define DEVICE_TEMP_TOPIC_ROOT HA_DISCOVERY_PREFIX "/sensor" TOPIC_PREFIX "/" DEVICE_NAME "_temp"
#define DEVICE_TEMP_DISCOVERY_TOPIC         DEVICE_TEMP_TOPIC_ROOT "/config"
//#define DEVICE_TEMP_STATE_TOPIC             DEVICE_TEMP_TOPIC_ROOT "/state"

// Topics for humidity sensor
#define DEVICE_HUMIDITY_TOPIC_ROOT HA_DISCOVERY_PREFIX "/sensor" TOPIC_PREFIX "/" DEVICE_NAME "_humidity"
#define DEVICE_HUMIDITY_DISCOVERY_TOPIC     DEVICE_HUMIDITY_TOPIC_ROOT "/config"
//#define DEVICE_HUMIDITY_STATE_TOPIC         DEVICE_HUMIDITY_TOPIC_ROOT "/state"

// Topics for LDR (light sensor)
#define DEVICE_LDR_TOPIC_ROOT HA_DISCOVERY_PREFIX "/sensor" TOPIC_PREFIX "/" DEVICE_NAME "_ldr"
#define DEVICE_LDR_DISCOVERY_TOPIC          DEVICE_LDR_TOPIC_ROOT "/config"
//#define DEVICE_LDR_STATE_TOPIC              DEVICE_LDR_TOPIC_ROOT "/state"

// Topics for LED. there are 3 sets because they cover on/off, brightness and RGB color seperately
#define DEVICE_LED_TOPIC_ROOT HA_DISCOVERY_PREFIX "/light" TOPIC_PREFIX "/" DEVICE_NAME "_led"
#define DEVICE_LED_DISCOVERY_TOPIC          DEVICE_LED_TOPIC_ROOT "/config"
//#define DEVICE_LED_STATE_TOPIC              DEVICE_LED_TOPIC_ROOT "/state"

#define DEVICE_LED_COMMAND_TOPIC            DEVICE_LED_TOPIC_ROOT "/set"
#define DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC DEVICE_LED_TOPIC_ROOT "/brightness"
#define DEVICE_LED_RGB_COMMAND_TOPIC        DEVICE_LED_TOPIC_ROOT "/rgb"

// Message text for LED state
#define MQTT_ON_CMD "ON"   // command that sets relay on
#define MQTT_OFF_CMD "OFF" // command that sets relay off

// Message text for device commands
#define MQTT_RESET_CMD "reset"           // command that resets the device
#define MQTT_STAT_CMD "stat"             // command to resend all state
#define MQTT_REGISTER_CMD "register"     // command to force reregistration
#define MQTT_UNREGISTER_CMD "unregister" // command to force unregistration

/**************************** SENSOR DEFINITIONS *******************************************/
// Variables for LDR(light sensor)
float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 25;

// Variables for temp sensor
float diffTEMP = 0.2;
float tempValue;

// Variables for humidity sensor
float diffHUM = 1;
float humValue;

// Variables for PIR(motion sensor)
int pirValue;
int pirStatus;
int pirOldValue;
long pirTimer;
bool motionStatus = false;

int calibrationTime = 5;

#define MQTT_MAX_PACKET_SIZE 1024

/******************************** GLOBALS for fade/flash *******************************/
byte red = 255;
byte green = 255;
byte blue = 255;
byte brightness = 255;

byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

bool stateOn = false;
bool updateState = false;

bool startFade = false;
unsigned long lastLoop = 0;
int transitionTime = 1;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB;
int redVal, grnVal, bluVal;

bool flash = false;
bool startFlash = false;
int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = red;
byte flashGreen = green;
byte flashBlue = blue;
byte flashBrightness = brightness;

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

/********************************** START SETUP*****************************************/
void setup()
{

  Serial.begin(115200);

  pinMode(PIRPIN, INPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(LDRPIN, INPUT);

  Serial.begin(115200);
  delay(10);
  
  dht.begin();

  ArduinoOTA.setPort(OTAport);

  ArduinoOTA.setHostname(SENSORNAME);

  ArduinoOTA.setPassword((const char *)OTApassword);

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++)
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Starting Node named " + String(SENSORNAME));

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IPess: ");
  Serial.println(WiFi.localIP());

  Serial.print("reading DHT22 temp: ");
  tempValue = dht.readTemperature(IsFahrenheit);
  humValue = dht.readHumidity();
  Serial.print(tempValue);
  Serial.print(" humidity: ");
  Serial.println(humValue);
  
  reconnect();

  setupmessagesforDiscovery();
}

void setupmessagesforDiscovery() {

  // Message text for auto discovery
  
  JSONVar device_pir_discovery_register_message;
  JSONVar device_temp_discovery_register_message;
  JSONVar device_humidity_discovery_register_message;
  JSONVar device_ldr_discovery_register_message;
  JSONVar device_led_discovery_register_message;
  
  //#define device_pir_discovery_register_message "{\"name\":\"" DEVICE_FRIENDLY_NAME " Motion Sensor\",\"device_class\":\"motion\",\"state_topic\":\"" DEVICE_PIR_STATE_TOPIC "\"}"
  device_pir_discovery_register_message["name"] = DEVICE_FRIENDLY_NAME " Motion Sensor";
  device_pir_discovery_register_message["device_class"] = "motion";
  device_pir_discovery_register_message["state_topic"] = DEVICE_DEVICE_STATE_TOPIC;
  device_pir_discovery_register_message["value_template"] = "{{ value_json.motion }}";
  
  Serial.println("Publishing device_pir_discovery_register_message json object");
  Serial.println(device_pir_discovery_register_message);
  client.publish(DEVICE_PIR_DISCOVERY_TOPIC, JSON.stringify(device_pir_discovery_register_message).c_str(), true);
  
  //#define device_temp_discovery_register_message "{\"name\":\"" DEVICE_FRIENDLY_NAME " Sensor Temp\",\"unit_of_measurement\":\"" UNIT_OF_MEASUREMENT "\",\"state_topic\":\"" DEVICE_TEMP_STATE_TOPIC "\"}"
  device_temp_discovery_register_message["name"] = DEVICE_FRIENDLY_NAME " Sensor Temp";
  device_temp_discovery_register_message["unit_of_measurement"] = UNIT_OF_MEASUREMENT;
  device_temp_discovery_register_message["state_topic"] = DEVICE_DEVICE_STATE_TOPIC;
  device_temp_discovery_register_message["value_template"] = "{{ value_json.temperature }}";
  
  Serial.println("Publishing device_temp_discovery_register_message json object");
  Serial.println(device_temp_discovery_register_message);
  client.publish(DEVICE_TEMP_DISCOVERY_TOPIC, JSON.stringify(device_temp_discovery_register_message).c_str(), true);
  
  //#define device_humidity_discovery_register_message "{\"name\":\"" DEVICE_FRIENDLY_NAME " Sensor Humidity\",\"unit_of_measurement\":\"%\",\"state_topic\":\"" DEVICE_HUMIDITY_STATE_TOPIC "\"}"
  device_humidity_discovery_register_message["name"] = DEVICE_FRIENDLY_NAME " Sensor Humidity";
  device_humidity_discovery_register_message["unit_of_measurement"] = "%";
  device_humidity_discovery_register_message["state_topic"] = DEVICE_DEVICE_STATE_TOPIC;
  device_humidity_discovery_register_message["value_template"] = "{{ value_json.humidity }}";
  
  Serial.println("Publishing device_humidity_discovery_register_message json object");
  Serial.println(device_humidity_discovery_register_message);
  client.publish(DEVICE_HUMIDITY_DISCOVERY_TOPIC, JSON.stringify(device_humidity_discovery_register_message).c_str(), true);
  
  //#define device_ldr_discovery_register_message "{\"name\":\"" DEVICE_FRIENDLY_NAME " Sensor Light Level\",\"unit_of_measurement\":\"lux\",\"state_topic\":\"" DEVICE_LDR_STATE_TOPIC "\"}"
  device_ldr_discovery_register_message["name"] = DEVICE_FRIENDLY_NAME " Sensor Light Level";
  device_ldr_discovery_register_message["unit_of_measurement"] = "lux";
  device_ldr_discovery_register_message["state_topic"] = DEVICE_DEVICE_STATE_TOPIC;
  device_ldr_discovery_register_message["value_template"] = "{{ value_json.ldr }}";
  
  Serial.println("Publishing device_ldr_discovery_register_message json object");
  Serial.println(device_ldr_discovery_register_message);
  client.publish(DEVICE_LDR_DISCOVERY_TOPIC, JSON.stringify(device_ldr_discovery_register_message).c_str(), true);
  
  /*//#define device_led_discovery_register_message "{
    \"name\":\"" DEVICE_FRIENDLY_NAME " LED\",
    \"brightness\":true,
    \"flash\":true,
    \"rgb\":true,
    \"optomistic\":false,
    \"qos\":0,
    \"command_topic\":\"" DEVICE_LED_COMMAND_TOPIC "\",
    \"brightness_command_topic\":\"" DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC "\",
    \"brightness_state_topic\":\"" DEVICE_LED_RGB_STATE_TOPIC "\",
    \"rgb_command_topic\":\"" DEVICE_LED_RGB_COMMAND_TOPIC "\",
    \"rgb_state_topic\":\"" DEVICE_LED_BRIGHTNESS_STATE_TOPIC "\"}"
  //*/
  device_led_discovery_register_message["name"] = DEVICE_FRIENDLY_NAME " LED";
  device_led_discovery_register_message["qos"] = 0;
  device_led_discovery_register_message["brightness_state_topic"] = DEVICE_DEVICE_STATE_TOPIC;
  device_led_discovery_register_message["brightness_value_template"] = "{{ (value_json.brightness|int(base=16))}}";
  device_led_discovery_register_message["rgb_state_topic"] = DEVICE_DEVICE_STATE_TOPIC;
  device_led_discovery_register_message["rgb_value_template"] = "{{ (value_json.color.r|int(base=16),value_json.color.g|int(base=16),value_json.color.b|int(base=16)) | join(',')}}";
  device_led_discovery_register_message["command_topic"] = DEVICE_LED_COMMAND_TOPIC;
  device_led_discovery_register_message["brightness_command_topic"] = DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC;
  device_led_discovery_register_message["rgb_command_topic"] = DEVICE_LED_RGB_COMMAND_TOPIC;
  
  Serial.println("Publishing device_led_discovery_register_message json object");
  Serial.println(device_led_discovery_register_message);
  if (!client.publish(DEVICE_LED_DISCOVERY_TOPIC, JSON.stringify(device_led_discovery_register_message).c_str(), true))
    Serial.println("Failed to publish");
  return;
}

/********************************** START SETUP WIFI*****************************************/
void setup_wifi()
{

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/********************************** START CALLBACK*****************************************/
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++)
  {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (strcmp(topic, DEVICE_LED_COMMAND_TOPIC) == 0){
    if (!processCommand(message)) return;
  }
  else if (strcmp(topic, DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC) == 0){
    if (!processBrightness(message)) return;
  }
  else if (strcmp(topic, DEVICE_LED_RGB_COMMAND_TOPIC) == 0){
    if (!processRGB(message)) return;
  } 
  else if (!processJson(message))
  {
    return;
  }

  if (stateOn)
  {
    // Update lights
    realRed = map(red, 0, 255, 0, brightness);
    realGreen = map(green, 0, 255, 0, brightness);
    realBlue = map(blue, 0, 255, 0, brightness);
  }
  else
  {
    realRed = 0;
    realGreen = 0;
    realBlue = 0;
  }

  startFade = true;
  inFade = false; // Kill the current fade

  sendState();
}

/********************************** START PROCESS brightness*****************************************/
bool processBrightness(char *message)
{
    Serial.print("Processing Brightness! .. ");
    Serial.println(atoi(message));
    brightness = atoi(message);

}

/********************************** START PROCESS rgb*****************************************/
bool processRGB(char *message)
{
    Serial.print("Processing RGB! .. ");
    Serial.println(message);
    char* color = strtok(message, ",");
    red = atoi(color);
    color = strtok(0, ",");
    green = atoi(color);
    color = strtok(0, ",");
    blue = atoi(color);
}

/********************************** START PROCESS command*****************************************/
bool processCommand(char *message)
{

    Serial.print("Processing Command! .. ");
    Serial.println(message);
    if (strcmp(message, on_cmd) == 0)
    {
      stateOn = true;
      return true;
    }
    else if (strcmp(message, off_cmd) == 0)
    {
      stateOn = false;
      return true;
    }
    return false;

}

/********************************** START PROCESS JSON*****************************************/
bool processJson(char *message)
{

  JSONVar root = JSON.parse(message);

  // JSON.typeof(jsonVar) can be used to get the type of the var
  if (JSON.typeof(root) == "undefined")
  {
    Serial.println("Parsing input failed!");
    return false;
  }
  // root.hasOwnProperty(key) checks if the object contains an entry for key
  if (root.hasOwnProperty("state"))
  {
    if (strcmp(root["state"], on_cmd) == 0)
    {
      stateOn = true;
    }
    else if (strcmp(root["state"], off_cmd) == 0)
    {
      stateOn = false;
    }
  }

  // If "flash" is included, treat RGB and brightness differently
  if (root.hasOwnProperty("flash"))
  {
    flashLength = (int)root["flash"] * 1000;

    if (root.hasOwnProperty("brightness"))
    {
      int myFlashBrightness = root["brightness"];
      flashBrightness = (byte)myFlashBrightness;
    }
    else
    {
      flashBrightness = (byte)brightness;
    }

    if (root.hasOwnProperty("color") && root["color"].hasOwnProperty("r") && root["color"].hasOwnProperty("g") && root["color"].hasOwnProperty("b"))
    {
      int tempRed = root["color"]["r"];
      int tempGreen = root["color"]["g"];
      int tempBlue = root["color"]["b"];
      red = (byte)tempRed;
      green = (byte)tempGreen;
      blue = (byte)tempBlue;
    }
    else
    {
      flashRed = red;
      flashGreen = green;
      flashBlue = blue;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
  else
  { // Not flashing
    flash = false;

    if (root.hasOwnProperty("color") && root["color"].hasOwnProperty("r") && root["color"].hasOwnProperty("g") && root["color"].hasOwnProperty("b"))
    {
      int tempRed = root["color"]["r"];
      int tempGreen = root["color"]["g"];
      int tempBlue = root["color"]["b"];
      red = (byte)tempRed;
      green = (byte)tempGreen;
      blue = (byte)tempBlue;
    }

    if (root.hasOwnProperty("brightness"))
    {
      int myInt = root["brightness"];
      brightness = (byte)myInt;
    }

    if (root.hasOwnProperty("transition"))
    {
      transitionTime = (int)root["transition"];
    }
    else
    {
      transitionTime = 0;
    }
  }

  return true;
}

/********************************** START SEND STATE*****************************************/
void sendState()
{
  JSONVar root;

  root["state"] = (stateOn) ? on_cmd : off_cmd;
  root["color"]["r"] = red;
  root["color"]["g"] = green;
  root["color"]["b"] = blue;

  root["brightness"] = brightness;
  root["humidity"] = (String)humValue;
  root["motion"] = (String)motionStatus;
  root["ldr"] = (String)LDR;
  root["temperature"] = (String)tempValue;
  root["heatIndex"] = (String)dht.computeHeatIndex(tempValue, humValue, IsFahrenheit);

  //Serial.println("Printing json object");
  //Serial.println(root);
  client.publish(DEVICE_DEVICE_STATE_TOPIC, JSON.stringify(root).c_str(), true);
}

/********************************** START SET COLOR *****************************************/
void setColor(int inR, int inG, int inB)
{
  analogWrite(redPin, inR);
  analogWrite(greenPin, inG);
  analogWrite(bluePin, inB);

  Serial.print("Setting LEDs: ");
  Serial.print("r: ");
  Serial.print(inR);
  Serial.print(", g: ");
  Serial.print(inG);
  Serial.print(", b: ");
  Serial.println(inB);
}

/********************************** START RECONNECT*****************************************/
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(SENSORNAME, mqtt_user, mqtt_password))
    {
      Serial.println("connected");
      client.subscribe(DEVICE_DEVICE_COMMAND_TOPIC);
      client.subscribe(DEVICE_LED_COMMAND_TOPIC);
      client.subscribe(DEVICE_LED_BRIGHTNESS_COMMAND_TOPIC);
      client.subscribe(DEVICE_LED_RGB_COMMAND_TOPIC);
      setColor(0, 0, 0);
      sendState();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff)
{
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

/********************************** START MAIN LOOP***************************************/
void loop()
{

  ArduinoOTA.handle();

  if (!client.connected())
  {
    // reconnect();
    software_Reset();
  }
  client.loop();

  if (!inFade)
  {
    
    updateState = false;

    //PIR CODE
    pirValue = digitalRead(PIRPIN); //read state of the

    if (pirValue == LOW && pirStatus != 1)
    {
      motionStatus = "standby";
      updateState = true;
      pirStatus = 1;
    }

    else if (pirValue == HIGH && pirStatus != 2)
    {
      motionStatus = "motion detected";
      updateState = true;
      pirStatus = 2;
    }

    float newTempValue = dht.readTemperature(IsFahrenheit);
    float newHumValue = dht.readHumidity();
    delay(200);

    if (checkBoundSensor(newTempValue, tempValue, diffTEMP))
    {
      tempValue = newTempValue;
      updateState = true;
    }

    if (checkBoundSensor(newHumValue, humValue, diffHUM))
    {
      humValue = newHumValue;
      updateState = true;
    }
    int newLDR = analogRead(LDRPIN);

    if (checkBoundSensor(newLDR, LDR, diffLDR))
    {
      LDR = newLDR;
      updateState = true;
    }

    if (updateState){
      
      Serial.print("update sensors?: ");
      sendState();
      
      Serial.println("done... ");
    }
      
  }

  if (flash)
  {
    if (startFlash)
    {
      startFlash = false;
      flashStartTime = millis();
    }

    if ((millis() - flashStartTime) <= flashLength)
    {
      if ((millis() - flashStartTime) % 1000 <= 500)
      {
        setColor(flashRed, flashGreen, flashBlue);
      }
      else
      {
        setColor(0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue);
      }
    }
    else
    {
      flash = false;
      setColor(realRed, realGreen, realBlue);
    }
  }

  if (startFade)
  {
    // If we don't want to fade, skip it.
    if (transitionTime == 0)
    {
      setColor(realRed, realGreen, realBlue);

      redVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;

      startFade = false;
    }
    else
    {
      loopCount = 0;
      stepR = calculateStep(redVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);

      inFade = true;
    }
  }

  if (inFade)
  {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime)
    {
      if (loopCount <= 255)
      {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);

        setColor(redVal, grnVal, bluVal); // Write current values to LED pins

        Serial.print("Loop count: ");
        Serial.println(loopCount);
        loopCount++;
      }
      else
      {
        inFade = false;
      }
    }
  }
}

/**************************** START TRANSITION FADER *****************************************/
// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS
  The program works like this:
  Imagine a crossfade that moves the red LED from 0-10,
    the green from 0-5, and the blue from 10 to 7, in
    ten steps.
    We'd want to count the 10 steps and increase or
    decrease color values in evenly stepped increments.
    Imagine a + indicates raising a value by 1, and a -
    equals lowering it. Our 10 step fade would look like:
    1 2 3 4 5 6 7 8 9 10
  R + + + + + + + + + +
  G   +   +   +   +   +
  B     -     -     -
  The red rises from 0 to 10 in ten steps, the green from
  0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.
  In the real program, the color percentages are converted to
  0-255 values, and there are 1020 steps (255*4).
  To figure out how big a step there should be between one up- or
  down-tick of one of the LED values, we call calculateStep(),
  which calculates the absolute gap between the start and end values,
  and then divides that gap by 1020 to determine the size of the step
  between adjustments in the value.
*/
int calculateStep(int prevValue, int endValue)
{
  int step = endValue - prevValue; // What's the overall gap?
  if (step)
  {                     // If its non-zero,
    step = 255 / step; //   divide by 1020
  }

  return step;
}

/* The next function is calculateVal. When the loop value, i,
   reaches the step size appropriate for one of the
   colors, it increases or decreases the value of that color by 1.
   (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i)
{
  if ((step) && i % step == 0)
  { // If step is non-zero and its time to change a value,
    if (step > 0)
    { //   increment the value if step is positive...
      val += 1;
    }
    else if (step < 0)
    { //   ...or decrement it if step is negative
      val -= 1;
    }
  }

  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255)
  {
    val = 255;
  }
  else if (val < 0)
  {
    val = 0;
  }

  return val;
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.print("resetting");
  ESP.reset();
}
