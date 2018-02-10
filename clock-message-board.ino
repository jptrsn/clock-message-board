
/*
   MQTT Clock & Message Board v 0.2.0
   Project by James Petersen, copyright 2018. Source code and documentation available at https://github.com/jptrsn/clock-message-board

   Code has been tested on a Wemos D1 Mini and a bare ESP-07 module.

   Source code and documentation are published under a Creative Commons Attribution-ShareAlike 4.0 License
   https://creativecommons.org/licenses/by-nc/4.0/

*/
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <HTU21D.h>
#include <ArduinoOTA.h>
// https://github.com/FastLED/FastLED/wiki/Interrupt-problems
//#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include <FastLED.h>

#include "config_esp07.h"


// HARDWARE SPI
MD_Parola P = MD_Parola(CS_PIN, MAX_DEVICES);

// WiFi login parameters - network name and password
const char* ssid = CONFIG_WIFI_SSID;
const char* password = CONFIG_WIFI_PASS;

const char* mqtt_server = CONFIG_MQTT_HOST;
const char* mqtt_username = CONFIG_MQTT_USER;
const char* mqtt_password = CONFIG_MQTT_PASS;
const char* client_id = CONFIG_MQTT_CLIENT_ID;
const char* host_name = CONFIG_HOST_NAME;
const int BUFFER_SIZE = JSON_OBJECT_SIZE(10);

const char* message_topic = CONFIG_MQTT_TOPIC_MESSAGE;
const char* command_topic = CONFIG_MQTT_TOPIC_COMMAND;
const char* light_state_topic = CONFIG_MQTT_LIGHT_STATE;
const char* light_set_topic = CONFIG_MQTT_LIGHT_SET;

const char* on_cmd = CONFIG_ON_CMD;
const char* off_cmd = CONFIG_OFF_CMD;
const char* stripEffect = "solid";
String stripEffectString = "solid";
String oldstripEffectString = "solid";
bool revertToOldStripEffect = false;
bool revertToOff = false;

/*********************************** FastLED Defintions ********************************/
byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

byte stripRed = 255;
byte stripGreen = 255;
byte stripBlue = 255;
byte stripBrightness = 255;

/******************************** GLOBALS for fade/flash *******************************/
bool stateOn = false;
bool startFade = false;
bool onbeforeflash = false;
unsigned long lastLoop = 0;
int transitionTime = 0;
int stripEffectSpeed = 0;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB;
int rdVal, grnVal, bluVal;

bool flash = false;
bool startFlash = false;
int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = stripRed;
byte flashGreen = stripGreen;
byte flashBlue = stripBlue;
byte flashBrightness = stripBrightness;

WiFiClient espClient;
PubSubClient client(espClient);

HTU21D sensor(HTU21D_RES_RH12_TEMP14);
//SHT21 sensor;
float publishedHumidity = 0;
float publishedTemperature = 0;

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
time_t getNtpTime();
void sendNTPpacket(IPAddress &address);
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t local;

uint8_t frameDelay = FRAME_DELAY_DEFAULT;  // default frame delay value
textEffect_t  effects[] =
{
  PA_PRINT,             // 0
  PA_SCAN_HORIZ,        // 1
  PA_SCROLL_LEFT,       // 2
  PA_WIPE,              // 3
  PA_SCROLL_UP_LEFT,    // 4
  PA_SCROLL_UP,         // 5
  PA_FADE,              // 6
  PA_OPENING_CURSOR,    // 7
  PA_GROW_UP,           // 8
  PA_SCROLL_UP_RIGHT,   // 9
  PA_BLINDS,            // 10
  PA_CLOSING,           // 11
  PA_GROW_DOWN,         // 12
  PA_SCAN_VERT,         // 13
  PA_SCROLL_DOWN_LEFT,  // 14
  PA_WIPE_CURSOR,       // 15
  PA_DISSOLVE,          // 16
  PA_MESH,              // 17
  PA_OPENING,           // 18
  PA_CLOSING_CURSOR,    // 19
  PA_SCROLL_DOWN_RIGHT, // 20
  PA_SCROLL_RIGHT,      // 21
  PA_SLICE,             // 22
  PA_SCROLL_DOWN,       // 23
};

// Global message buffers shared by Wifi and Scrolling functions
#define BUF_SIZE CONFIG_BUF_SIZE
char curMessage[BUF_SIZE];
char newMessage[BUF_SIZE];
bool newMessageAvailable = false;
char currentTime[BUF_SIZE];


// Global variables that need to be available to code

textEffect_t scrollEffect = CONFIG_DEFAULT_EFFECT;
textEffect_t effectIn = effects[2];
textEffect_t effectOut = effects[2];
uint16_t messagePause = 0;

unsigned long lastSensorUpdate;
unsigned long lastMessageDisplayed;
bool updateLastMessageDisplayed;

byte lastMinute;
byte machineState = 1;
byte repeatMessage = MESSAGE_REPEAT_DEFAULT;
byte i = 0;
byte clockBrightness = CLOCK_BRIGHTNESS_DEFAULT;
byte messageBrightness = MESSAGE_BRIGHTNESS_DEFAULT;
int delayBetweenMessages = MESSAGE_DELAY_DEFAULT;

void setup() {
  delay(500);

  pinMode(LED_PIN, OUTPUT);

  lastSensorUpdate = millis();
  lastMessageDisplayed = millis();
  if (DEBUG) {
    Serial.begin(115200);
  }

  configureLedStrip();

  P.begin();
  P.displayClear();
  P.setIntensity(clockBrightness);
  P.setTextEffect(PA_SCROLL_UP, PA_SCROLL_DOWN);

  P.displayScroll(curMessage, PA_CENTER, scrollEffect, FRAME_DELAY_DEFAULT);
  P.displayReset();
  curMessage[0] = newMessage[0] = '\0';

  // Connect to and initialise WiFi network
  PRINT("\nConnecting to ", ssid);
  sprintf(curMessage, "%s %s", "Connecting to ", ssid);

  WiFi.hostname(host_name);
  WiFi.begin(ssid, password);
  waitForMessageComplete(true);

  handleWifi(false);

  configureOTA();

  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(120);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  sensor.begin(4,5);

  PRINTS("\nWiFi connected");
  getNtpTime();
  updateCurrentTime();

  P.displayReset();
  sprintf(curMessage, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  PRINT("\nAssigned IP ", curMessage);
  waitForMessageComplete(false);

  digitalWrite(LED_PIN, HIGH);
}

void loop() {

  ArduinoOTA.handle();

  if (client.connected()) {
    client.loop();
  }

  handleLedStrip();

  if (P.displayAnimate()) {

    // Ensure WiFi is connected. Enable error output to the MAX7219 display.
    handleWifi(true);

    // If we're not connected to the MQTT message topic, attempt to reconnect
    if (!client.connected()) {
      reconnect();
    } else {
      client.loop();
    }

    if (revertToOldStripEffect) {
      stripEffectString = oldstripEffectString;
      if (revertToOff) {
        stateOn = false;
        revertToOff = false;
      }
      revertToOldStripEffect = false;
    }

    // If the flag is set to update the timestamp of the last message display, do so. If there's a new message
    // to display, do not reset the flag
    if (updateLastMessageDisplayed && !newMessageAvailable) {
      lastMessageDisplayed = millis();
      updateLastMessageDisplayed = false;
    }

    // When a new message is available and the display is in the correct state, update the display with the new message
//    if (newMessageAvailable && machineState != 3 && machineState != 1) {
//      PRINTLN("---------------------------- newMessageAvailable flag set ----------------------------");
//      strcpy(curMessage, newMessage);
//      machineState = 3;
//      i = 0;
//    }

    // Run state machine evaluations
    handlemachineState();
  }
}

/***********************************************************************************/
//                              State Machine methods

void handlemachineState() {
  // Write the machine state to the serial monitor if it is not in static-display mode (state 0)
  if (DEBUG && machineState) {
    PRINTS("machineState: ");
    PRINTLN(machineState);
  }

  switch (machineState) {
    case 0: { // Clock is showing the current time
        if (i > 0 && millis() - lastMessageDisplayed > delayBetweenMessages && !newMessageAvailable) {
          PRINTLN("---");
          PRINT("---------------- Message repeat ", i);
          PRINTLN(" ----------------");
          machineState = 3;
        } else if (newMessageAvailable && machineState != 1) {
          strcpy(curMessage, newMessage);
          machineState = 3;
        } else {
          if (clockMinuteChanged(lastMinute)) {
            machineState = 5;
          } else if (USE_SENSOR) {
            readSensor();
          }
        }
        break;
      }
    case 1: { // Clock is transitioning from hidden to showing current time
        showTime();
        machineState = 0;
        break;
      }
    case 3: { // Clock is transitioning from showing current time to hidden, in order to display a new message
        PRINT("repeatMessage: ", repeatMessage);
        PRINTLN(" ");
        if (i < repeatMessage) {
          hideTime();
          newMessageAvailable = false;
          machineState = 4;
        } else {
          repeatMessage = MESSAGE_REPEAT_DEFAULT;
          frameDelay = FRAME_DELAY_DEFAULT;
          messagePause = MESSAGE_PAUSE_DEFAULT;
          P.setSpeed(frameDelay);
          i = 0;
          machineState = 0;
          PRINTLN("Do not display");
        }
        break;
      }
    case 4: { // New message received, needs to be animated
        PRINTS("display message - ");
        PRINTLN(curMessage);
        //        P.displayScroll(curMessage, PA_CENTER, PA_SCROLL_LEFT, frameDelay);
        P.displayText(curMessage, PA_CENTER, frameDelay, messagePause, effectIn, effectOut);
        i++;
        updateLastMessageDisplayed = true;
        machineState = 1;
        break;
      }
    case 5: { // Clock is displaying exit animation
        hideTime();
        machineState = 6;
        break;
      }
    case 6: { // Clock display is blank, value can be changed
        updateCurrentTime();
        machineState = 1;
        break;
      }
  }
}

/***********************************************************************************/
//                              Clock methods

bool clockMinuteChanged(byte& thisMinute) {
  if (minute() != thisMinute) {
    //    PRINTLN("Time changed");
    thisMinute = minute();
    return true;
  }
  return false;
}

void updateCurrentTime() {
  //  PRINTLN("updateCurrentTime");
  local = myTZ.toLocal(now(), &tcr);
  sprintf(currentTime, "%d:%02d", hour(local), minute(local));
  PRINTLN(currentTime);
  //  getNtpTime();
}

// Animate the current time off the display
void hideTime() {
  P.displayText(currentTime, PA_CENTER, FRAME_DELAY_DEFAULT * 2, 0, PA_PRINT, PA_SCROLL_UP);
}

// Animate the current time onto the display
void showTime() {
  //  PRINTLN("showTime");
  P.displayText(currentTime, PA_CENTER, FRAME_DELAY_DEFAULT * 2, 500, PA_SCROLL_UP);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  // get a random server from the pool
  WiFi.hostByName(NTP_SERVER_POOL, ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL;
    }
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

/***********************************************************************************/
//                              HTU21D methods

void readSensor() {
  if (millis() - lastSensorUpdate < CONFIG_SENSOR_DELAY) {
    return;
  }
  PRINTLN("readSensor");
  // HTU21D library
  float humidity = sensor.readCompensatedHumidity() + HUMIDITY_CORRECTION;
  float celsTemp = sensor.readTemperature() + TEMPERATURE_CORRECTION;

  postData(celsTemp, humidity);
  lastSensorUpdate = millis();
}

/***********************************************************************************/
//                              MQTT methods

void callback(char* topic, byte* payload, unsigned int length) {
  PRINTS("New message arrived: ");
  PRINTLN(topic);
  if (strcmp(topic, light_set_topic) == 0) {
    PRINTLN("light callback");
    callbackLight(topic, payload, length);
  } else {
    char message[length + 1];
    for (int b = 0; b < length; b++) {
      message[b] = (char)payload[b];
    }
    message[length] = '\0';

    newMessageAvailable = processJson(message);
    PRINTS("New message arrived: ");
    PRINTLN(message);

  }
}

bool processJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    PRINTS("parseObject() failed");
    return false;
  }

  if (root.containsKey("message")) {
    strcpy(newMessage, root["message"]);
    PRINTS("new message ");
    PRINTLN(newMessage);
  }

  if (root.containsKey("repeat")) {
    PRINTS("---------------------- root.repeat ");
    repeatMessage = root["repeat"].as<int>();
    PRINTLN(repeatMessage);
  }

  bool hasPause = false;

  if (root.containsKey("textEffect")) {
    PRINTLN("textEffect");
    JsonObject& textEffectObj = root["textEffect"];
    if (textEffectObj.containsKey("in")) {
      effectIn = effects[textEffectObj["in"].as<int>()];
    } else {
      effectIn = effects[2]; // Scroll from right to left
    }

    if (textEffectObj.containsKey("out")) {
      effectOut = effects[textEffectObj["out"].as<int>()];
    } else {
      effectOut = effects[2]; // Scroll from right to left
    }

    if (textEffectObj.containsKey("pause")) {
      messagePause = textEffectObj["pause"].as<int>();
      hasPause = true;
    }

  } else {
    effectIn = effects[2];
    effectOut = effects[2];
  }
  P.setTextEffect(effectIn, effectOut);

  if (root.containsKey("frameDelay")) {
    frameDelay = root["frameDelay"].as<int>();
  } else {
    frameDelay = FRAME_DELAY_DEFAULT;
  }

  if (!hasPause) {
    messagePause = MESSAGE_PAUSE_DEFAULT;
  }

  return true;
}

void reconnect() {
  if (!client.connected()) {
    PRINTS("Attempting MQTT connection...");
    client.connect(client_id, mqtt_username, mqtt_password);
    
    P.displayScroll("Connecting to MQTT", PA_CENTER, scrollEffect, FRAME_DELAY_DEFAULT);
    waitForMessageComplete(false);
    
    // Attempt to connect
    if (client.connected()) {
      PRINTS("connected");
      client.subscribe(message_topic);
      client.subscribe(command_topic);
      client.subscribe(light_set_topic);
      
      P.displayScroll("MQTT connected", PA_CENTER, scrollEffect, FRAME_DELAY_DEFAULT);
      waitForMessageComplete(false);
      
    } else {
      PRINTS("failed, rc=");
      PRINTLN(client.state());
      machineState = 6;
    }
  }
}

void postData(float tempCelsius, float humidityPercent) {
  if (!client.connected()) {
    reconnect();
  } else {
    char buf[10];
    if (tempCelsius != publishedTemperature) {
      dtostrf(tempCelsius, 0, 0, buf);
      client.publish(CONFIG_MQTT_TOPIC_TEMP, buf);
      publishedTemperature = tempCelsius;
    }

    if (humidityPercent != publishedHumidity) {
      dtostrf(humidityPercent, 0, 0, buf);
      client.publish(CONFIG_MQTT_TOPIC_HUMIDITY, buf);
      publishedHumidity = humidityPercent;
    }
  }
}


/***********************************************************************************/
//                              Utility methods

void handleWifi(bool displayErr) {
  char* errMessage;
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED)
  {

    if (++count > 10) {
      ESP.restart();
    }

    digitalWrite(LED_PIN, LOW);

    char* err = err2Str(WiFi.status());
    PRINT("\n", err);
    PRINTLN("");

    if (P.displayAnimate() && displayErr) {
      sprintf(errMessage, "Error: %s", err);
      P.displayScroll(errMessage, PA_CENTER, scrollEffect, FRAME_DELAY_DEFAULT);
    }
    yield();

    digitalWrite(LED_PIN, HIGH);

    if (WiFi.status() != WL_CONNECTED) {
      PRINTLN("Retrying in .5s");
      delay(500);
    }

  }
}

char *err2Str(wl_status_t code) {
  switch (code)
  {
    case WL_IDLE_STATUS:    return ("IDLE");           break; // WiFi is in process of changing between statuses
    case WL_NO_SSID_AVAIL:  return ("NO_SSID_AVAIL");  break; // case configured SSID cannot be reached
    case WL_CONNECTED:      return ("CONNECTED");      break; // successful connection is established
    case WL_CONNECT_FAILED: return ("WRONG_PASSWORD"); break; // password is incorrect
    case WL_DISCONNECTED:   return ("CONNECT_FAILED"); break; // module is not configured in station mode
    default: return ("??");
  }
}

void waitForMessageComplete(bool blinkStatusLed) {
  int a = 0;
  PRINTLN(" ");
  bool state = 0;
  while (!P.displayAnimate()) {
    a++;
    if (a % 2500 == 0) {
      PRINTS(".");
      state = ! state;
      if (blinkStatusLed) {
        digitalWrite(LED_PIN, state);
      }
    }
    yield();
    ArduinoOTA.handle();
  }
}

void configureOTA() {
  ArduinoOTA.onStart([]() {
    PRINTS("OTA Start");
    P.displayScroll("Updating Firmware", PA_CENTER, scrollEffect, FRAME_DELAY_DEFAULT);
    waitForMessageComplete(false);
  });
  ArduinoOTA.onEnd([]() {
    PRINTS("\nEnd");
    P.displayScroll("Firmware Updated!", PA_CENTER, scrollEffect, FRAME_DELAY_DEFAULT);
    waitForMessageComplete(false);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char* m = "-";
    if (P.displayAnimate()) {
      int percent = (progress / (total / 100));
      char* prog = "-/|\\";
      m[0] = prog[percent % 4];
      P.displayText(m, PA_CENTER, 10, 10, PA_PRINT);
    }
    PRINT("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    PRINT("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) PRINTS("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) PRINTS("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) PRINTS("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) PRINTS("Receive Failed");
    else if (error == OTA_END_ERROR) PRINTS("End Failed");
  });
  ArduinoOTA.setHostname(host_name);
  ArduinoOTA.begin();
}

/***********************************************************************************/
//                              WS2812 Methods

/********************************** GLOBALS for EFFECTS ******************************/
//RAINBOW
uint8_t thishue = 0;                                          // Starting hue value.
uint8_t deltahue = 10;

//CANDYCANE
CRGBPalette16 currentPalettestriped; //for Candy Cane
CRGBPalette16 gPal; //for fire

//NOISE
static uint16_t dist;         // A random number for our noise generator.
uint16_t scale = 30;          // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
uint8_t maxChanges = 48;      // Value for blending between palettes.
CRGBPalette16 targetPalette(OceanColors_p);
CRGBPalette16 currentPalette(CRGB::Black);

//TWINKLE
#define DENSITY     80
int twinklecounter = 0;

//RIPPLE
uint8_t colour;                                               // Ripple colour is randomized.
int center = 0;                                               // Center of the current ripple.
int step = -1;                                                // -1 is the initializing step.
uint8_t myfade = 255;                                         // Starting brightness.
#define maxsteps 16                                           // Case statement wouldn't allow a variable.
uint8_t bgcol = 0;                                            // Background colour rotates.
int thisdelay = 20;                                           // Standard delay value.

//DOTS
uint8_t   count =   0;                                        // Count up to 255 and then reverts to 0
uint8_t fadeval = 224;                                        // Trail behind the LED's. Lower => faster fade.
uint8_t bpm = 30;

//LIGHTNING
uint8_t frequency = 50;                                       // controls the interval between strikes
uint8_t flashes = 8;                                          //the upper limit of flashes per strike
unsigned int dimmer = 1;
uint8_t ledstart;                                             // Starting location of a flash
uint8_t ledlen;
int lightningcounter = 0;

//FUNKBOX
int idex = 0;                //-LED INDEX (0 to WS_LENGTH-1
int TOP_INDEX = int(WS_LENGTH / 2);
int thissat = 255;           //-FX LOOPS DELAY VAR
uint8_t thishuepolice = 0;
int antipodal_index(int i) {
  int iN = i + TOP_INDEX;
  if (i >= TOP_INDEX) {
    iN = ( i + TOP_INDEX ) % WS_LENGTH;
  }
  return iN;
}

//FIRE
#define COOLING  55
#define SPARKING 120
bool gReverseDirection = false;

//BPM
uint8_t gHue = 0;

struct CRGB leds[WS_LENGTH];

/********************************** START SETUP*****************************************/
void configureLedStrip() {

  FastLED.addLeds<CHIPSET, WS_PIN, COLOR_ORDER>(leds, WS_LENGTH);

  setupStripedPalette( CRGB::Red, CRGB::Red, CRGB::White, CRGB::White); //for CANDY CANE

  gPal = HeatColors_p; //for FIRE
}

void callbackLight(char* topic, byte* payload, unsigned int len) {
  digitalWrite(LED_PIN, LOW);
  char message[len + 1];
  for (int i = 0; i < len; i++) {
    message[i] = (char)payload[i];
  }
  message[len] = '\0';

  if (!processLightJson(message)) {
    return;
  }

  if (stateOn) {

    realRed = map(stripRed, 0, 255, 0, stripBrightness);
    realGreen = map(stripGreen, 0, 255, 0, stripBrightness);
    realBlue = map(stripBlue, 0, 255, 0, stripBrightness);
  }
  else {

    realRed = 0;
    realGreen = 0;
    realBlue = 0;
  }

  startFade = true;
  inFade = false; // Kill the current fade

  sendState();
  digitalWrite(LED_PIN, HIGH);
}

bool processLightJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    return false;
  }

  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      stateOn = false;
      onbeforeflash = false;
    }
  }

  // If "flash" is included, treat RGB and brightness differently
  if (root.containsKey("flash")) {
    flashLength = (int)root["flash"] * 1000;

    oldstripEffectString = stripEffectString;

    if (root.containsKey("brightness")) {
      flashBrightness = root["brightness"];
    }
    else {
      flashBrightness = stripBrightness;
    }

    if (root.containsKey("color")) {
      flashRed = root["color"]["r"];
      flashGreen = root["color"]["g"];
      flashBlue = root["color"]["b"];
    }
    else {
      flashRed = stripRed;
      flashGreen = stripGreen;
      flashBlue = stripBlue;
    }

    if (root.containsKey("effect")) {
      stripEffect = root["effect"];
      stripEffectString = stripEffect;
      twinklecounter = 0; //manage twinklecounter
    }

    if (root.containsKey("transition")) {
      transitionTime = root["transition"];
    }
    else if ( stripEffectString == "solid") {
      transitionTime = 1;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
  else { // Not flashing
    flash = false;

    if (stateOn) {   //if the light is turned on and the light isn't flashing
      onbeforeflash = true;
    }

    if (root.containsKey("color")) {
      stripRed = root["color"]["r"];
      stripGreen = root["color"]["g"];
      stripBlue = root["color"]["b"];
    }

    if (root.containsKey("color_temp")) {
      //temp comes in as mireds, need to convert to kelvin then to RGB
      int color_temp = root["color_temp"];
      unsigned int kelvin  = MILLION / color_temp;

      temp2rgb(kelvin);

    }

    if (root.containsKey("brightness")) {
      stripBrightness = root["brightness"];
    }

    if (root.containsKey("effect")) {
      if (root.containsKey("notify_effect")) {
        oldstripEffectString = stripEffectString;
        revertToOldStripEffect = true;
        if (!stateOn) {
          revertToOff = true;
          stateOn = true;
        }
      }
      stripEffect = root["effect"];
      stripEffectString = stripEffect;
      twinklecounter = 0; //manage twinklecounter
    }

    if (root.containsKey("transition")) {
      transitionTime = root["transition"];
    }
    else if ( stripEffectString == "solid") {
      transitionTime = 0;
    }

  }

  return true;
}

void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["state"] = (stateOn) ? on_cmd : off_cmd;
  JsonObject& color = root.createNestedObject("color");
  color["r"] = stripRed;
  color["g"] = stripGreen;
  color["b"] = stripBlue;

  root["brightness"] = stripBrightness;
  root["effect"] = stripEffectString.c_str();


  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  client.publish(light_state_topic, buffer, true);
}

void setColor(int inR, int inG, int inB) {
  for (int i = 0; i < WS_LENGTH; i++) {
    leds[i].red   = inR;
    leds[i].green = inG;
    leds[i].blue  = inB;
  }

  FastLED.show();
}

void handleLedStrip() {
  //EFFECT BPM
  if (stripEffectString == "bpm") {
    uint8_t BeatsPerMinute = 62;
    CRGBPalette16 palette = PartyColors_p;
    uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
    for ( int i = 0; i < WS_LENGTH; i++) { //9948
      leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
    }
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 30;
    }
    showleds();
  }


  //EFFECT Candy Cane
  if (stripEffectString == "candy cane") {
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* higher = faster motion */
    fill_palette( leds, WS_LENGTH,
                  startIndex, 16, /* higher = narrower stripes */
                  currentPalettestriped, 255, LINEARBLEND);
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 0;
    }
    showleds();
  }


  //EFFECT CONFETTI
  if (stripEffectString == "confetti" ) {
    fadeToBlackBy( leds, WS_LENGTH, 25);
    int pos = random16(WS_LENGTH);
    leds[pos] += CRGB(realRed + random8(64), realGreen, realBlue);
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 30;
    }
    showleds();
  }


  //EFFECT CYCLON RAINBOW
  if (stripEffectString == "cyclon rainbow") {                    //Single Dot Down
    static uint8_t hue = 0;
    // First slide the led in one direction
    for (int i = 0; i < WS_LENGTH; i++) {
      // Set the i'th led to red
      leds[i] = CHSV(hue++, 255, 255);
      // Show the leds
      showleds();
      // now that we've shown the leds, reset the i'th led to black
      // leds[i] = CRGB::Black;
      fadeall();
      // Wait a little bit before we loop around and do it again
      delay(10);
    }
    for (int i = (WS_LENGTH) - 1; i >= 0; i--) {
      // Set the i'th led to red
      leds[i] = CHSV(hue++, 255, 255);
      // Show the leds
      showleds();
      // now that we've shown the leds, reset the i'th led to black
      // leds[i] = CRGB::Black;
      fadeall();
      // Wait a little bit before we loop around and do it again
      delay(10);
    }
  }


  //EFFECT DOTS
  if (stripEffectString == "dots") {
    uint8_t inner = beatsin8(bpm, WS_LENGTH / 4, WS_LENGTH / 4 * 3);
    uint8_t outer = beatsin8(bpm, 0, WS_LENGTH - 1);
    uint8_t middle = beatsin8(bpm, WS_LENGTH / 3, WS_LENGTH / 3 * 2);
    leds[middle] = CRGB::Purple;
    leds[inner] = CRGB::Blue;
    leds[outer] = CRGB::Aqua;
    nscale8(leds, WS_LENGTH, fadeval);

    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 30;
    }
    showleds();
  }


  //EFFECT FIRE
  if (stripEffectString == "fire") {
    Fire2012WithPalette();
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 150;
    }
    showleds();
  }

  random16_add_entropy( random8());


  //EFFECT Glitter
  if (stripEffectString == "glitter") {
    fadeToBlackBy( leds, WS_LENGTH, 20);
    addGlitterColor(80, realRed, realGreen, realBlue);
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 30;
    }
    showleds();
  }


  //EFFECT JUGGLE
  if (stripEffectString == "juggle" ) {                           // eight colored dots, weaving in and out of sync with each other
    fadeToBlackBy(leds, WS_LENGTH, 20);
    for (int i = 0; i < 8; i++) {
      leds[beatsin16(i + 7, 0, WS_LENGTH - 1  )] |= CRGB(realRed, realGreen, realBlue);
    }
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 130;
    }
    showleds();
  }


  //EFFECT LIGHTNING
  if (stripEffectString == "lightning") {
    twinklecounter = twinklecounter + 1;                     //Resets strip if previous animation was running
    if (twinklecounter < 2) {
      FastLED.clear();
      FastLED.show();
    }
    ledstart = random8(WS_LENGTH);           // Determine starting location of flash
    ledlen = random8(WS_LENGTH - ledstart);  // Determine length of flash (not to go beyond WS_LENGTH-1)
    for (int flashCounter = 0; flashCounter < random8(3, flashes); flashCounter++) {
      if (flashCounter == 0) dimmer = 5;    // the brightness of the leader is scaled down by a factor of 5
      else dimmer = random8(1, 3);          // return strokes are brighter than the leader
      fill_solid(leds + ledstart, ledlen, CHSV(255, 0, 255 / dimmer));
      showleds();    // Show a section of LED's
      delay(random8(4, 10));                // each flash only lasts 4-10 milliseconds
      fill_solid(leds + ledstart, ledlen, CHSV(255, 0, 0)); // Clear the section of LED's
      showleds();
      if (flashCounter == 0) delay (130);   // longer delay until next flash after the leader
      delay(50 + random8(100));             // shorter delay between strokes
    }
    delay(random8(frequency) * 100);        // delay between strikes
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 0;
    }
    showleds();
  }


  //EFFECT POLICE ALL
  if (stripEffectString == "police all") {                 //POLICE LIGHTS (TWO COLOR SOLID)
    idex++;
    if (idex >= WS_LENGTH) {
      idex = 0;
    }
    int idexR = idex;
    int idexB = antipodal_index(idexR);
    int thathue = (thishuepolice + 160) % 255;
    leds[idexR] = CHSV(thishuepolice, thissat, 255);
    leds[idexB] = CHSV(thathue, thissat, 255);
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 30;
    }
    showleds();
  }

  //EFFECT POLICE ONE
  if (stripEffectString == "police one") {
    idex++;
    if (idex >= WS_LENGTH) {
      idex = 0;
    }
    int idexR = idex;
    int idexB = antipodal_index(idexR);
    int thathue = (thishuepolice + 160) % 255;
    for (int i = 0; i < WS_LENGTH; i++ ) {
      if (i == idexR) {
        leds[i] = CHSV(thishuepolice, thissat, 255);
      }
      else if (i == idexB) {
        leds[i] = CHSV(thathue, thissat, 255);
      }
      else {
        leds[i] = CHSV(0, 0, 0);
      }
    }
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 30;
    }
    showleds();
  }


  //EFFECT RAINBOW
  if (stripEffectString == "rainbow") {
    // FastLED's built-in rainbow generator
    static uint8_t starthue = 0;    thishue++;
    fill_rainbow(leds, WS_LENGTH, thishue, deltahue);
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 130;
    }
    showleds();
  }


  //EFFECT RAINBOW WITH GLITTER
  if (stripEffectString == "rainbow with glitter") {               // FastLED's built-in rainbow generator with Glitter
    static uint8_t starthue = 0;
    thishue++;
    fill_rainbow(leds, WS_LENGTH, thishue, deltahue);
    addGlitter(80);
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 130;
    }
    showleds();
  }


  //EFFECT SIENLON
  if (stripEffectString == "sinelon") {
    fadeToBlackBy( leds, WS_LENGTH, 20);
    int pos = beatsin16(13, 0, WS_LENGTH - 1);
    leds[pos] += CRGB(realRed, realGreen, realBlue);
    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 150;
    }
    showleds();
  }


  //EFFECT TWINKLE
  if (stripEffectString == "twinkle") {
    twinklecounter = twinklecounter + 1;
    if (twinklecounter < 2) {                               //Resets strip if previous animation was running
      FastLED.clear();
      FastLED.show();
    }
    const CRGB lightcolor(8, 7, 1);
    for ( int i = 0; i < WS_LENGTH; i++) {
      if ( !leds[i]) continue; // skip black pixels
      if ( leds[i].r & 1) { // is red odd?
        leds[i] -= lightcolor; // darken if red is odd
      } else {
        leds[i] += lightcolor; // brighten if red is even
      }
    }
    if ( random8() < DENSITY) {
      int j = random16(WS_LENGTH);
      if ( !leds[j] ) leds[j] = lightcolor;
    }

    if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
      transitionTime = 0;
    }
    showleds();
  }


  EVERY_N_MILLISECONDS(10) {

    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);  // FOR NOISE ANIMATIon
    {
      gHue++;
    }

    //EFFECT NOISE
    if (stripEffectString == "noise") {
      for (int i = 0; i < WS_LENGTH; i++) {                                     // Just onE loop to fill up the LED array as all of the pixels change.
        uint8_t index = inoise8(i * scale, dist + i * scale) % 255;            // Get a value from the noise function. I'm using both x and y axis.
        leds[i] = ColorFromPalette(currentPalette, index, 255, LINEARBLEND);   // With that value, look up the 8 bit colour palette value and assign it to the current LED.
      }
      dist += beatsin8(10, 1, 4);                                              // Moving along the distance (that random number we started out with). Vary it a bit with a sine wave.
      // In some sketches, I've used millis() instead of an incremented counter. Works a treat.
      if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
        transitionTime = 0;
      }
      showleds();
    }

    //EFFECT RIPPLE
    if (stripEffectString == "ripple") {
      for (int i = 0; i < WS_LENGTH; i++) leds[i] = CHSV(bgcol++, 255, 15);  // Rotate background colour.
      switch (step) {
        case -1:                                                          // Initialize ripple variables.
          center = random(WS_LENGTH);
          colour = random8();
          step = 0;
          break;
        case 0:
          leds[center] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
          step ++;
          break;
        case maxsteps:                                                    // At the end of the ripples.
          step = -1;
          break;
        default:                                                             // Middle of the ripples.
          leds[(center + step + WS_LENGTH) % WS_LENGTH] += CHSV(colour, 255, myfade / step * 2);   // Simple wrap from Marc Miller
          leds[(center - step + WS_LENGTH) % WS_LENGTH] += CHSV(colour, 255, myfade / step * 2);
          step ++;                                                         // Next step.
          break;
      }
      if ((transitionTime == 0 or transitionTime == NULL) && !revertToOldStripEffect) {
        transitionTime = 30;
      }
      showleds();
    }

  }


  EVERY_N_SECONDS(5) {
    targetPalette = CRGBPalette16(CHSV(random8(), 255, random8(128, 255)), CHSV(random8(), 255, random8(128, 255)), CHSV(random8(), 192, random8(128, 255)), CHSV(random8(), 255, random8(128, 255)));
  }

  //FLASH AND FADE SUPPORT
  if (flash) {
    if (startFlash) {
      startFlash = false;
      flashStartTime = millis();
    }

    if ((millis() - flashStartTime) <= flashLength) {
      if ((millis() - flashStartTime) % 1000 <= 500) {
        setColor(flashRed, flashGreen, flashBlue);
      }
      else {
        setColor(0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue);
      }
    }
    else {
      flash = false;
      stripEffectString = oldstripEffectString;
      if (onbeforeflash) { //keeps light off after flash if light was originally off
        setColor(realRed, realGreen, realBlue);
      }
      else {
        stateOn = false;
        setColor(0, 0, 0);
        sendState();
      }
    }
  }

  if (startFade && stripEffectString == "solid") {
    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      setColor(realRed, realGreen, realBlue);

      rdVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;

      startFade = false;
    }
    else {
      loopCount = 0;
      stepR = calculateStep(rdVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);

      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
      if (loopCount <= 1020) {
        lastLoop = now;

        rdVal = calculateVal(stepR, rdVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);

        if (stripEffectString == "solid") {
          setColor(rdVal, grnVal, bluVal); // Write current values to LED pins
        }
        loopCount++;
      }
      else {
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
int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero,
    step = 1020 / step;          //   divide by 1020
  }

  return step;
}
/* The next function is calculateVal. When the loop value, i,
   reaches the step size appropriate for one of the
   colors, it increases or decreases the value of that color by 1.
   (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i) {
  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;
    }
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    }
  }

  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  }
  else if (val < 0) {
    val = 0;
  }

  return val;
}



/**************************** START STRIPLED PALETTE *****************************************/
void setupStripedPalette( CRGB A, CRGB AB, CRGB B, CRGB BA) {
  currentPalettestriped = CRGBPalette16(
                            A, A, A, A, A, A, A, A, B, B, B, B, B, B, B, B
                            //    A, A, A, A, A, A, A, A, B, B, B, B, B, B, B, B
                          );
}



/********************************** START FADE************************************************/
void fadeall() {
  for (int i = 0; i < WS_LENGTH; i++) {
    leds[i].nscale8(250);  //for CYCLon
  }
}



/********************************** START FIRE **********************************************/
void Fire2012WithPalette()
{
  // Array of temperature readings at each simulation cell
  static byte heat[WS_LENGTH];

  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < WS_LENGTH; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / WS_LENGTH) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = WS_LENGTH - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if ( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160, 255) );
  }

  // Step 4.  Map from heat cells to LED colors
  for ( int j = 0; j < WS_LENGTH; j++) {
    // Scale the heat value from 0-255 down to 0-240
    // for best results with color palettes.
    byte colorindex = scale8( heat[j], 240);
    CRGB color = ColorFromPalette( gPal, colorindex);
    int pixelnumber;
    if ( gReverseDirection ) {
      pixelnumber = (WS_LENGTH - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }
}



/********************************** START ADD GLITTER *********************************************/
void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(WS_LENGTH) ] += CRGB::White;
  }
}



/********************************** START ADD GLITTER COLOR ****************************************/
void addGlitterColor( fract8 chanceOfGlitter, int stripRed, int stripGreen, int stripBlue)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(WS_LENGTH) ] += CRGB(stripRed, stripGreen, stripBlue);
  }
}



/********************************** START SHOW LEDS ***********************************************/
void showleds() {

  delay(1);

  if (stateOn) {
    FastLED.setBrightness(stripBrightness);  //EXECUTE EFFECT COLOR
    FastLED.show();
    if (transitionTime > 0 && transitionTime < 130) {  //Sets animation speed based on receieved value
      FastLED.delay(1000 / transitionTime);
      //delay(10*transitionTime);
    }
  }
  else if (startFade) {
    setColor(0, 0, 0);
    startFade = false;
  }
}
void temp2rgb(unsigned int kelvin) {
  int tmp_internal = kelvin / 100.0;

  // red
  if (tmp_internal <= 66) {
    stripRed = 255;
  } else {
    float tmp_stripRed = 329.698727446 * pow(tmp_internal - 60, -0.1332047592);
    if (tmp_stripRed < 0) {
      stripRed = 0;
    } else if (tmp_stripRed > 255) {
      stripRed = 255;
    } else {
      stripRed = tmp_stripRed;
    }
  }

  // green
  if (tmp_internal <= 66) {
    float tmp_stripGreen = 99.4708025861 * log(tmp_internal) - 161.1195681661;
    if (tmp_stripGreen < 0) {
      stripGreen = 0;
    } else if (tmp_stripGreen > 255) {
      stripGreen = 255;
    } else {
      stripGreen = tmp_stripGreen;
    }
  } else {
    float tmp_stripGreen = 288.1221695283 * pow(tmp_internal - 60, -0.0755148492);
    if (tmp_stripGreen < 0) {
      stripGreen = 0;
    } else if (tmp_stripGreen > 255) {
      stripGreen = 255;
    } else {
      stripGreen = tmp_stripGreen;
    }
  }

  // blue
  if (tmp_internal >= 66) {
    stripBlue = 255;
  } else if (tmp_internal <= 19) {
    stripBlue = 0;
  } else {
    float tmp_stripBlue = 138.5177312231 * log(tmp_internal - 10) - 305.0447927307;
    if (tmp_stripBlue < 0) {
      stripBlue = 0;
    } else if (tmp_stripBlue > 255) {
      stripBlue = 255;
    } else {
      stripBlue = tmp_stripBlue;
    }
  }
}
