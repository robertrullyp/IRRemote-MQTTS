#define UID              "ESP32DevKit-v1-"    //sesuaikan unique id untuk perangkat
// #define UID              "NodeMCU-v3-"    //sesuaikan unique id untuk perangkat
#define USE_TLS    // uncomment (hapus tanda // di awal perintah) jika koneksi mqtt menggunakan SSL/TLS
#define DECODE_AC
#if !( defined(ESP8266) ||  defined(ESP32) )
#error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif
#define ESP_WIFIMANAGER_VERSION_MIN_TARGET      "ESP_WiFiManager v1.12.0"
#define ESP_WIFIMANAGER_VERSION_MIN             1012000
#define _WIFIMGR_LOGLEVEL_    1     // Use from 0 to 4. Higher number, more debugging messages and memory usage.
// To not display stored SSIDs and PWDs on Config Portal, select false. Default is true
// Even the stored Credentials are not display, just leave them all blank to reconnect and reuse the stored Credentials
//#define DISPLAY_STORED_CREDENTIALS_IN_CP        false
#include <Arduino.h>
#include <assert.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <AHTxx.h>
// #include <OneButton.h>          // for button
#include <FS.h>
#include <ArduinoJson.h>        // Now support ArduinoJson 6.0.0+ ( tested with v6.15.2 to v6.16.1 ) // get it from https://arduinojson.org/ or install via Arduino library manager
//For ESP32, To use ESP32 Dev Module, QIO, Flash 4MB/80MHz, Upload 921600
//Ported to ESP32
#ifdef ESP32
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiMulti.h>          // From v1.1.0
WiFiMulti wifiMulti;
#if ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2) ) // LittleFS has higher priority than SPIFFS
#define USE_LITTLEFS    true
#define USE_SPIFFS      false
#elif defined(ARDUINO_ESP32C3_DEV)  // For core v1.0.6-, ESP32-C3 only supporting SPIFFS and EEPROM. To use v2.0.0+ for LittleFS
#define USE_LITTLEFS          false
#define USE_SPIFFS            true
#endif
#if USE_LITTLEFS        // Use LittleFS
#include "FS.h"
// Check cores/esp32/esp_arduino_version.h and cores/esp32/core_version.h
//#if ( ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(2, 0, 0) )  //(ESP_ARDUINO_VERSION_MAJOR >= 2)
#if ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2) )
#if (_WIFIMGR_LOGLEVEL_ > 3)
#warning Using ESP32 Core 1.0.6 or 2.0.0+
#endif
// The library has been merged into esp32 core from release 1.0.6
#include <LittleFS.h>       // https://github.com/espressif/arduino-esp32/tree/master/libraries/LittleFS
FS* filesystem =      &LittleFS;
#define FileFS        LittleFS
#define FS_Name       "LittleFS"
#else
#if (_WIFIMGR_LOGLEVEL_ > 3)
#warning Using ESP32 Core 1.0.5-. You must install LITTLEFS library
#endif
// The library has been merged into esp32 core from release 1.0.6
#include <LITTLEFS.h>       // https://github.com/lorol/LITTLEFS
FS* filesystem =      &LITTLEFS;
#define FileFS        LITTLEFS
#define FS_Name       "LittleFS"
#endif
#elif USE_SPIFFS
#include <SPIFFS.h>
FS* filesystem =      &SPIFFS;
#define FileFS        SPIFFS
#define FS_Name       "SPIFFS"
#else
#include <FFat.h>     // Use FFat
FS* filesystem =      &FFat;
#define FileFS        FFat
#define FS_Name       "FFat"
#endif
#define LED_BUILTIN       15  //2
#define LED_ON            HIGH
#define LED_OFF           LOW
#else
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFiMulti.h>     // From v1.1.0
ESP8266WiFiMulti wifiMulti;
#define USE_LITTLEFS      true
#if USE_LITTLEFS
#include <LittleFS.h>
FS* filesystem = &LittleFS;
#define FileFS    LittleFS
#define FS_Name       "LittleFS"
#else
FS* filesystem = &SPIFFS;
#define FileFS    SPIFFS
#define FS_Name       "SPIFFS"
#endif
#define ESP_getChipId()   (ESP.getChipId())
#define LED_ON      LOW
#define LED_OFF     HIGH
#endif
// These defines must be put before #include <ESP_DoubleResetDetector.h>
// to select where to store DoubleResetDetector's variable.
// For ESP32, You must select one to be true (EEPROM or SPIFFS)
// For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
// Otherwise, library will use default EEPROM storage
#ifdef ESP32
// These defines must be put before #include <ESP_DoubleResetDetector.h>
// to select where to store DoubleResetDetector's variable.
// For ESP32, You must select one to be true (EEPROM or SPIFFS)
// Otherwise, library will use default EEPROM storage
#if USE_LITTLEFS
#define ESP_DRD_USE_LITTLEFS    true
#define ESP_DRD_USE_SPIFFS      false
#define ESP_DRD_USE_EEPROM      false
#elif USE_SPIFFS
#define ESP_DRD_USE_LITTLEFS    false
#define ESP_DRD_USE_SPIFFS      true
#define ESP_DRD_USE_EEPROM      false
#else
#define ESP_DRD_USE_LITTLEFS    false
#define ESP_DRD_USE_SPIFFS      false
#define ESP_DRD_USE_EEPROM      true
#endif
#else //ESP8266
// For DRD
// These defines must be put before #include <ESP_DoubleResetDetector.h>
// to select where to store DoubleResetDetector's variable.
// For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
// Otherwise, library will use default EEPROM storage
#if USE_LITTLEFS
#define ESP_DRD_USE_LITTLEFS    true
#define ESP_DRD_USE_SPIFFS      false
#else
#define ESP_DRD_USE_LITTLEFS    false
#define ESP_DRD_USE_SPIFFS      true
#endif
#define ESP_DRD_USE_EEPROM      false
#define ESP8266_DRD_USE_RTC     false
#endif
#define DOUBLERESETDETECTOR_DEBUG       true  //false
#include <ESP_DoubleResetDetector.h>      //https://github.com/khoih-prog/ESP_DoubleResetDetector
#define DRD_TIMEOUT 3     // Number of seconds after reset during which a  // subseqent reset will be considered a double reset.
#define DRD_ADDRESS 0      // RTC Memory Address for the DoubleResetDetector to use
DoubleResetDetector* drd = NULL;
const char* CONFIG_FILE = "/ConfigMQTT.json";
// Default configuration values for MQTT
#define SERVER              "iotsmarthome.my.id"
#ifdef USE_TLS
#define SERVERPORT          "8883"   // Default Port TLS
#else
#define SERVERPORT          "1883"   // Default Port NON-TLS
#endif
#define USERNAME            "usertest"
#define KEY                 "usertest"
#define FINGERPRINT         "C8:58:1B:0B:83:DE:D4:C6:A2:39:D9:03:9F:9B:6F:B0:5C:45:55:9F"
// Labels for custom parameters in WiFi manager
#define SERVER_Label             "SERVER_Label"
#define SERVERPORT_Label         "SERVERPORT_Label"
#define USERNAME_Label           "USERNAME_Label"
#define KEY_Label                "KEY_Label"
#define FINGERPRINT_Label        "FINGERPRINT_Label"
// Just dummy topics. To be updated later when got valid data from FS or Config Portal
String MQTT_Pub_Topic       = "private/sensors";    
String MQTT_Status_Topic    = "private/status";
String MQTT_Sub_Topic       = "private/set";
// Variables to save custom parameters to...
// I would like to use these instead of #defines
#define custom_SERVER_LEN       20
#define custom_PORT_LEN          5
#define custom_USERNAME_LEN     20
#define custom_KEY_LEN          40
#define custom_FINGERPRINT_LEN  62
char custom_SERVER[custom_SERVER_LEN];
char custom_SERVERPORT[custom_PORT_LEN];
char custom_USERNAME[custom_USERNAME_LEN];
char custom_KEY[custom_KEY_LEN];
char custom_FINGERPRINT[custom_FINGERPRINT_LEN];
// Function Prototypes
void MQTT_connect();
bool readConfigFile();
bool writeConfigFile();
//#define FORMAT_FILESYSTEM       true     // From v1.1.1, You only need to format the filesystem once
#define FORMAT_FILESYSTEM         false
#define MIN_AP_PASSWORD_SIZE    8
#define SSID_MAX_LEN            32
#define PASS_MAX_LEN            64    //From v1.0.10, WPA2 passwords can be up to 63 characters long.
typedef struct {
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw  [PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct {
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;
#define NUM_WIFI_CREDENTIALS      2
// Assuming max 491 chars
#define TZNAME_MAX_LEN            50
#define TIMEZONE_MAX_LEN          50
typedef struct {
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
  char TZ_Name[TZNAME_MAX_LEN];     // "America/Toronto"
  char TZ[TIMEZONE_MAX_LEN];        // "EST5EDT,M3.2.0,M11.1.0"
  uint16_t checksum;
} WM_Config;
WM_Config         WM_config;
#define  CONFIG_FILENAME              F("/wifi_cred.dat")
bool initialConfig = false;       // Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESPAsync_WiFiManager.h>
#define USE_AVAILABLE_PAGES     false
// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
//#define USE_STATIC_IP_CONFIG_IN_CP          false
// Use false to disable NTP config. Advisable when using Cellphone, Tablet to access Config Portal.
// See Issue 23: On Android phone ConfigPortal is unresponsive (https://github.com/khoih-prog/ESP_WiFiManager/issues/23)
#define USE_ESP_WIFIMANAGER_NTP     false
// Just use enough to save memory. On ESP8266, can cause blank ConfigPortal screen    // if using too much memory
#define USING_AFRICA        false
#define USING_AMERICA       false
#define USING_ANTARCTICA    false
#define USING_ASIA          false
#define USING_ATLANTIC      false
#define USING_AUSTRALIA     false
#define USING_EUROPE        false
#define USING_INDIAN        false
#define USING_PACIFIC       false
#define USING_ETC_GMT       true
// Use true to enable CloudFlare NTP service. System can hang if you don't have Internet access while accessing CloudFlare
// See Issue #21: CloudFlare link in the default portal (https://github.com/khoih-prog/ESP_WiFiManager/issues/21)
#define USE_CLOUDFLARE_NTP          false
#define USING_CORS_FEATURE          true    // New in v1.0.11
// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
#if defined(USE_DHCP_IP)          // Force DHCP to be true
#undef USE_DHCP_IP
#endif
#define USE_DHCP_IP     true
#else
#define USE_DHCP_IP     true  // You can select DHCP or Static IP here
//#define USE_DHCP_IP     false
#endif
#if ( USE_DHCP_IP )           // Use DHCP
#if (_WIFIMGR_LOGLEVEL_ > 3)
#warning Using DHCP IP
#endif
IPAddress stationIP   = IPAddress(0, 0, 0, 0);
IPAddress gatewayIP   = IPAddress(192, 168, 1, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);
#else                         // Use static IP
#if (_WIFIMGR_LOGLEVEL_ > 3)
#warning Using static IP
#endif
#ifdef ESP32
IPAddress stationIP   = IPAddress(192, 168, 1, 232);
#else
IPAddress stationIP   = IPAddress(192, 168, 1, 186);
#endif
IPAddress gatewayIP   = IPAddress(192, 168, 1, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);
#endif
#define USE_CONFIGURABLE_DNS      true
IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(208, 67, 222, 222);
#define USE_CUSTOM_AP_IP          false
// New in v1.4.0
IPAddress APStaticIP  = IPAddress(192, 168, 100, 1);
IPAddress APStaticGW  = IPAddress(192, 168, 100, 1);
IPAddress APStaticSN  = IPAddress(255, 255, 255, 0);
// Must be placed before #include <ESP_WiFiManager.h>, or default port 80 will be used
//#define HTTP_PORT     8080
#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager
// Redundant, for v1.8.0 only
//#include <ESP_WiFiManager-Impl.h>         //https://github.com/khoih-prog/ESP_WiFiManager
// For Config Portal
String ssid = UID + String(ESP_getChipId(), HEX);      // SSID for Config Portal
String password;                                          // PW for Config Portal
String Router_SSID;   // SSID and PW for your Router
String Router_Pass;   // PW for your Router
int mqttfailcount = 0;
#define HTTP_PORT           80

// Create an ESP32 WiFiClient class to connect to the MQTT server
#ifdef USE_TLS
WiFiClientSecure *client     = NULL;  //TLS
#else
WiFiClient *client           = NULL;  //NON-TLS
#endif
PubSubClient   *mqtt         = NULL;
String ClientNID = UID + String(ESP_getChipId(), HEX);
const uint16_t kRecvPin = 14;
const uint16_t kIrLed = 27;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
bool irrecst = false;
const uint16_t kCaptureBufferSize = 1024;
#if DECODE_AC
// Some A/C units have gaps in their protocols of ~40ms. e.g. Kelvinator
// A value this large may swallow repeats of some protocols
const uint8_t kTimeout = 50;
#else   // DECODE_AC
// Suits most messages, while not swallowing many repeats.
const uint8_t kTimeout = 15;
#endif  // DECODE_AC
const uint16_t kMinUnknownSize = 12;
const uint8_t kTolerancePercentage = kTolerance;  // kTolerance is normally 25%
#define LEGACY_TIMING_INFO false
IRGreeAC ac(kIrLed);  // Set the GPIO to be used for sending messages.
IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type


///////////////////////////////////////////
// New in v1.4.0
/******************************************
   // Defined in ESPAsync_WiFiManager.h
  typedef struct
  {
  IPAddress _ap_static_ip;
  IPAddress _ap_static_gw;
  IPAddress _ap_static_sn;

  }  WiFi_AP_IPConfig;

  typedef struct
  {
  IPAddress _sta_static_ip;
  IPAddress _sta_static_gw;
  IPAddress _sta_static_sn;
  #if USE_CONFIGURABLE_DNS
  IPAddress _sta_static_dns1;
  IPAddress _sta_static_dns2;
  #endif
  }  WiFi_STA_IPConfig;
******************************************/

WiFi_AP_IPConfig  WM_AP_IPconfig;
WiFi_STA_IPConfig WM_STA_IPconfig;

void initAPIPConfigStruct(WiFi_AP_IPConfig &in_WM_AP_IPconfig) {
  in_WM_AP_IPconfig._ap_static_ip   = APStaticIP;
  in_WM_AP_IPconfig._ap_static_gw   = APStaticGW;
  in_WM_AP_IPconfig._ap_static_sn   = APStaticSN;
}

void initSTAIPConfigStruct(WiFi_STA_IPConfig &in_WM_STA_IPconfig) {
  in_WM_STA_IPconfig._sta_static_ip   = stationIP;
  in_WM_STA_IPconfig._sta_static_gw   = gatewayIP;
  in_WM_STA_IPconfig._sta_static_sn   = netMask;
#if USE_CONFIGURABLE_DNS
  in_WM_STA_IPconfig._sta_static_dns1 = dns1IP;
  in_WM_STA_IPconfig._sta_static_dns2 = dns2IP;
#endif
}

void displayIPConfigStruct(WiFi_STA_IPConfig in_WM_STA_IPconfig) {
  LOGERROR3(F("stationIP ="), in_WM_STA_IPconfig._sta_static_ip, ", gatewayIP =", in_WM_STA_IPconfig._sta_static_gw);
  LOGERROR1(F("netMask ="), in_WM_STA_IPconfig._sta_static_sn);
#if USE_CONFIGURABLE_DNS
  LOGERROR3(F("dns1IP ="), in_WM_STA_IPconfig._sta_static_dns1, ", dns2IP =", in_WM_STA_IPconfig._sta_static_dns2);
#endif
}

void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig) {
#if USE_CONFIGURABLE_DNS
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn, in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);
#else
  // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
  WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
#endif
}

uint8_t connectMultiWiFi() {
#if ESP32
  // For ESP32, this better be 0 to shorten the connect time.
  // For ESP32-S2/C3, must be > 500
#if ( USING_ESP32_S2 || USING_ESP32_C3 )
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS           500L
#else
  // For ESP32 core v1.0.6, must be >= 500
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS           800L
#endif
#else
  // For ESP8266, this better be 2200 to enable connect the 1st time
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS             2200L
#endif
#define WIFI_MULTI_CONNECT_WAITING_MS                   500L
  uint8_t status;
  //WiFi.mode(WIFI_STA);
  LOGERROR(F("ConnectMultiWiFi with :"));
  if ( (Router_SSID != "") && (Router_Pass != "") ) {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass );
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }
  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++) {    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) ) {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }
  LOGERROR(F("Connecting MultiWifi..."));
  //WiFi.mode(WIFI_STA);
#if !USE_DHCP_IP
  configWiFi(WM_STA_IPconfig);    // New in v1.4.0
#endif
  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);
  while ( ( i++ < 20 ) && ( status != WL_CONNECTED ) ) {
    status = WiFi.status();
    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }
  if ( status == WL_CONNECTED ) {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else {
    LOGERROR(F("WiFi not connected"));
    drd->loop();    // To avoid unnecessary DRD
#if ESP8266
    ESP.reset();
#else
    ESP.restart();
#endif
  }
  return status;
}

void toggleLED() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  //toggle state
}

#if USE_ESP_WIFIMANAGER_NTP
void printLocalTime() {
#if ESP8266
  static time_t now;
  now = time(nullptr);
  if ( now > 1451602800 ) {
    Serial.print("Local Date/Time: ");
    Serial.print(ctime(&now));
  }
#else
  struct tm timeinfo;
  getLocalTime( &timeinfo );
  // Valid only if year > 2000.
  // You can get from timeinfo : tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec
  if (timeinfo.tm_year > 100 ) {
    Serial.print("Local Date/Time: ");
    Serial.print( asctime( &timeinfo ) );
  }
#endif
}
#endif

void heartBeatPrint() {
#if USE_ESP_WIFIMANAGER_NTP
  printLocalTime();
#else
  static int num = 1;
  if (WiFi.status() == WL_CONNECTED)
    Serial.print(F("W"));        // W means connected to WiFi
  else
    Serial.print(F("N"));        // N means not connected to WiFi
  if (num == 40) {
    Serial.println();
    num = 1;
  }
  else if (num++ % 5 == 0) {
    Serial.print(F(" "));
  }
#endif
}

void publishMQTT() {
//  float temperature, humidity;
//  AHTSensor(&temperature, &humidity);
  // For debug only
  //Serial.print(F("Published Temp = "));
  //Serial.println(some_number);
  MQTT_connect();
  if (mqtt->publish(String(MQTT_Pub_Topic + "/Sensors/Temperature").c_str(),String(aht10.readTemperature()).c_str())) {
    Serial.print(F("Tt"));        // T means publishing OK
    mqttfailcount = 0 ;
  }
  else {
    Serial.print(F("F"));        // F means publishing failure
    mqttfailcount = mqttfailcount+1 ;
  }
  if (mqtt->publish(String(MQTT_Pub_Topic + "/Sensors/Humidity").c_str(),String(aht10.readHumidity()).c_str())) {
    Serial.print(F("Th"));        // Th means publishing OK
    mqttfailcount = 0 ;
  }
  else {
    Serial.print(F("F"));        // F means publishing failure
    mqttfailcount = mqttfailcount+1 ;
  }
  if (mqttfailcount >= 15) 
#if ESP8266
    ESP.reset();
#else
    ESP.restart();
#endif
  Serial.print(F("  "));
  Serial.print(mqttfailcount);
  Serial.print(F("  "));
}

void check_WiFi() {
  if ( (WiFi.status() != WL_CONNECTED) ) {
    Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
    connectMultiWiFi();
  }
}

void check_status() {
  static ulong checkstatus_timeout  = 0;
  static ulong LEDstatus_timeout    = 0;
  static ulong checkwifi_timeout    = 0;
  static ulong mqtt_publish_timeout = 0;
  ulong current_millis = millis();
#define LED_INTERVAL          2000L
#define PUBLISH_INTERVAL      20000L
#define WIFICHECK_INTERVAL    1000L
#if USE_ESP_WIFIMANAGER_NTP
#define HEARTBEAT_INTERVAL    60000L
#else
#define HEARTBEAT_INTERVAL    10000L
#endif
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0)) {   // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }
  if ((current_millis > LEDstatus_timeout) || (LEDstatus_timeout == 0)) {    // Toggle LED at LED_INTERVAL = 2s
    toggleLED();
    LEDstatus_timeout = current_millis + LED_INTERVAL;
  }
  if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0)) {  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
    heartBeatPrint();
    checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
  }
  if ((current_millis > mqtt_publish_timeout) || (mqtt_publish_timeout == 0)) {  // Check every PUBLISH_INTERVAL (60) seconds.
    if (WiFi.status() == WL_CONNECTED) {
      publishMQTT();
      PublishACState();
    }
    mqtt_publish_timeout = current_millis + PUBLISH_INTERVAL;
  }
}
int calcChecksum(uint8_t* address, uint16_t sizeToCalc) {
  uint16_t checkSum = 0;
  for (uint16_t index = 0; index < sizeToCalc; index++) {
    checkSum += * ( ( (byte*) address ) + index);
  }
  return checkSum;
}

bool loadConfigData() {
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));
  memset((void *) &WM_config,       0, sizeof(WM_config));
  memset((void *) &WM_STA_IPconfig, 0, sizeof(WM_STA_IPconfig));  // New in v1.4.0
  if (file) {
    file.readBytes((char *) &WM_config,   sizeof(WM_config));
    file.readBytes((char *) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));    // New in v1.4.0
    file.close();
    LOGERROR(F("OK"));
    if ( WM_config.checksum != calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) ) ) {
      LOGERROR(F("WM_config checksum wrong"));
      return false;
    }
    displayIPConfigStruct(WM_STA_IPconfig);    // New in v1.4.0
    return true;
  }
  else {
    LOGERROR(F("failed"));
    return false;
  }
}

void saveConfigData() {
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));
  if (file) {
    WM_config.checksum = calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) );
    file.write((uint8_t*) &WM_config, sizeof(WM_config));
    displayIPConfigStruct(WM_STA_IPconfig);
    file.write((uint8_t*) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));    // New in v1.4.0
    file.close();
    LOGERROR(F("OK"));
  }
  else {
    LOGERROR(F("failed"));
  }
}

void deleteOldInstances() {  // Delete previous instances
  if (mqtt) {
    delete mqtt;
    mqtt = NULL;
    Serial.println(F("Deleting old MQTT object"));
  }
}

void createNewInstances() {
  if (!client) {
#ifdef USE_TLS
    client = new WiFiClientSecure(); //TLS
#ifdef ESP32
    client->setInsecure();
#else
    client->setFingerprint(custom_FINGERPRINT); //TLS Required
#endif
#else
    client = new WiFiClient;  //NON-TLS
#endif
    Serial.print(F("\nCreating new WiFi client object : "));
    Serial.println(client ? F("OK") : F("failed"));
  }
  if (!mqtt) {  // Create new instances from new data
    //Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
    //mqtt = new Adafruit_MQTT_Client(client, custom_AIO_SERVER, atoi(custom_AIO_SERVERPORT), custom_AIO_USERNAME, custom_AIO_KEY);
    mqtt = new PubSubClient(custom_SERVER,atoi(custom_SERVERPORT),*client);
    mqtt->setCallback(callback);
    mqtt->connect(ClientNID.c_str(),custom_USERNAME, custom_KEY);
    MQTT_Sub_Topic = String(custom_USERNAME) + "/" + ClientNID;
//    MQTT_Sub_Topic = "nodemcuv3";
    mqtt->subscribe((MQTT_Sub_Topic + "/set/#").c_str());
//    EspMQTTClient = new client(custom_SERVER, atoi(custom_SERVERPORT), custom_USERNAME, custom_KEY, ClientNID);

    Serial.print(F("Creating new MQTT object : "));
    if (mqtt) {
      Serial.println(F("OK"));
      Serial.println(String("SERVER = ")    + custom_SERVER    + ", SERVERPORT = "  + custom_SERVERPORT);
      Serial.println(String("USERNAME = ")  + custom_USERNAME  + ", KEY = "         + custom_KEY);
      Serial.println(String("FINGERPRINT = ")  + custom_FINGERPRINT);
    }
    else
      Serial.println(F("Failed"));
  }
#ifdef USE_TLS
  Serial.println(F("TLS"));
#else
  Serial.println(F("NON-TLS"));
#endif
}

void wifi_manager() {
  Serial.println(F("\nConfig Portal requested."));
  digitalWrite(LED_BUILTIN, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.
  ESP_WiFiManager ESP_wifiManager("ConfigOnDRD_FS-MQTT");;  //Local intialization. Once its business is done, there is no need to keep it around
  //Check if there is stored WiFi router/password credentials.
  //If not found, device will remain in configuration mode until switched off via webserver.
  Serial.print(F("Opening Configuration Portal. "));
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();
  if ( !initialConfig && (Router_SSID != "") && (Router_Pass != "") ) {  // From v1.1.1, Don't permit NULL password
    ESP_wifiManager.setConfigPortalTimeout(120);    //If valid AP credential and not DRD, set timeout 120s.
    Serial.println(F("Got stored Credentials. Timeout 120s"));
  }
  else {
    ESP_wifiManager.setConfigPortalTimeout(0);
    Serial.print(F("No timeout : "));
    if (initialConfig) {
      Serial.println(F("DRD or No stored Credentials.."));
    }
    else {
      Serial.println(F("No stored Credentials."));
    }
  }
  //Local intialization. Once its business is done, there is no need to keep it around
  // Extra parameters to be configured
  // After connecting, parameter.getValue() will get you the configured value
  // Format: <ID> <Placeholder text> <default value> <length> <custom HTML> <label placement>
  // (*** we are not using <custom HTML> and <label placement> ***)
  ESP_WMParameter SERVER_FIELD(SERVER_Label, "SERVER", custom_SERVER, custom_SERVER_LEN + 1);  // SERVER
  ESP_WMParameter SERVERPORT_FIELD(SERVERPORT_Label, "SERVER PORT", custom_SERVERPORT, custom_PORT_LEN + 1);  // SERVERPORT
  ESP_WMParameter USERNAME_FIELD(USERNAME_Label, "USERNAME", custom_USERNAME, custom_USERNAME_LEN + 1);  // USERNAME
  ESP_WMParameter KEY_FIELD(KEY_Label, "KEY", custom_KEY, custom_KEY_LEN + 1);  // PASSWORD
  ESP_WMParameter FINGERPRINT_FIELD(FINGERPRINT_Label, "FINGERPRINT", custom_FINGERPRINT, custom_FINGERPRINT_LEN + 1);  // FINGERPRINT

  // add all parameters here
  ESP_wifiManager.addParameter(&SERVER_FIELD);
  ESP_wifiManager.addParameter(&SERVERPORT_FIELD);
  ESP_wifiManager.addParameter(&USERNAME_FIELD);
  ESP_wifiManager.addParameter(&KEY_FIELD);
  ESP_wifiManager.addParameter(&FINGERPRINT_FIELD);
  // order of adding is not important

  // Sets timeout in seconds until configuration portal gets turned off.
  // If not specified device will remain in configuration mode until
  // switched off via webserver or device is restarted.
  //ESP_wifiManager.setConfigPortalTimeout(600);
  ESP_wifiManager.setMinimumSignalQuality(-1);
  ESP_wifiManager.setConfigPortalChannel(0);    // From v1.0.10 only // Set config portal channel, default = 1. Use 0 => random channel from 1-13
#if USE_CUSTOM_AP_IP
  // New in v1.4.0
  ESP_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);  //set custom ip for portal
  //////
#endif
#if !USE_DHCP_IP
  // New in v1.4.0
  ESP_wifiManager.setSTAStaticIPConfig(WM_STA_IPconfig);  // Set (static IP, Gateway, Subnetmask, DNS1 and DNS2) or (IP, Gateway, Subnetmask). New in v1.0.5
#endif
#if USING_CORS_FEATURE
  ESP_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");  // New from v1.1.1
#endif
  // Start an access point  // and goes into a blocking loop awaiting configuration.
  // Once the user leaves the portal with the exit button  // processing will continue
  ssid.toUpperCase();  // SSID to uppercase
  password = ssid;
  Serial.print(F("Starting configuration portal @ "));
#if USE_CUSTOM_AP_IP
  Serial.print(APStaticIP);
#else
  Serial.print(F("192.168.4.1"));
#endif
#if defined(HTTP_PORT_TO_USE)
  Serial.print(F(":")); Serial.print(HTTP_PORT_TO_USE);
#endif
  Serial.print(F(", SSID = "));
  Serial.print(ssid);
  Serial.print(F(", PWD = "));
  Serial.println(password);
#if DISPLAY_STORED_CREDENTIALS_IN_CP
  // New. Update Credentials, got from loadConfigData(), to display on CP
  ESP_wifiManager.setCredentials(WM_config.WiFi_Creds[0].wifi_ssid, WM_config.WiFi_Creds[0].wifi_pw,
                                 WM_config.WiFi_Creds[1].wifi_ssid, WM_config.WiFi_Creds[1].wifi_pw);
#endif
  if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password.c_str())) {
    Serial.println(F("Not connected to WiFi but continuing anyway."));
  }
  else {
    // If you get here you have connected to the WiFi
    Serial.println(F("Connected...yeey :)"));
    Serial.print(F("Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("Gateway IP: "));
    Serial.println(WiFi.gatewayIP());
  }
  // Only clear then save data if CP entered and with new valid Credentials
  // No CP => stored getSSID() = ""
  if ( String(ESP_wifiManager.getSSID(0)) != "" && String(ESP_wifiManager.getSSID(1)) != "" ) {
    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));
    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++) {
      String tempSSID = ESP_wifiManager.getSSID(i);
      String tempPW   = ESP_wifiManager.getPW(i);
      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);
      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) ) {      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }
#if USE_ESP_WIFIMANAGER_NTP
    String tempTZ   = ESP_wifiManager.getTimezoneName();
    if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
      strcpy(WM_config.TZ_Name, tempTZ.c_str());
    else
      strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);
    const char * TZ_Result = ESP_wifiManager.getTZ(WM_config.TZ_Name);
    if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
      strcpy(WM_config.TZ, TZ_Result);
    else
      strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);
    if ( strlen(WM_config.TZ_Name) > 0 ) {
      LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);
#if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org");
#else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
    }
    else {
      LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
    }
#endif
    ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);    // New in v1.4.0
    saveConfigData();
  }
  // Getting posted form values and overriding local variables parameters
  // Config file is written regardless the connection state
  strcpy(custom_SERVER, SERVER_FIELD.getValue());
  strcpy(custom_SERVERPORT, SERVERPORT_FIELD.getValue());
  strcpy(custom_USERNAME, USERNAME_FIELD.getValue());
  strcpy(custom_KEY, KEY_FIELD.getValue());
  strcpy(custom_FINGERPRINT, FINGERPRINT_FIELD.getValue());
  writeConfigFile();  // Writing JSON config file to flash for next boot
  digitalWrite(LED_BUILTIN, LED_OFF); // Turn LED off as we are not in configuration mode.
  deleteOldInstances();
  MQTT_Pub_Topic = String(custom_USERNAME) + "/" + ClientNID ;
  MQTT_Status_Topic = String(custom_USERNAME) + "/" + ClientNID + "/state";
  createNewInstances();
}

bool readConfigFile() {
  File f = FileFS.open(CONFIG_FILE, "r");  // this opens the config file in read-mode
  if (!f) {
    Serial.println(F("Config File not found"));
    return false;
  }
  else {
    size_t size = f.size();    // we could open the file
    std::unique_ptr<char[]> buf(new char[size + 1]);    // Allocate a buffer to store contents of the file.
    f.readBytes(buf.get(), size);    // Read and store file contents in buf
    f.close();    // Closing file
    // Using dynamic JSON buffer which is not the recommended memory model, but anyway // See https://github.com/bblanchon/ArduinoJson/wiki/Memory%20model
#if (ARDUINOJSON_VERSION_MAJOR >= 6)
    DynamicJsonDocument json(1024);
    auto deserializeError = deserializeJson(json, buf.get());
    if ( deserializeError ) {
      Serial.println(F("JSON parseObject() failed"));
      return false;
    }
    serializeJson(json, Serial);
#else
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.parseObject(buf.get());    // Parse JSON string
    if (!json.success()) {    // Test if parsing succeeds.
      Serial.println(F("JSON parseObject() failed"));
      return false;
    }
    json.printTo(Serial);
#endif
    // Parse all config file parameters, override
    // local config variables with parsed values
    if (json.containsKey(SERVER_Label)) {
      strcpy(custom_SERVER, json[SERVER_Label]);
    }
    if (json.containsKey(SERVERPORT_Label)) {
      strcpy(custom_SERVERPORT, json[SERVERPORT_Label]);
    }
    if (json.containsKey(USERNAME_Label)) {
      strcpy(custom_USERNAME, json[USERNAME_Label]);
    }
    if (json.containsKey(KEY_Label)) {
      strcpy(custom_KEY, json[KEY_Label]);
    }
    if (json.containsKey(FINGERPRINT_Label)) {
      strcpy(custom_FINGERPRINT, json[FINGERPRINT_Label]);
    }
  }
  Serial.println(F("\nConfig File successfully parsed"));
  return true;
}

bool writeConfigFile() {
  Serial.println(F("Saving Config File"));
#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  DynamicJsonDocument json(1024);
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
#endif
  // JSONify local configuration parameters
  json[SERVER_Label]      = custom_SERVER;
  json[SERVERPORT_Label]  = custom_SERVERPORT;
  json[USERNAME_Label]    = custom_USERNAME;
  json[KEY_Label]         = custom_KEY;
  json[FINGERPRINT_Label] = custom_FINGERPRINT;

  File f = FileFS.open(CONFIG_FILE, "w");  // Open file for writing
  if (!f) {
    Serial.println(F("Failed to open Config File for writing"));
    return false;
  }
#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  serializeJsonPretty(json, Serial);
  serializeJson(json, f);  // Write data to file and close it
#else
  json.prettyPrintTo(Serial);
  json.printTo(f);  // Write data to file and close it
#endif
  f.close();
  Serial.println(F("\nConfig File successfully saved"));
  return true;
}

// this function is just to display newly saved data,
// it is not necessary though, because data is displayed
// after WiFi manager resets ESP32
void newConfigData() {
  Serial.println();
  Serial.print(F("custom_SERVER: "));
  Serial.println(custom_SERVER);
  Serial.print(F("custom_SERVERPORT: "));
  Serial.println(custom_SERVERPORT);
  Serial.print(F("custom_USERNAME: "));
  Serial.println(custom_USERNAME);
  Serial.print(F("custom_KEY: "));
  Serial.println(custom_KEY);
  Serial.print(F("custom_FINGERPRINT: "));
  Serial.println(custom_FINGERPRINT);
  Serial.println();
}

void MQTT_connect() {
  int8_t ret;
  MQTT_Pub_Topic = String(custom_USERNAME) + "/" + ClientNID;
  MQTT_Status_Topic = String(custom_USERNAME) + "/" + ClientNID + "/state";
  createNewInstances();
  // Return if already connected
  if (mqtt->connected()) {
    return;
  }
  Serial.println(F("Connecting to MQTT (3 attempts)..."));
  uint8_t attempt = 3;
  while ((ret = mqtt->connected()) != 0) {    // connect will return 0 for connected
//    Serial.println(mqtt->connectErrorString(ret));
    Serial.println(mqtt->state());
    Serial.println(F("Another attemtpt to connect to MQTT in 2 seconds..."));
    mqtt->disconnect();
    delay(2000);  // wait 2 seconds
    attempt--;
    if (attempt == 0) {
      Serial.println(F("MQTT connection failed. Continuing with program..."));
      return;
    }
  }
  Serial.println(F("MQTT connection successful!"));
}

void printState() {
  Serial.println("GREE A/C remote is in the following state:");  // Display the settings.
  Serial.printf("  %s\n", ac.toString().c_str());  // Display the encoded IR sequence.
  unsigned char* ir_code = ac.getRaw();
  Serial.print("IR Code: 0x");
  for (uint8_t i = 0; i < kGreeStateLength; i++)
    Serial.printf("%02X", ir_code[i]);
  Serial.println();
  Serial.println();
}

void setup() {
  // Put your setup code here, to run once
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED digital pin as an output.
  Serial.begin(115200);
  while (!Serial);
  delay(200);
  Serial.print(F("\nStarting ConfigOnDRD_FS_MQTT_Ptr using ")); Serial.print(FS_Name);
  Serial.print(F(" on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_WIFIMANAGER_VERSION);
  Serial.println(ESP_DOUBLE_RESET_DETECTOR_VERSION);
#if defined(ESP_WIFIMANAGER_VERSION_MIN)
  if (ESP_WIFIMANAGER_VERSION_INT < ESP_WIFIMANAGER_VERSION_MIN) {
    Serial.print("Warning. Must use this example on Version later than : ");
    Serial.println(ESP_WIFIMANAGER_VERSION_MIN_TARGET);
  }
#endif
  Serial.setDebugOutput(false);
  if (FORMAT_FILESYSTEM) {  // Mount the filesystem
    Serial.println(F("Forced Formatting."));
    FileFS.format();
  }
#ifdef ESP32
  if (!FileFS.begin(true))
#else
  if (!FileFS.begin())
#endif
  {
#ifdef ESP8266
    FileFS.format();  // Format FileFS if not yet
#endif
    Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));
    if (!FileFS.begin()) {
      delay(100);      // prevents debug info from the library to hide err message.
#if USE_LITTLEFS
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
      Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif
      while (true) {
        delay(1);
      }
    }
  }

  // New in v1.4.0
  initAPIPConfigStruct(WM_AP_IPconfig);
  initSTAIPConfigStruct(WM_STA_IPconfig);
  //////

  if (!readConfigFile()) {
    Serial.println(F("Can't read Config File, using default values"));
  }
  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (!drd) {
    Serial.println(F("Can't instantiate. Disable DRD feature"));
  }
  else if (drd->detectDoubleReset()) {
//    ESP_wifiManager.setConfigPortalTimeout(0);   // DRD, disable timeout.
    Serial.println(F("Open Config Portal without Timeout: Double Reset Detected"));
    initialConfig = true;
  }
  if (initialConfig) {
    loadConfigData();
    wifi_manager();
  }
  else {
    initialConfig = true;    // Pretend CP is necessary as we have no AP Credentials
    if (loadConfigData()) {    // Load stored data, the addAP ready for MultiWiFi reconnection
#if USE_ESP_WIFIMANAGER_NTP
      if ( strlen(WM_config.TZ_Name) > 0 ) {
        LOGERROR3(F("Current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);
#if ESP8266
        configTime(WM_config.TZ, "pool.ntp.org");
#else
        //configTzTime(WM_config.TZ, "pool.ntp.org" );
        configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
      }
      else {
        Serial.println(F("Current Timezone is not set. Enter Config Portal to set."));
      }
#endif

      for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++) {
        // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
        if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) ) {
          LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
          wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
          initialConfig = false;
        }
      }
    }
    if (initialConfig) {
      Serial.println(F("Open Config Portal without Timeout: No stored WiFi Credentials"));
      wifi_manager();
    }
    else if ( WiFi.status() != WL_CONNECTED ) {
      Serial.println(F("ConnectMultiWiFi in setup"));
      connectMultiWiFi();
    }
  }
  digitalWrite(LED_BUILTIN, LED_OFF); // Turn led off as we are not in configuration mode.
//=================================================================================================================================
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3"); // "admin"
  ArduinoOTA.setPasswordHash("63243dca61b32d6443e99bcc939b222f");  // "passwordota"
  // ArduinoOTA.setPasswordHash("ac43724f16e9241d990427ab7c8f4228");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Wire.begin();
  aht10.begin();
  irsend.begin();
  assert(irutils::lowLevelSanityCheck() == 0);
#if DECODE_HASH
  // Ignore messages with less than minimum on or off pulses.
  irrecv.setUnknownThreshold(kMinUnknownSize);
#endif  // DECODE_HASH
  irrecv.setTolerance(kTolerancePercentage);  // Override the default tolerance.
  irrecv.enableIRIn();  // Start the receiver  
  Serial.println("Default state of the remote.");
  Serial.println("Setting desired state for A/C.");
  ac.begin();
  ac.setUseFahrenheit(false); //ac.setUseFahrenheit(false), ac.setUseFahrenheit(true)
  ac.setDisplayTempSource(0);
  printState();
//=================================================================================================================================
}

// Loop function
void loop() {
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  if (drd) drd->loop(); 
  if(mqtt) mqtt->loop();
  ArduinoOTA.handle();
  check_status();  // this is just for checking if we are connected to WiFi  
  if (irrecst && irrecv.decode(&results)) {
    Serial.println();
    Serial.print(resultToHumanReadableBasic(&results));
    //    String description = IRAcUtils::resultAcToString(&results);
    //    if (description.length()) Serial.println(D_STR_MESGDESC ": " + description);
    //    yield();  // Feed the WDT as the text output can take a while to print.
    Serial.print(F("\nSending HEX code value : "));
    Serial.print(results.value, HEX);
    Serial.print("...");
//    if (! mqtt->publish((String(MQTT_Pub_Topic) + "/ir-receiver/code").c_str(), String(results.value, HEX).c_str())) {
//      Serial.println(F("Failed Sending Code"));
    if (! mqtt->publish((String(MQTT_Pub_Topic) + "/state/ir-receiver/code").c_str(), String(resultToHumanReadableBasic(&results)).c_str())) {
      Serial.println(F("Failed Sending Code"));
    } else {
      Serial.println(F("Code Sent!"));
    }
    Serial.print("Decimal Code :"); Serial.println(results.value, DEC);
    Serial.println();
    irrecv.resume();  // Receive the next value
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Message: " + message);
  if (String(topic) == (MQTT_Sub_Topic + "/set/ac/light").c_str()) ac.setLight(message.toInt());
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/turbo") ac.setTurbo(message.toInt());
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/xfan") ac.setXFan(message.toInt());
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/sleep") ac.setSleep(message.toInt());
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/temperature") ac.setTemp(message.toInt());
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/power") ac.setPower(message.toInt());
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "off") ac.off();
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message != "off") {
    ac.on();
    if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "auto") ac.setMode(0);  //ac.setMode(kGreeAuto);
    else if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "cool") ac.setMode(1);  //ac.setMode(kGreeCool); 
    else if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "dry") ac.setMode(2);  //ac.setMode(kGreeDry);
    else if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "fan_only") ac.setMode(3);  //ac.setMode(kGreeFan);
  //  else if (String(topic) == "nodemcuv3//set/ac/mode" && message == "heat") ac.setMode(4);  //ac.setMode(kGreeHeat); 
  }
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Auto") ac.setFan(0);  //ac.setFan(kGreeFanAuto);
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Minimum") ac.setFan(1);  //ac.setFan(kGreeFanMin);
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Medium") ac.setFan(2);  //ac.setFan(kGreeFanMed);
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Maximum") ac.setFan(3);  //ac.setFan(kGreeFanMax);
  if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Last Pos") ac.setSwingVertical(0,0);  //ac.setSwingVertical(false,kGreeSwingLastPos);  //0b0000
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Auto") ac.setSwingVertical(1,1);  //ac.setSwingVertical(true,kGreeSwingAuto);  //0b0001
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Up") ac.setSwingVertical(0,2);  //ac.setSwingVertical(false,kGreeSwingUp);  //0b0010
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle Up") ac.setSwingVertical(0,3);  //ac.setSwingVertical(false,kGreeSwingMiddleUp);  //0b0011
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle") ac.setSwingVertical(0,4);  //ac.setSwingVertical(false,kGreeSwingMiddle);  //0b0100
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle Down") ac.setSwingVertical(0,5);  //ac.setSwingVertical(false,kGreeSwingMiddleDown);  //0b0101
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Down") ac.setSwingVertical(0,6);  //ac.setSwingVertical(false,kGreeSwingDown);  //0b0110
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Down Auto") ac.setSwingVertical(1,7);  //ac.setSwingVertical(true,kGreeSwingDownAuto);  //0b0111
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle Auto") ac.setSwingVertical(1,9);  //ac.setSwingVertical(true,kGreeSwingMiddleAuto);  //0b1001
  else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Up Auto") ac.setSwingVertical(1,11);  //ac.setSwingVertical(true,kGreeSwingUpAuto);  //0b1011
  if (strstr(topic, (MQTT_Sub_Topic + "/set/ac").c_str()) != NULL) {
    printState();
    PublishACState();
    ac.send();
  }

//  if (strstr(topic, (MQTT_Sub_Topic + "/universal").c_str()) != NULL) irsend.send(PANASONIC, 0x40040100BCBD, 48, 0);
//  if (String(topic) == (MQTT_Sub_Topic + "/universal/tv-panasonic").c_str()) irsend.send(PANASONIC, (strtoull(message.c_str(), NULL, 16)), 48, 0);  //HEX Code manually by MQTT msg, ex : 0x40040100BCBD
  if (strstr(topic, "/tv-panasonic") != NULL) irsend.send(PANASONIC, (strtoull(message.c_str(), NULL, 16)), 48, 0);  //HEX Code manually by MQTT msg, ex : 0x40040100BCBD
  if (String(topic) == (MQTT_Sub_Topic + "/set/ir-receiver/state") && message == "1") irrecst = true;
  else if (String(topic) == (MQTT_Sub_Topic + "/set/ir-receiver/state") && message == "0") irrecst = false;
}

void PublishACState() {
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/light").c_str(),String(ac.getLight()).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/turbo").c_str(),String(ac.getTurbo()).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/xfan").c_str(),String(ac.getXFan()).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/sleep").c_str(),String(ac.getSleep()).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/temperature").c_str(),String(ac.getTemp()).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/power").c_str(),String(ac.getPower()).c_str());
  if (ac.getPower() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "off");
  else if (ac.getMode() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"auto");
  else if (ac.getMode() == 1) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"cool");
  else if (ac.getMode() == 2) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"dry");
  else if (ac.getMode() == 3) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"fan_only");
//  else if (ac.getMode() == 4) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"heat");
  if (ac.getFan() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Auto");
  else if (ac.getFan() == 1) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Minimum");
  else if (ac.getFan() == 2) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Medium");
  else if (ac.getFan() == 3) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Maximum");
  if (ac.getSwingVerticalPosition() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Last Pos");
  else if (ac.getSwingVerticalPosition() == 1) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Auto");
  else if (ac.getSwingVerticalPosition() == 2) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Up");
  else if (ac.getSwingVerticalPosition() == 3) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle Up");
  else if (ac.getSwingVerticalPosition() == 4) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle");
  else if (ac.getSwingVerticalPosition() == 5) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle Down");
  else if (ac.getSwingVerticalPosition() == 6) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Down");
  else if (ac.getSwingVerticalPosition() == 7) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Down Auto");
  else if (ac.getSwingVerticalPosition() == 9) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle Auto");
  else if (ac.getSwingVerticalPosition() == 11) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Up Auto");
  Serial.println(F("AC State Updated"));
}
