#if !( defined(ESP8266) ||  defined(ESP32) )
#error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif
#define USE_SSL          //Uncomment jika koneksi menggunakan SSL/TLS
#ifdef ESP32
#define UID              "ESP32DevKit-v1-"    //sesuaikan unique id untuk perangkat
#else
#define UID              "NodeMCU-v3-"    //sesuaikan unique id untuk perangkat
#endif
#define DECODE_AC
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
#ifdef ESP32
#include <ArduinoOTA.h>
#endif
#include <PubSubClient.h>
#include <Wire.h>
#include <AHTxx.h>
// #include <OneButton.h>          // for button
#include <FS.h>
#include <ArduinoJson.h>        // Now support ArduinoJson 6.0.0+ ( tested with v6.15.2 to v6.16.1 ) // get it from https://arduinojson.org/ or install via Arduino library manager
#ifdef USE_SSL
  #include <WiFiClientSecure.h>
#endif
//For ESP32, To use ESP32 Dev Module, QIO, Flash 4MB/80MHz, Upload 921600
//Ported to ESP32
#ifdef ESP32
#include <esp_wifi.h>
#include <WiFi.h>
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
#define SERVER_LOCAL        "192.168.1.93"
#define SERVERPORT          "1883"   // Default Port NON-TLS
#define SERVERPRIO          "1"
#define USERNAME            "usertest"
#define KEY                 "usertest"
#define FINGERPRINT         "C8:58:1B:0B:83:DE:D4:C6:A2:39:D9:03:9F:9B:6F:B0:5C:45:55:9F"
// Labels for custom parameters in WiFi manager
#define SERVER_Label             "SERVER_Label"
#define SERVER_LOCAL_Label       "SERVER_LOCAL_Label"
#define SERVERPORT_Label         "SERVERPORT_Label"
#define SERVERPRIO_Label         "SERVERPRIO_Label"
#define USERNAME_Label           "USERNAME_Label"
#define KEY_Label                "KEY_Label"
#define FINGERPRINT_Label        "FINGERPRINT_Label"
// Just dummy topics. To be updated later when got valid data from FS or Config Portal
String MQTT_Pub_Topic       = "private/sensors";    
String MQTT_Status_Topic    = "private/status";
String MQTT_Sub_Topic       = "private/set";
// Variables to save custom parameters to...
// I would like to use these instead of #defines
#define custom_SERVER_LEN             30
#define custom_SERVER_LOCAL_LEN       20
#define custom_PORT_LEN               5
#define custom_USERNAME_LEN           20
#define custom_KEY_LEN                40
#ifdef USE_SSL
#ifdef ESP32
  #define custom_FINGERPRINT_LEN        2200
#else
  #define custom_FINGERPRINT_LEN        64
#endif
#endif
char custom_SERVER[custom_SERVER_LEN];
char custom_SERVER_LOCAL[custom_SERVER_LOCAL_LEN];
char custom_SERVERPORT[custom_PORT_LEN];
bool custom_SERVERPRIO;              //false: prioritas server lokal, true: prioritas server inet
char custom_USERNAME[custom_USERNAME_LEN];
char custom_KEY[custom_KEY_LEN];
#ifdef USE_SSL
char custom_FINGERPRINT[custom_FINGERPRINT_LEN];
#endif
bool servernet = false;
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
IPAddress gatewayIP   = IPAddress(0, 0, 0, 0);
IPAddress netMask     = IPAddress(0, 0, 0, 0);
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
#ifdef USE_SSL
WiFiClientSecure *client     = NULL;  // SSL/TLS
#else
WiFiClient *client           = NULL;  // NON-SSL/TLS
#endif
PubSubClient   *mqtt         = NULL;
String ClientNID = UID + String(ESP_getChipId(), HEX);
#ifdef ESP32
const uint16_t kRecvPin = 16;
const uint16_t kIrLed = 27;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
const int d[] = {34,35,32,33};
bool vd[] = {0,0,0,0};
bool lvd[] = {0,0,0,0};
#else
const uint16_t kRecvPin = 14;
const uint16_t kIrLed = 12;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
#endif
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
// IRGreeAC ac(kIrLed);  // Set the GPIO to be used for sending AC messages.
IRac irac(kIrLed);  // Set the GPIO to be used for sending AC messages.
IRsend irsend(kIrLed);  // Set the GPIO to be used to sending the message.
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type
stdAc::state_t  ACParam;
bool pubacmodeskip = false;
String serin = "";

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
  MQTT_connect();
  // if (mqtt->publish(String(MQTT_Pub_Topic + "/sensors/temperature").c_str(),String(aht10.readTemperature()).c_str())) {
  if (mqtt->publish((MQTT_Status_Topic + "/sensors/temperature").c_str(),String(aht10.readTemperature()).c_str())) {
    Serial.print(F("T"));        // T means publishing OK
    mqttfailcount = 0 ;
  }
  else {
    Serial.print(F("F"));        // F means publishing failure
    mqttfailcount = mqttfailcount+1 ;
  }
  // if (mqtt->publish(String(MQTT_Pub_Topic + "/sensors/humidity").c_str(),String(aht10.readHumidity()).c_str())) {
  if (mqtt->publish((MQTT_Status_Topic + "/sensors/humidity").c_str(),String(aht10.readHumidity()).c_str())) {
    Serial.print(F("H"));        // H means publishing OK
    mqttfailcount = 0 ;
  }
  else {
    Serial.print(F("F"));        // F means publishing failure
    mqttfailcount = mqttfailcount+1 ;
  }
  if (mqttfailcount >= 3) {
    servernet = !servernet;
    deleteOldInstances();
    createNewInstances();
    if (mqttfailcount >= 10) 
#if ESP8266
    ESP.reset();
#else
    ESP.restart();
#endif
  }
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
      PublishACState(pubacmodeskip);
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
  if (client) {
    delete client;
    client = NULL;
    Serial.println(F("Deleting old WiFi object"));
  }
  if (mqtt) {
    delete mqtt;
    mqtt = NULL;
    Serial.println(F("Deleting old MQTT object"));
  }  
}

// write this format on fingerprint/root ca param (add "\n" for every line in root ca cert, make sure there is no space!) ex : 
// -----BEGIN CERTIFICATE-----\nMIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n…......................................emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n-----END CERTIFICATE-----\n
#ifdef ESP32
void replacenewline(char* str) {
    char* p = str;
    while ((p = strstr(p, "\\n")) != NULL) {
        *p++ = '\n';
        memmove(p, p + 1, strlen(p));
    }
} // menyesuaikan format root ca dari param wm cp
#endif

void createNewInstances() {
  if (!client) {
#ifdef USE_SSL
    client = new WiFiClientSecure(); // SSL/TLS
#ifdef ESP32
    replacenewline(custom_FINGERPRINT);
    if (servernet == custom_SERVERPRIO) client->setInsecure();
    else client->setCACert(custom_FINGERPRINT);
#else
    client->setFingerprint(custom_FINGERPRINT); // SSL/TLS Required
#endif
#else
    client = new WiFiClient;  //NON-TLS
#endif
    Serial.print(F("\nCreating new WiFi client object : "));
    Serial.println(client ? F("OK") : F("failed"));
  }
  if (!mqtt) {  // Create new instances from new data
    //Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
    // mqtt = new PubSubClient(*client);
    MQTT_Sub_Topic = String(custom_USERNAME) + "/" + ClientNID;
    if (servernet == custom_SERVERPRIO) {
      mqtt = new PubSubClient(custom_SERVER_LOCAL,atoi(custom_SERVERPORT),callback,*client);
      // mqtt->setServer(custom_SERVER_LOCAL, atoi(custom_SERVERPORT));
      Serial.println(F("PAKE SERVER LOCAL"));
      }
    else { 
      mqtt = new PubSubClient(custom_SERVER,atoi(custom_SERVERPORT),callback,*client);
      // mqtt->setServer(custom_SERVER, atoi(custom_SERVERPORT));
      Serial.println(F("PAKE SERVER IOT"));
      }
    mqtt->setBufferSize(512);
    mqtt->connect(ClientNID.c_str(), custom_USERNAME, custom_KEY, (MQTT_Pub_Topic + "/status").c_str(), 0, true, "offline");    
    mqtt->subscribe((MQTT_Sub_Topic + "/set/#").c_str());
    Serial.print(F("Creating new MQTT object : "));
    if (mqtt) {
      Serial.println(F("OK"));
      Serial.println(String("SERVER = ")    + custom_SERVER    + ", SERVER LOCAL = "  + custom_SERVER_LOCAL + ", INET PRIORITY = " + custom_SERVERPRIO);
      Serial.println(String("USERNAME = ")  + custom_USERNAME  + ", PASSWORD = "           + custom_KEY + ", SERVERPORT = " + custom_SERVERPORT);
#ifdef ESP32
      Serial.println(String("ROOTCACERT = ")  + custom_FINGERPRINT );
#else
      Serial.println(String("FINGERPRINT = ")  + custom_FINGERPRINT );
#endif
    }
    else Serial.println(F("Failed Create MQTT Instance"));
  }
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
    ESP_wifiManager.setConfigPortalTimeout(120);
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
  char chtml[26] = "type=\"checkbox\"";
  if (custom_SERVERPRIO == 1) strcat(chtml, " checked");
  else strcpy(chtml, "type=\"checkbox\"");
  ESP_WMParameter SERVER_FIELD(SERVER_Label, "SERVER", custom_SERVER, custom_SERVER_LEN + 1);  // SERVER
  ESP_WMParameter SERVER_LOCAL_FIELD(SERVER_LOCAL_Label, "SERVER LOCAL", custom_SERVER_LOCAL, custom_SERVER_LOCAL_LEN + 1);  // LOCAL SERVER
  ESP_WMParameter SERVERPORT_FIELD(SERVERPORT_Label, "SERVER PORT", custom_SERVERPORT, custom_PORT_LEN + 1);  // SERVERPORT
  ESP_WMParameter SERVERPRIO_FIELD(SERVERPRIO_Label, "INET(IOT) PRIORITY", "Y", 2, chtml, WFM_LABEL_AFTER);  // SSL/TLS
  ESP_WMParameter USERNAME_FIELD(USERNAME_Label, "USERNAME", custom_USERNAME, custom_USERNAME_LEN + 1);  // USERNAME
  ESP_WMParameter KEY_FIELD(KEY_Label, "PASSWORD", custom_KEY, custom_KEY_LEN + 1);  // PASSWORD
#ifdef USE_SSL
#ifdef ESP32
  ESP_WMParameter FINGERPRINT_FIELD(FINGERPRINT_Label, "ROOT CA CERT ", custom_FINGERPRINT, custom_FINGERPRINT_LEN + 1);  // FINGERPRINT
#else
  ESP_WMParameter FINGERPRINT_FIELD(FINGERPRINT_Label, "FINGERPRINT", custom_FINGERPRINT, custom_FINGERPRINT_LEN + 1);  // FINGERPRINT
#endif
#endif
  // add all parameters here
  ESP_wifiManager.addParameter(&SERVER_FIELD);
  ESP_wifiManager.addParameter(&SERVER_LOCAL_FIELD);
  ESP_wifiManager.addParameter(&SERVERPORT_FIELD);
  ESP_wifiManager.addParameter(&SERVERPRIO_FIELD);
  ESP_wifiManager.addParameter(&USERNAME_FIELD);
  ESP_wifiManager.addParameter(&KEY_FIELD);
#ifdef USE_SSL
  ESP_wifiManager.addParameter(&FINGERPRINT_FIELD);
#endif
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
    Serial.print(F("Subnet Mask: "));
    Serial.println(WiFi.subnetMask());
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
  strcpy(custom_SERVER_LOCAL, SERVER_LOCAL_FIELD.getValue());
  strcpy(custom_SERVERPORT, SERVERPORT_FIELD.getValue());
  custom_SERVERPRIO = (strncmp(SERVERPRIO_FIELD.getValue(), "Y", 1) == 0);
  strcpy(custom_USERNAME, USERNAME_FIELD.getValue());
  strcpy(custom_KEY, KEY_FIELD.getValue());
#ifdef USE_SSL
  strcpy(custom_FINGERPRINT, FINGERPRINT_FIELD.getValue());
#endif
  writeConfigFile();  // Writing JSON config file to flash for next boot
  digitalWrite(LED_BUILTIN, LED_OFF); // Turn LED off as we are not in configuration mode.
  deleteOldInstances();
  MQTT_Pub_Topic = String(custom_USERNAME) + "/" + ClientNID ;
  MQTT_Status_Topic = MQTT_Pub_Topic + "/state";
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
    if (json.containsKey(SERVER_LOCAL_Label)) {
      strcpy(custom_SERVER_LOCAL, json[SERVER_LOCAL_Label]);
    }
    if (json.containsKey(SERVERPORT_Label)) {
      strcpy(custom_SERVERPORT, json[SERVERPORT_Label]);
    }
    if (json.containsKey(SERVERPRIO_Label)) {
      custom_SERVERPRIO = json[SERVERPRIO_Label];
    }
    if (json.containsKey(USERNAME_Label)) {
      strcpy(custom_USERNAME, json[USERNAME_Label]);
    }
    if (json.containsKey(KEY_Label)) {
      strcpy(custom_KEY, json[KEY_Label]);
    }
#ifdef USE_SSL
    if (json.containsKey(FINGERPRINT_Label)) {
      strcpy(custom_FINGERPRINT, json[FINGERPRINT_Label]);
    }
#endif
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
  json[SERVER_Label]            = custom_SERVER;
  json[SERVER_LOCAL_Label]      = custom_SERVER_LOCAL;
  json[SERVERPORT_Label]        = custom_SERVERPORT;
  json[SERVERPRIO_Label]        = custom_SERVERPRIO;
  json[USERNAME_Label]          = custom_USERNAME;
  json[KEY_Label]               = custom_KEY;
#ifdef USE_SSL
  json[FINGERPRINT_Label]       = custom_FINGERPRINT;
#endif
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
  Serial.print(F("custom_SERVER_LOCAL: "));
  Serial.println(custom_SERVER_LOCAL);
  Serial.print(F("custom_SERVERPORT: "));
  Serial.println(custom_SERVERPORT);
  Serial.print(F("custom_SERVERPRIO: "));
  Serial.println(custom_SERVERPRIO);
  Serial.print(F("custom_USERNAME: "));
  Serial.println(custom_USERNAME);
  Serial.print(F("custom_PASSWORD: "));
  Serial.println(custom_KEY);
#ifdef USE_SSL
#ifdef ESP32
  Serial.print(F("custom_ROOTCACERT: "));
  Serial.println(custom_FINGERPRINT);
#else
  Serial.print(F("custom_FINGERPRINT: "));
  Serial.println(custom_FINGERPRINT);
#endif
#endif
  Serial.println();
}

void MQTT_connect() {
  bool ret;
  MQTT_Pub_Topic = String(custom_USERNAME) + "/" + ClientNID;
  MQTT_Status_Topic = String(custom_USERNAME) + "/" + ClientNID + "/state";
  createNewInstances();
  // Return if already connected
  if (mqtt->connected()) {
    mqtt->publish((MQTT_Pub_Topic + "/status").c_str(), "online");
    return;
  }
  Serial.println(F("Connecting to MQTT (3 attempts)..."));
  uint8_t attempt = 3;
  while ((ret = mqtt->connected()) != 1) {    // connect will return 0 for connected
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

// void printState() {
//   Serial.println(F("GREE A/C remote is in the following state:"));  // Display the settings.
//   Serial.printf("  %s\n", ac.toString().c_str());  // Display the encoded IR sequence.
//   unsigned char* ir_code = ac.getRaw();
//   Serial.print(F("IR Code: 0x"));
//   for (uint8_t i = 0; i < kGreeStateLength; i++)
//     Serial.printf("%02X", ir_code[i]);
//   Serial.println();
//   Serial.println();
// }


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("Message received on topic: "));
  Serial.println(topic);
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Message: " + message);
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/light") ac.setLight(message.toInt());
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/turbo") ac.setTurbo(message.toInt());
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/xfan") ac.setXFan(message.toInt());
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/sleep") ac.setSleep(message.toInt());
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/temperature") ac.setTemp(message.toInt());
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/power") ac.setPower(message.toInt());
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "off") ac.setPower(0);
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message != "off") {
  //   if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "auto") ac.setMode(0);  //ac.setMode(kGreeAuto);
  //   else if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "cool") ac.setMode(1);  //ac.setMode(kGreeCool); 
  //   else if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "dry") ac.setMode(2);  //ac.setMode(kGreeDry);
  //   else if (String(topic) == MQTT_Sub_Topic + "/set/ac/mode" && message == "fan_only") ac.setMode(3);  //ac.setMode(kGreeFan);
  // //  else if (String(topic) == "nodemcuv3//set/ac/mode" && message == "heat") ac.setMode(4);  //ac.setMode(kGreeHeat); 
  //   ac.setPower(1);
  // }
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Auto") ac.setFan(0);  //ac.setFan(kGreeFanAuto);
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Minimum") ac.setFan(1);  //ac.setFan(kGreeFanMin);
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Medium") ac.setFan(2);  //ac.setFan(kGreeFanMed);
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/fan" && message == "Maximum") ac.setFan(3);  //ac.setFan(kGreeFanMax);
  // if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Last Pos") ac.setSwingVertical(0,0);  //ac.setSwingVertical(false,kGreeSwingLastPos);  //0b0000
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Auto") ac.setSwingVertical(1,1);  //ac.setSwingVertical(true,kGreeSwingAuto);  //0b0001
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Up") ac.setSwingVertical(0,2);  //ac.setSwingVertical(false,kGreeSwingUp);  //0b0010
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle Up") ac.setSwingVertical(0,3);  //ac.setSwingVertical(false,kGreeSwingMiddleUp);  //0b0011
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle") ac.setSwingVertical(0,4);  //ac.setSwingVertical(false,kGreeSwingMiddle);  //0b0100
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle Down") ac.setSwingVertical(0,5);  //ac.setSwingVertical(false,kGreeSwingMiddleDown);  //0b0101
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Down") ac.setSwingVertical(0,6);  //ac.setSwingVertical(false,kGreeSwingDown);  //0b0110
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Down Auto") ac.setSwingVertical(1,7);  //ac.setSwingVertical(true,kGreeSwingDownAuto);  //0b0111
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Middle Auto") ac.setSwingVertical(1,9);  //ac.setSwingVertical(true,kGreeSwingMiddleAuto);  //0b1001
  // else if (String(topic) == MQTT_Sub_Topic + "/set/ac/swing" && message == "Up Auto") ac.setSwingVertical(1,11);  //ac.setSwingVertical(true,kGreeSwingUpAuto);  //0b1011
  // if (strstr(topic, (MQTT_Sub_Topic + "/set/ac").c_str()) != NULL) {
  //   printState();
  //   PublishACState();
  //   ac.send();
  // }
//  if (strstr(topic, (MQTT_Sub_Topic + "/universal").c_str()) != NULL) irsend.send(PANASONIC, 0x40040100BCBD, 48, 0);
//  if (String(topic) == (MQTT_Sub_Topic + "/universal/tv-panasonic").c_str()) irsend.send(PANASONIC, (strtoull(message.c_str(), NULL, 16)), 48, 0);  //HEX Code manually by MQTT msg, ex : 0x40040100BCBD
  // if (strstr(topic, "/panasonic") != NULL) irsend.send(PANASONIC, (strtoull(message.c_str(), NULL, 16)), 48, 0);  //HEX Code manually by MQTT msg, ex : 0x40040100BCBD
  // if (strstr(topic, "/gree") != NULL) irsend.send(GREE, (strtoull(message.c_str(), NULL, 16)), 64, 0);  //HEX Code manually by MQTT msg, ex : 0x40040100BCBD


    //When got topic ended with "/AC"
  if (String(topic) == MQTT_Sub_Topic + "/set/ac") {
    DynamicJsonDocument jmqttdata(512);
    DeserializationError error = deserializeJson(jmqttdata, message);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    ACParam.protocol = protocol(jmqttdata["Protocol"]);
    ACParam.model = irac.strToModel(jmqttdata["Model"]);
    ACParam.power = jmqttdata["Power"];
    ACParam.mode = irac.strToOpmode(jmqttdata["Mode"]);
    ACParam.degrees = jmqttdata["Temperature"];
    ACParam.celsius = jmqttdata["Celsius"];
    ACParam.fanspeed = irac.strToFanspeed(jmqttdata["Fan"]);
    ACParam.swingv = irac.strToSwingV(jmqttdata["SwingV"]);
    ACParam.swingh = irac.strToSwingH(jmqttdata["SwingH"]);
    ACParam.quiet = jmqttdata["Quiet"];
    ACParam.turbo = jmqttdata["Turbo"];
    ACParam.econo = jmqttdata["Eco"];
    ACParam.light = jmqttdata["Light"];
    ACParam.filter = jmqttdata["Filter"];
    ACParam.clean = jmqttdata["Clean"];
    ACParam.beep = jmqttdata["Beep"];
    ACParam.sleep = jmqttdata["Sleep"];
    ACParam.clock = jmqttdata["Clock"];
    irac.sendAc(ACParam);
    jmqttdata["Protocol"] = ACParam.protocol;
    jmqttdata["Model"] = ACParam.model;
    jmqttdata["Power"] = ACParam.power; 
    jmqttdata["Mode"] = irac.opmodeToString(ACParam.mode); 
    jmqttdata["Temperature"] = ACParam.degrees; 
    jmqttdata["Celsius"] = ACParam.celsius; 
    jmqttdata["Fan"] = irac.fanspeedToString(ACParam.fanspeed);
    jmqttdata["SwingV"] = irac.swingvToString(ACParam.swingv);
    jmqttdata["SwingH"] = irac.swinghToString(ACParam.swingh);
    jmqttdata["Quiet"] = ACParam.quiet;
    jmqttdata["Turbo"] = ACParam.turbo; 
    jmqttdata["Eco"] = ACParam.econo;
    jmqttdata["Light"] = ACParam.light;
    jmqttdata["Filter"] = ACParam.filter; 
    jmqttdata["Clean"] = ACParam.clean; 
    jmqttdata["Beep"] = ACParam.beep; 
    jmqttdata["Sleep"] = ACParam.sleep; 
    jmqttdata["Clock"] = ACParam.clock;
    char buffr[512];
    size_t n = serializeJsonPretty(jmqttdata, buffr);
    mqtt->publish((String(MQTT_Status_Topic)+"/ac").c_str(), buffr, n);
    PublishACState(pubacmodeskip);
    if (!n) {
      Serial.print(F("serializeJsonPretty() failed: "));
      Serial.println(error.f_str());
      return;
    }
    Serial.println(F("Setting AC with Parameter : "));
    serializeJsonPretty(jmqttdata, Serial);
  }
  else if (strstr(topic, "set/ac/") != NULL) {
    String topicStr = String(topic);
    int LastIndex = topicStr.lastIndexOf('/');
    int SecondLastIndex = topicStr.lastIndexOf('/', LastIndex - 1);
    int ThirdLastIndex = topicStr.lastIndexOf('/', SecondLastIndex - 1);
    String param = topicStr.substring(LastIndex + 1);
    String proto = topicStr.substring(SecondLastIndex + 1, LastIndex);
    String mdl = topicStr.substring(ThirdLastIndex + 1, SecondLastIndex);
    param.toUpperCase();
    ACParam.model=irac.strToModel(mdl.c_str());
    mqtt->publish((String(MQTT_Status_Topic)+"/ac/model").c_str(),String(ACParam.model).c_str());
    ACParam.protocol=protocol(proto);
    mqtt->publish((String(MQTT_Status_Topic)+"/ac/protocol").c_str(),String(ACParam.protocol).c_str());
    if (param == "LIGHT") ACParam.light=message.toInt();
    else if (param == "TURBO") ACParam.turbo=message.toInt();
    else if (param == "XFAN") ACParam.clean=message.toInt();
    else if (param == "SLEEP") ACParam.sleep=message.toInt();
    else if (param == "CLOCK") ACParam.clock=message.toInt();
    else if (param == "TEMPERATURE") ACParam.degrees=message.toInt();
    else if (param == "QUIET") ACParam.quiet=message.toInt();
    else if (param == "ECO") ACParam.econo=message.toInt();
    else if (param == "FILTER") ACParam.filter=message.toInt();
    else if (param == "BEEP") ACParam.beep=message.toInt();
    else if (param == "CELSIUS") ACParam.celsius=message.toInt();
    else if (param == "FAN") ACParam.fanspeed=irac.strToFanspeed(message.c_str());
    else if (param == "SWINGV") ACParam.swingv=irac.strToSwingV(message.c_str());
    else if (param == "SWINGH") ACParam.swingh=irac.strToSwingH(message.c_str());
    // Adjustment to Home Assistant MQTT HVAC
    else if (param == "POWER") {
      ACParam.power=message.toInt();
      if (ACParam.power != 0) pubacmodeskip = false;
      else {
        pubacmodeskip = true;
        mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "off");
      }
    }
    if (param == "MODE") {
      if (message != "off") {
        ACParam.mode = irac.strToOpmode(message.c_str());
        ACParam.power = true;
        pubacmodeskip = false;
      }
      else {
        ACParam.power = false;
        pubacmodeskip = true;
        mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "off");
      }
    }
    irac.sendAc(ACParam);
    PublishACState(pubacmodeskip);
  }
  if (String(topic) == MQTT_Sub_Topic + "/set/universal") {
    DynamicJsonDocument jmqttdata(512);
    DeserializationError error = deserializeJson(jmqttdata, message);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    irsend.send(protocol(jmqttdata["Protocol"]),(strtoull(jmqttdata["Code"], NULL, 16)), jmqttdata["Bit"], jmqttdata["Repeat"]);
    Serial.println(F("Sending IR Code with Parameter : "));
    serializeJsonPretty(jmqttdata, Serial);
  }
  else if (strstr(topic, "/set/universal/") != NULL) {
    String topicStr = String(topic);
    int LastIndex = topicStr.lastIndexOf('/');
    int SecondLastIndex = topicStr.lastIndexOf('/', LastIndex - 1);
    String jbit = topicStr.substring(LastIndex + 1);
    String proto = topicStr.substring(SecondLastIndex + 1, LastIndex);
    irsend.send(protocol(proto),(strtoull(message.c_str(), NULL, 16)), jbit.toInt(), 0);    //HEX Code manually by MQTT msg, ex : 0x40040100BCBD
  }
  if (String(topic) == (MQTT_Sub_Topic + "/set/ir-receiver/state") && message == "1") irrecst = true;
  else if (String(topic) == (MQTT_Sub_Topic + "/set/ir-receiver/state") && message == "0") irrecst = false;

  if (String(topic) == MQTT_Sub_Topic + "/set/cmd") {
    message.toUpperCase();
    if (message == "RESET" || message == "REBOOT")
#if ESP8266
    ESP.reset();
#else
    ESP.restart();
#endif
  }
}

decode_type_t protocol(String pcek) {
    pcek.toUpperCase();
    if (pcek == "UNKNOWN") return UNKNOWN;
    else if (pcek == "RC5") return RC5;
    else if (pcek == "RC6") return RC6;
    else if (pcek == "NEC") return NEC;
    else if (pcek == "SONY") return SONY;
    else if (pcek == "PANASONIC") return PANASONIC;
    else if (pcek == "JVC") return JVC;
    else if (pcek == "SAMSUNG") return SAMSUNG;
    else if (pcek == "WHYNTER") return WHYNTER;
    else if (pcek == "AIWA_RC_T501") return AIWA_RC_T501;
    else if (pcek == "LG") return LG;
    else if (pcek == "SANYO") return SANYO;
    else if (pcek == "MITSUBISHI") return MITSUBISHI;
    else if (pcek == "DISH") return DISH;
    else if (pcek == "SHARP") return SHARP;
    else if (pcek == "COOLIX") return COOLIX;
    else if (pcek == "DAIKIN") return DAIKIN;
    else if (pcek == "DENON") return DENON;
    else if (pcek == "KELVINATOR") return KELVINATOR;
    else if (pcek == "SHERWOOD") return SHERWOOD;
    else if (pcek == "MITSUBISHI_AC") return MITSUBISHI_AC;
    else if (pcek == "RCMM") return RCMM;
    else if (pcek == "SANYO_LC7461") return SANYO_LC7461;
    else if (pcek == "RC5X") return RC5X;
    else if (pcek == "GREE") return GREE;
    else if (pcek == "PRONTO") return PRONTO;
    else if (pcek == "NEC_LIKE") return NEC_LIKE;
    else if (pcek == "ARGO") return ARGO;
    else if (pcek == "TROTEC") return TROTEC;
    else if (pcek == "NIKAI") return NIKAI;
    else if (pcek == "RAW") return RAW;
    else if (pcek == "GLOBALCACHE") return GLOBALCACHE;
    else if (pcek == "TOSHIBA_AC") return TOSHIBA_AC;
    else if (pcek == "FUJITSU_AC") return FUJITSU_AC;
    else if (pcek == "MIDEA") return MIDEA;
    else if (pcek == "MAGIQUEST") return MAGIQUEST;
    else if (pcek == "LASERTAG") return LASERTAG;
    else if (pcek == "CARRIER_AC") return CARRIER_AC;
    else if (pcek == "HAIER_AC") return HAIER_AC;
    else if (pcek == "MITSUBISHI2") return MITSUBISHI2;
    else if (pcek == "HITACHI_AC") return HITACHI_AC;
    else if (pcek == "HITACHI_AC1") return HITACHI_AC1;
    else if (pcek == "HITACHI_AC2") return HITACHI_AC2;
    else if (pcek == "GICABLE") return GICABLE;
    else if (pcek == "HAIER_AC_YRW02") return HAIER_AC_YRW02;
    else if (pcek == "WHIRLPOOL_AC") return WHIRLPOOL_AC;
    else if (pcek == "SAMSUNG_AC") return SAMSUNG_AC;
    else if (pcek == "LUTRON") return LUTRON;
    else if (pcek == "ELECTRA_AC") return ELECTRA_AC;
    else if (pcek == "PANASONIC_AC") return PANASONIC_AC;
    else if (pcek == "PIONEER") return PIONEER;
    else if (pcek == "LG2") return LG2;
    else if (pcek == "MWM") return MWM;
    else if (pcek == "DAIKIN2") return DAIKIN2;
    else if (pcek == "VESTEL_AC") return VESTEL_AC;
    else if (pcek == "TECO") return TECO;
    else if (pcek == "SAMSUNG36") return SAMSUNG36;
    else if (pcek == "TCL112AC") return TCL112AC;
    else if (pcek == "LEGOPF") return LEGOPF;
    else if (pcek == "MITSUBISHI_HEAVY_88") return MITSUBISHI_HEAVY_88;
    else if (pcek == "MITSUBISHI_HEAVY_152") return MITSUBISHI_HEAVY_152;
    else if (pcek == "DAIKIN216") return DAIKIN216;
    else if (pcek == "SHARP_AC") return SHARP_AC;
    else if (pcek == "GOODWEATHER") return GOODWEATHER;
    else if (pcek == "INAX") return INAX;
    else if (pcek == "DAIKIN160") return DAIKIN160;
    else if (pcek == "NEOCLIMA") return NEOCLIMA;
    else if (pcek == "DAIKIN176") return DAIKIN176;
    else if (pcek == "DAIKIN128") return DAIKIN128;
    else if (pcek == "AMCOR") return AMCOR;
    else if (pcek == "DAIKIN152") return DAIKIN152;
    else if (pcek == "MITSUBISHI136") return MITSUBISHI136;
    else if (pcek == "MITSUBISHI112") return MITSUBISHI112;
    else if (pcek == "HITACHI_AC424") return HITACHI_AC424;
    else if (pcek == "SONY_38K") return SONY_38K;
    else if (pcek == "EPSON") return EPSON;
    else if (pcek == "SYMPHONY") return SYMPHONY;
    else if (pcek == "HITACHI_AC3") return HITACHI_AC3;
    else if (pcek == "DAIKIN64") return DAIKIN64;
    else if (pcek == "AIRWELL") return AIRWELL;
    else if (pcek == "DELONGHI_AC") return DELONGHI_AC;
    else if (pcek == "DOSHISHA") return DOSHISHA;
    else if (pcek == "MULTIBRACKETS") return MULTIBRACKETS;
    else if (pcek == "CARRIER_AC40") return CARRIER_AC40;
    else if (pcek == "CARRIER_AC64") return CARRIER_AC64;
    else if (pcek == "HITACHI_AC344") return HITACHI_AC344;
    else if (pcek == "CORONA_AC") return CORONA_AC;
    else if (pcek == "MIDEA24") return MIDEA24;
    else if (pcek == "ZEPEAL") return ZEPEAL;
    else if (pcek == "SANYO_AC") return SANYO_AC;
    else if (pcek == "VOLTAS") return VOLTAS;
    else if (pcek == "METZ") return METZ;
    else if (pcek == "TRANSCOLD") return TRANSCOLD;
    else if (pcek == "TECHNIBEL_AC") return TECHNIBEL_AC;
    else if (pcek == "MIRAGE") return MIRAGE;
    else if (pcek == "ELITESCREENS") return ELITESCREENS;
    else if (pcek == "PANASONIC_AC32") return PANASONIC_AC32;
    else if (pcek == "MILESTAG2") return MILESTAG2;
    else if (pcek == "ECOCLIM") return ECOCLIM;
    else if (pcek == "XMP") return XMP;
    else if (pcek == "TRUMA") return TRUMA;
    else if (pcek == "HAIER_AC176") return HAIER_AC176;
    else if (pcek == "TEKNOPOINT") return TEKNOPOINT;
    else if (pcek == "KELON") return KELON;
    else if (pcek == "TROTEC_3550") return TROTEC_3550;
    else if (pcek == "SANYO_AC88") return SANYO_AC88;
    else if (pcek == "BOSE") return BOSE;
    else if (pcek == "ARRIS") return ARRIS;
    else if (pcek == "RHOSS") return RHOSS;
    else if (pcek == "AIRTON") return AIRTON;
    else if (pcek == "COOLIX48") return COOLIX48;
    else if (pcek == "HITACHI_AC264") return HITACHI_AC264;
    else if (pcek == "KELON168") return KELON168;
    else if (pcek == "HITACHI_AC296") return HITACHI_AC296;
    else if (pcek == "DAIKIN200") return DAIKIN200;
    else if (pcek == "HAIER_AC160") return HAIER_AC160;
    else if (pcek == "CARRIER_AC128") return CARRIER_AC128;
    else if (pcek == "TOTO") return TOTO;
    else if (pcek == "CLIMABUTLER") return CLIMABUTLER;
    else if (pcek == "TCL96AC") return TCL96AC;
    else if (pcek == "BOSCH144") return BOSCH144;
    else if (pcek == "SANYO_AC152") return SANYO_AC152;
    else if (pcek == "DAIKIN312") return DAIKIN312;
    else if (pcek == "GORENJE") return GORENJE;
    else if (pcek == "WOWWEE") return WOWWEE;
    else if (pcek == "CARRIER_AC84") return CARRIER_AC84;
    else if (pcek == "YORK") return YORK;
    else return UNUSED;
}

void PublishACState(bool skipmode) {
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/model").c_str(),String(ACParam.model).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/protocol").c_str(),String(ACParam.protocol).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/light").c_str(),String(ACParam.light).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/turbo").c_str(),String(ACParam.turbo).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/xfan").c_str(),String(ACParam.clean).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/sleep").c_str(),String(ACParam.sleep).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/clock").c_str(),String(ACParam.clock).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/temperature").c_str(),String(ACParam.degrees).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/quiet").c_str(),String(ACParam.quiet).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/eco").c_str(),String(ACParam.econo).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/filter").c_str(),String(ACParam.filter).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/beep").c_str(),String(ACParam.beep).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/celsius").c_str(),String(ACParam.celsius).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/power").c_str(),String(ACParam.power).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(),irac.fanspeedToString(ACParam.fanspeed).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/swingv").c_str(),irac.swingvToString(ACParam.swingv).c_str());
  mqtt->publish((String(MQTT_Status_Topic)+"/ac/swingh").c_str(),irac.swinghToString(ACParam.swingh).c_str());
  // Adjustment to Home Assistant MQTT HVAC
  if (skipmode==0) {
    if (irac.opmodeToString(ACParam.mode) == "Off") mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "off");
    else if (irac.opmodeToString(ACParam.mode) == "Cool") mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "cool");
    else if (irac.opmodeToString(ACParam.mode) == "Dry") mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "dry");
    else if (irac.opmodeToString(ACParam.mode) == "Fan") mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "fan_only");
    else if (irac.opmodeToString(ACParam.mode) == "Auto") mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "auto");
  }
  Serial.println(F("AC State Updated"));
//   mqtt->publish((String(MQTT_Status_Topic)+"/ac/light").c_str(),String(ac.getLight()).c_str());
//   mqtt->publish((String(MQTT_Status_Topic)+"/ac/turbo").c_str(),String(ac.getTurbo()).c_str());
//   mqtt->publish((String(MQTT_Status_Topic)+"/ac/xfan").c_str(),String(ac.getXFan()).c_str());
//   mqtt->publish((String(MQTT_Status_Topic)+"/ac/sleep").c_str(),String(ac.getSleep()).c_str());
//   mqtt->publish((String(MQTT_Status_Topic)+"/ac/temperature").c_str(),String(ac.getTemp()).c_str());
//   mqtt->publish((String(MQTT_Status_Topic)+"/ac/power").c_str(),String(ac.getPower()).c_str());
//   if (ac.getPower() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(), "off");
//   else if (ac.getMode() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"auto");
//   else if (ac.getMode() == 1) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"cool");
//   else if (ac.getMode() == 2) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"dry");
//   else if (ac.getMode() == 3) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"fan_only");
// //  else if (ac.getMode() == 4) mqtt->publish((String(MQTT_Status_Topic)+"/ac/mode").c_str(),"heat");
//   if (ac.getFan() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Auto");
//   else if (ac.getFan() == 1) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Minimum");
//   else if (ac.getFan() == 2) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Medium");
//   else if (ac.getFan() == 3) mqtt->publish((String(MQTT_Status_Topic)+"/ac/fan").c_str(), "Maximum");
//   if (ac.getSwingVerticalPosition() == 0) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Last Pos");
//   else if (ac.getSwingVerticalPosition() == 1) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Auto");
//   else if (ac.getSwingVerticalPosition() == 2) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Up");
//   else if (ac.getSwingVerticalPosition() == 3) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle Up");
//   else if (ac.getSwingVerticalPosition() == 4) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle");
//   else if (ac.getSwingVerticalPosition() == 5) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle Down");
//   else if (ac.getSwingVerticalPosition() == 6) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Down");
//   else if (ac.getSwingVerticalPosition() == 7) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Down Auto");
//   else if (ac.getSwingVerticalPosition() == 9) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Middle Auto");
//   else if (ac.getSwingVerticalPosition() == 11) mqtt->publish((String(MQTT_Status_Topic)+"/ac/swing").c_str(), "Up Auto");
}

// void printacparam() {
//     Serial.print(F("Protocol : "));Serial.println(ACParam.protocol);
//     Serial.print(F("Model : "));Serial.println(ACParam.model);
//     Serial.print(F("Power : "));Serial.println(ACParam.power);
//     Serial.print(F("Mode : "));Serial.println(irac.opmodeToString(ACParam.mode));
//     Serial.print(F("Temperature : "));Serial.println(ACParam.degrees);
//     Serial.print(F("Celsius : "));Serial.println(ACParam.celsius);
//     Serial.print(F("FanSpeed : "));Serial.println(irac.fanspeedToString(ACParam.fanspeed));
//     Serial.print(F("SwingV : "));Serial.println(irac.swingvToString(ACParam.swingv));
//     Serial.print(F("SwingH : "));Serial.println(irac.swinghToString(ACParam.swingh));
//     Serial.print(F("Quiet : "));Serial.println(ACParam.quiet);
//     Serial.print(F("Turbo : "));Serial.println(ACParam.turbo);
//     Serial.print(F("Eco : "));Serial.println(ACParam.econo);
//     Serial.print(F("Light : "));Serial.println(ACParam.light);
//     Serial.print(F("Filter : "));Serial.println(ACParam.filter);
//     Serial.print(F("Clean : "));Serial.println(ACParam.clean);
//     Serial.print(F("Beep : "));Serial.println(ACParam.beep);
//     Serial.print(F("Sleep : "));Serial.println(ACParam.sleep);
//     Serial.print(F("Clock : "));Serial.println(ACParam.clock);
// }


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
#ifdef ESP32
  for (int i = 0; i < 4; i++) pinMode(d[i],INPUT);
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
    Serial.println(F("\nEnd"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println(F("Auth Failed"));
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println(F("Begin Failed"));
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println(F("Connect Failed"));
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println(F("Receive Failed"));
    } else if (error == OTA_END_ERROR) {
      Serial.println(F("End Failed"));
    }
  });
  ArduinoOTA.begin();
#endif
//=================================================================================================================================
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
  // Serial.println(F("Default state of the remote."));
  // Serial.println(F("Setting desired state for A/C."));
  // ac.begin();
  // ac.setUseFahrenheit(false); //ac.setUseFahrenheit(false), ac.setUseFahrenheit(true)
  // ac.setDisplayTempSource(0);
  // printState();
#ifdef USE_SSL
  Serial.println(F("SSL/TLS"));
#else
  Serial.println(F("NON-SSL/ TLS"));
#endif
}

// Loop function
void loop() {
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  if (drd) drd->loop();
  if(mqtt) mqtt->loop();
#ifdef ESP32
  ArduinoOTA.handle();
  for (int i = 0; i < 4; i++) {
    vd[i] = digitalRead(d[i]);
    if (lvd[i] != vd[i]) {
      lvd[i] = vd[i];
      Serial.print(F("D"));
      Serial.print(i);
      Serial.print(F(" = "));
      Serial.println(vd[i]);
      mqtt->publish((MQTT_Status_Topic + "/rf/d" + i).c_str(), String(vd[i]).c_str());
    }
  }
#endif
  check_status();  // this is just for checking if we are connected to WiFi  
  if (irrecst == true && irrecv.decode(&results)) {
    Serial.println();
    Serial.print(resultToHumanReadableBasic(&results));
    //    String description = IRAcUtils::resultAcToString(&results);
    //    if (description.length()) Serial.println(D_STR_MESGDESC ": " + description);
    //    yield();  // Feed the WDT as the text output can take a while to print.
    Serial.print(F("\nSending HEX code value......."));
//    if (! mqtt->publish((String(MQTT_Pub_Topic) + "/ir-receiver/code").c_str(), String(results.value, HEX).c_str())) {
//      Serial.println(F("Failed Sending Code"));
    if (!mqtt->publish((MQTT_Status_Topic + "/ir-receiver/code").c_str(), String(resultToHumanReadableBasic(&results)).c_str())) {
      Serial.println(F("Failed Sending Code"));
    } else {
      mqtt->publish((MQTT_Status_Topic + "/ir-receiver/hexcode").c_str(), String(resultToHexidecimal(&results)).c_str());
      Serial.println(F("Code Sent!"));
    }
    Serial.print(F("HEX Code : ")); Serial.println(results.value, HEX);
    Serial.println();
    irrecv.resume();  // Receive the next value
  }
  if (Serial.available() > 0) {     // Serial Data Format {"Protocol":"GREE","Model":"YB1FA","Power":1,"Mode":"Cool","Temperature":23,"Celsius":1,"Fan":"max","SwingV":"highest","SwingH":"Auto","Quiet":false,"Turbo":false,"Eco":false,"Light":true,"Filter":true,"Clean":false,"Beep":true,"Sleep":-1,"Clock":-1}
    char data = Serial.read();
    serin += data;
    if (data == '\n') {
      DynamicJsonDocument jdata(512);
      DeserializationError error = deserializeJson(jdata, serin);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      ACParam.protocol = protocol(jdata["Protocol"]);
      ACParam.model = irac.strToModel(jdata["Model"]);
      ACParam.power = jdata["Power"];
      ACParam.mode = irac.strToOpmode(jdata["Mode"]);
      ACParam.degrees = jdata["Temperature"];
      ACParam.celsius = jdata["Celsius"];
      ACParam.fanspeed = irac.strToFanspeed(jdata["Fan"]);
      ACParam.swingv = irac.strToSwingV(jdata["SwingV"]);
      ACParam.swingh = irac.strToSwingH(jdata["SwingH"]);
      ACParam.quiet = jdata["Quiet"];
      ACParam.turbo = jdata["Turbo"];
      ACParam.econo = jdata["Eco"];
      ACParam.light = jdata["Light"];
      ACParam.filter = jdata["Filter"];
      ACParam.clean = jdata["Clean"];
      ACParam.beep = jdata["Beep"];
      ACParam.sleep = jdata["Sleep"];
      ACParam.clock = jdata["Clock"];
      irac.sendAc(ACParam);
      serin="";
      jdata["Protocol"] = ACParam.protocol;
      jdata["Model"] = ACParam.model;
      jdata["Power"] = ACParam.power; 
      jdata["Mode"] = irac.opmodeToString(ACParam.mode); 
      jdata["Temperature"] = ACParam.degrees; 
      jdata["Celsius"] = ACParam.celsius; 
      jdata["Fan"] = irac.fanspeedToString(ACParam.fanspeed);
      jdata["SwingV"] = irac.swingvToString(ACParam.swingv);
      jdata["SwingH"] = irac.swinghToString(ACParam.swingh);
      jdata["Quiet"] = ACParam.quiet;
      jdata["Turbo"] = ACParam.turbo; 
      jdata["Eco"] = ACParam.econo;
      jdata["Light"] = ACParam.light;
      jdata["Filter"] = ACParam.filter; 
      jdata["Clean"] = ACParam.clean; 
      jdata["Beep"] = ACParam.beep; 
      jdata["Sleep"] = ACParam.sleep; 
      jdata["Clock"] = ACParam.clock;
      Serial.println();
      Serial.println(F("Setting AC with Parameter : "));
      serializeJsonPretty(jdata, Serial);
      PublishACState(pubacmodeskip);
    }
  }
}