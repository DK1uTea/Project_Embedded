#define ESP_DRD_USE_SPIFFS true

// ----------------------------
// Standard Libraries - Already Installed if you have ESP32 set up
// ----------------------------

#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>

// ----------------------------
// Additional Libraries - each one of these will need to be installed.
// ----------------------------

#include <WiFiManager.h>
// Captive portal for configuring the WiFi

// Can be installed from the library manager (Search for "WifiManager", install the Alhpa version)
// https://github.com/tzapu/WiFiManager

#include <ESP_DoubleResetDetector.h>
// A library for checking if the reset button has been pressed twice
// Can be used to enable config mode
// Can be installed from the library manager (Search for "ESP_DoubleResetDetector")
//https://github.com/khoih-prog/ESP_DoubleResetDetector

#include <ArduinoJson.h>
// ArduinoJson is used for parsing and creating the config file.
// Search for "Arduino Json" in the Arduino Library manager
// https://github.com/bblanchon/ArduinoJson

// -------------------------------------
// -------   Other Config   ------
// -------------------------------------

#include <Wire.h>
// #include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#define BUZZER_PIN 14
#define BUZZER_CHANNEL 0
#define BEEP_DURATION 5000 // 1 second (in milliseconds)
#include <HTTPClient.h>
#include "time.h"

const int PIN_LED = 2;

#define JSON_CONFIG_FILE "/sample_config.json"

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

// -----------------------------

// -----------------------------

DoubleResetDetector *drd;

//flag for saving data
bool shouldSaveConfig = false;

char testString[50] = "deafult value";
unsigned long long testNumber = 12345678123ULL;
char testNumberStr[20]; // Allocate a char array to hold the converted number as a string
int apikey = 1234567;

unsigned long button_time = 0;  
unsigned long last_button_time = 0;
const float fallThreshold = 650000; // Adjust this value to suit your needs (in m/s^3)
const int sampleInterval = 10;   // Interval in milliseconds between readings
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;
String serverName = "https://api.callmebot.com/whatsapp.php?";
const char* ntpServer = "in.pool.ntp.org";
const long  gmtOffset_sec = 106200;
const int   daylightOffset_sec = 0;
char dateTimeStr[30];
bool buttonInterrupted = false;
unsigned long delayStartTime = 0;
const int DOUBLE_PRESS_THRESHOLD = 1500; // Time threshold for double press in milliseconds
bool lastFallDetection = false;
unsigned long lastFallTime = 0;
IPAddress staticIP(192, 168, 1, 100); // Replace with the desired static IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8); // Replace with your DNS server IP address


struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {12, 0, false};

// MPU6050 mpu;
Adafruit_MPU6050 mpu;
TwoWire myWire = TwoWire(0);

String urlEncode(const char* str) {
  const char* hex = "0123456789ABCDEF";
  String encodedStr = "";

  while (*str != 0) {
    if (('a' <= *str && *str <= 'z')
        || ('A' <= *str && *str <= 'Z')
        || ('0' <= *str && *str <= '9')) {
      encodedStr += *str;
    } else {
      encodedStr += '%';
      encodedStr += hex[*str >> 4];
      encodedStr += hex[*str & 0xf];
    }
    str++;
  }

  return encodedStr;
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  
  // Print only the current day and time
  
  strftime(dateTimeStr, sizeof(dateTimeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
}


void IRAM_ATTR isr() {
  button_time = millis();
  if (button_time - last_button_time > 250){
    button1.numberKeyPresses++;
    button1.pressed = true;
    last_button_time = button_time;
    buttonInterrupted = true; // Set the flag to indicate button press
  }
}

void saveConfigFile()
{
  Serial.println(F("Saving config"));
  StaticJsonDocument<512> json;
  json["testString"] = testString;
  json["testNumber"] = testNumber;
  json["apikey"] = apikey;

  File configFile = SPIFFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }

  serializeJsonPretty(json, Serial);
  if (serializeJson(json, configFile) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }
  configFile.close();
}

bool loadConfigFile()
{
  //clean FS, for testing
  // SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  // May need to make it begin(true) first time you are using SPIFFS
  // NOTE: This might not be a good way to do this! begin(true) reformats the spiffs
  // it will only get called if it fails to mount, which probably means it needs to be
  // formatted, but maybe dont use this if you have something important saved on spiffs
  // that can't be replaced.
  if (SPIFFS.begin(false) || SPIFFS.begin(true))
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists(JSON_CONFIG_FILE))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
      if (configFile)
      {
        Serial.println("opened config file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, Serial);
        if (!error)
        {
          Serial.println("\nparsed json");

          strcpy(testString, json["testString"]);
          testNumber = json["testNumber"].as<unsigned long long>(); // Correctly read the testNumber
          apikey = json["apikey"].as<int>(); // Correctly read the apikey

          return true;
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
  //end read
  return false;
}





//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// This gets called when the config mode is launced, might
// be useful to update a display with this info.
void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered Conf Mode");

  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());

  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}

bool timeReady = false;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Adafruit MPU6050 test!");

  // Dùng GPIO2 (SDA), GPIO15 (SCL) nếu bạn chắc chắn các chân này ổn định
  myWire.begin(2, 15);

  // Gọi hàm begin với địa chỉ + bus I2C tuỳ chỉnh
  if (!mpu.begin(0x68, &myWire)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Verify the connection
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);
  ledcSetup(BUZZER_CHANNEL, 2000, 16); // 2000 Hz, 8-bit resolution
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  pinMode(PIN_LED, OUTPUT);

  bool forceConfig = false;

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (drd->detectDoubleReset())
  {
    Serial.println(F("Forcing config mode as there was a Double reset detected"));
    forceConfig = true;
  }

  bool spiffsSetup = loadConfigFile();
  if (!spiffsSetup) {
    Serial.println(F("Forcing config mode as there is no saved config"));
    forceConfig = true;
  }

  //WiFi.disconnect();
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP


  // wm.resetSettings(); // wipe settings

  const char *configInfoText = "<div style='margin-bottom: 20px;'>Ask your emergency contact to open scan this QR <a href='https://i.ibb.co/rMzYN3z/callmebot-qr.png'>here</a> and provide their API key.</div>";

  // Text to be displayed below the parameters
  const char *configInfoTextBottom = "<div style='margin-top: 20px;'>More configuration options can be added here...</div>";

  WiFiManager wm;
  wm.setSaveConfigCallback(saveConfigCallback);
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wm.setAPCallback(configModeCallback);

  // Add the text above the parameters
  WiFiManagerParameter config_info_top(configInfoText);
  wm.addParameter(&config_info_top);

  // --- additional Configs params ---

  // Text box (String)
  WiFiManagerParameter custom_text_box("key_text", "Enter your Name", testString, 50); // 50 == max length

  // Text box (Number)
  sprintf(testNumberStr, "%llu", testNumber); // Convert the testNumber to a string
  WiFiManagerParameter custom_text_box_num("key_num", "Enter Emergency Contact's number", testNumberStr, 12); // 12 == max length

  // Text box (API Key)
  char convertedValueApi[7];
  sprintf(convertedValueApi, "%d", apikey); // Need to convert to a string to display a default value.
  WiFiManagerParameter custom_text_box_api("key_api", "Enter Emergency contact's API key", convertedValueApi, 7); // 7 == max length


  // Add all your parameters here
  wm.addParameter(&custom_text_box);
  wm.addParameter(&custom_text_box_num);
  wm.addParameter(&custom_text_box_api);

  // Add the text below the parameters
  WiFiManagerParameter config_info_bottom(configInfoTextBottom);
  wm.addParameter(&config_info_bottom);
  Serial.println("hello");

  digitalWrite(PIN_LED, LOW);
  if (forceConfig)
  {
    if (!wm.startConfigPortal("Fall_detector", "clock123"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  else
  {
    if (!wm.autoConnect("Fall_detector", "clock123"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      // if we still have not connected restart and try all over again
      ESP.restart();
      delay(5000);
    }
  }

  // If we get here, we are connected to the WiFi
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    WiFi.config(staticIP, gateway, subnet, dns);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Chờ đồng bộ NTP
    struct tm timeinfo;
    int retry = 0;
    const int max_retries = 20;
    while (!getLocalTime(&timeinfo) && retry < max_retries) {
      Serial.println("Waiting for NTP time sync...");
      delay(500);
      retry++;
    }

    if (retry >= max_retries) {
      Serial.println("Failed to sync time with NTP server.");
    } else {
      Serial.println("Time synced successfully.");
      printLocalTime();  // in ra thời gian
    }

    digitalWrite(PIN_LED, HIGH);
  } else {
    Serial.println("WiFi not connected. Restarting...");
    ESP.restart();
  }


  digitalWrite(PIN_LED, HIGH);

  // Copy the string value
  strncpy(testString, custom_text_box.getValue(), sizeof(testString));
  Serial.print("testString: ");
  Serial.println(testString);

  //Convert the number value
  testNumber = strtoull(custom_text_box_num.getValue(), nullptr, 10);
  sprintf(testNumberStr, "%llu", strtoull(custom_text_box_num.getValue(), nullptr, 10));
  Serial.print("testNumber: ");
  Serial.println(testNumberStr);

  apikey = atoi(custom_text_box_api.getValue());
  Serial.print("apikey: ");
  Serial.println(apikey);

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveConfigFile();
  }
}

// ... Rest of the code ...

void loop() {
  drd->loop();

  if (timeReady) {
    printLocalTime();
  }

  static unsigned long lastTime = 0;
  static bool firstSample = true;
  HTTPClient http;

  if (millis() - lastTime >= sampleInterval) {
    lastTime = millis();

    // Lấy dữ liệu từ cảm biến
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    Serial.printf("ax: %.2f, ay: %.2f, az: %.2f (m/s^2)\n", ax, ay, az);

    float acceleration_g_x = ax / 9.8;
    float acceleration_g_y = ay / 9.8;
    float acceleration_g_z = az / 9.8;

    if (firstSample) {
      prevAccX = acceleration_g_x;
      prevAccY = acceleration_g_y;
      prevAccZ = acceleration_g_z;
      firstSample = false;
      return;
    }

    float jerkX = (acceleration_g_x - prevAccX) / (sampleInterval / 1000.0);
    float jerkY = (acceleration_g_y - prevAccY) / (sampleInterval / 1000.0);
    float jerkZ = (acceleration_g_z - prevAccZ) / (sampleInterval / 1000.0);

    Serial.printf("Jerk X: %.2f, Y: %.2f, Z: %.2f\n", jerkX, jerkY, jerkZ);

    prevAccX = acceleration_g_x;
    prevAccY = acceleration_g_y;
    prevAccZ = acceleration_g_z;

    float jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);
    Serial.printf("jerkMagnitude value: %.2f\n", jerkMagnitude);

    if (jerkMagnitude > fallThreshold && (millis() - lastFallTime > 10000)) {
      Serial.println("Fall detected!");
      ledcWriteTone(BUZZER_CHANNEL, 5000);

      buttonInterrupted = false;
      delayStartTime = millis();

      while (delayStartTime > 0 && millis() - delayStartTime < BEEP_DURATION) {
        if (button1.pressed) {
          button1.pressed = false;
          ledcWrite(BUZZER_CHANNEL, 0);
          Serial.println("Buzzer stopped by button press.");
          break;
        }
      }

      if (!buttonInterrupted) {
        char testNumberStr[20];
        sprintf(testNumberStr, "%llu", testNumber);
        String encodedDateTime = urlEncode(dateTimeStr);
        String serverPath = serverName + "phone=" + testNumberStr + "&apikey=" + String(apikey) + "&text=A+Fall+has+been+detected+for+user:+" + testString + "+on+" + encodedDateTime;

        http.begin(serverPath.c_str());
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0) {
          Serial.println("Alert message sent successfully!");
          Serial.printf("HTTP Response code: %d\n", httpResponseCode);
          Serial.println(http.getString());
          lastFallDetection = true;
          lastFallTime = millis();
        } else {
          Serial.printf("Error code: %d\n", httpResponseCode);
        }

        http.end();
      }

      delayStartTime = 0;
    }

    if (button1.pressed) {
      if (delayStartTime > 0) delayStartTime = 0;
      Serial.printf("Stop Button was pressed %u times\n", button1.numberKeyPresses);
      button1.pressed = false;
      ledcWrite(BUZZER_CHANNEL, 0);
    }
  }

  if (button1.numberKeyPresses >= 2 && (millis() - button_time) <= DOUBLE_PRESS_THRESHOLD) {
    Serial.println("Double press detected!");
    if (lastFallDetection) {
      char testNumberStr[20];
      sprintf(testNumberStr, "%llu", testNumber);
      String safeMessage = "Last+fall+detection+was+false%2C+user:+%22" + String(testString) + "%22+is+safe";
      String serverPath = serverName + "phone=" + testNumberStr + "&apikey=" + String(apikey) + "&text=" + safeMessage;

      http.begin(serverPath.c_str());
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        Serial.println("Safe message sent successfully!");
        Serial.printf("HTTP Response code: %d\n", httpResponseCode);
        Serial.println(http.getString());
      } else {
        Serial.printf("Error code: %d\n", httpResponseCode);
      }

      http.end();
      lastFallDetection = false;
    } else {
      Serial.println("No previous fall detection to send safe message.");
    }

    button1.numberKeyPresses = 0;
  }
}
