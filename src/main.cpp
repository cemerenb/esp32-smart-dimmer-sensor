/*
  Rui Santos
  Complete project details at our blog: https://RandomNerdTutorials.com/esp32-esp8266-firebase-authentication/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based in the Authenticatiions Examples by Firebase-ESP-Client library by mobizt: https://github.com/mobizt/Firebase-ESP-Client/tree/main/examples/Authentications
*/

#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <FirebaseHelper.h>
#include <FastLED.h>
#include <Controls.h>
#include <BulbData.h>
#include <EEPROM.h>
#include <BluetoothHelper.h>
#include <ezButton.h>
#include <DHT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "time.h"
#include <stdio.h>

// Insert your network credentials
// #define WIFI_SSID "TTNET_ZyXEL_3JN3"
// #define WIFI_SSID "Emir's iPhone 14"
// #define WIFI_SSID "Asaf's Galaxy S21 5G"
// #define WIFI_PASSWORD "ataturk_1881"
// #define WIFI_PASSWORD "test12345"
// #define WIFI_PASSWORD "fnsc2964"
#define DHTPIN 27 // DHT11 sensörünün bağlı olduğu pin
#define encoderPinA 25
#define encoderPinB 26
#define BUTTON_PIN 15
#define BAUD_RATE 115200
ezButton button(BUTTON_PIN); // create ezButton object that attach to pin 7;
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
WiFiUDP ntpUDP;
const char *ntpServer = "tr.pool.ntp.org";
// variables will change:
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
unsigned long previousMillis = 0;
const long interval = 180000; // 3 dakika
int led_state = LOW;
volatile int encoderValue = 0;
volatile int lastEncoded = 0;
volatile long lastMillis = 0;
volatile bool buttonState = false;
volatile bool lastButtonState = true;
unsigned long debounceDelay = 50;
bool ledEnabled = false;
FirebaseJson dataJson;
BulbData bulbData;
String ssid;
String password;
bool isCompleted = false;
int localBrightness = 128;
unsigned long sendDataPrevMillis = 0;
int count = 10;

void sendTemperatureData(float heat, float humidity,String dateTime)
{
  Firebase.RTDB.setInt(&fbdo, getSensorPath("heat", homegroupId), heat);
  Firebase.RTDB.setInt(&fbdo, getSensorPath("humidity", homegroupId), humidity);
  Firebase.RTDB.setString(&fbdo, getSensorPath("dateTime", homegroupId), dateTime);
  Serial.println("Data sent to database");
  Serial.print("Sıcaklık: ");
  Serial.print(heat);
  Serial.print(" °C, Nem: ");
  Serial.print(humidity);
  Serial.println(" %");
}

void readDHT11()
{
  struct tm timeinfo;
  char buffer[80];
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }

  strftime(buffer, 80, "%DT%T", &timeinfo);
  Serial.println(buffer);
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  sendTemperatureData(t, h,buffer);
}

void handleEncoder()
{
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
  {
    encoderValue++;
  }
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    encoderValue--;
  }

  lastEncoded = encoded;
}
// Initialize WiFi
void initWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
}

void readDataFromEEPROM()
{
  ssid.reserve(SSID_MAX_LENGTH);
  password.reserve(PASSWORD_MAX_LENGTH);
  homegroupId.reserve(SSID_MAX_LENGTH);
  Serial.println("String for data reserved");
  ssid = EEPROM.readString(0);
  password = EEPROM.readString(SSID_MAX_LENGTH);
  homegroupId = EEPROM.readString(SSID_MAX_LENGTH * 2);
  Serial.print("ssid:");
  Serial.println(ssid);
  Serial.print("password:");
  Serial.println(password);
  Serial.print("homegroupId:");
  Serial.println(homegroupId);
}

bool isEEPROMDataValid()
{
  return (!ssid.isEmpty() && !password.isEmpty() && !homegroupId.isEmpty());
}

void completeSetup()
{
  setupFirebase();

  // Getting the user UID might take a few seconds
  waitForUserUid();

  // Start HTTP stream

  // streamCallback runs each time when stream receives data
  // streamTimeoutCallback runs when connection was timeout during HTTP stream
}

void setup()
{
  dht.begin();
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  button.setDebounceTime(50);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncoder, CHANGE);

  Serial.begin(BAUD_RATE);

  EEPROM.begin(512);
  readDataFromEEPROM();
  if (!isEEPROMDataValid())
  {
    Serial.println("EEPROM data not valid!");
    // Bluetooth helper will call setupCallback when connection is success
    setupCallback = &completeSetup;
    startBLE();
    isCompleted = true;
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    setenv("TZ", "<+03>-3", 1);
  }
  else
  {
    initWiFi();
    completeSetup();
    setupCallback = &completeSetup;
    startBLE();
    Firebase.RTDB.getInt(&fbdo, resourcePath("color/brightness", homegroupId));
    localBrightness = fbdo.to<int>();
    Serial.print("İlk değer");
    Serial.println(localBrightness);
    isCompleted = true;
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    setenv("TZ", "<+03>-3", 1);
  }
}
void updateFirebaseWithLocalData()
{

  String pathToDevice = resourcePath("", homegroupId);
  Firebase.RTDB.setInt(&fbdo, resourcePath("color/brightness", homegroupId), localBrightness);
}
void loop()
{
  button.loop();
  if (button.isReleased())
  {
    Firebase.RTDB.getBool(&fbdo, resourcePath("color/enabled", homegroupId));
    bool databaseState = fbdo.to<bool>();
    Serial.print("Bulb state from database: ");
    Serial.println(databaseState);

    // Toggle the state
    buttonState = !databaseState;

    // Set the boolean value in RTDB based on the button state
    Firebase.RTDB.setBool(&fbdo, resourcePath("color/enabled", homegroupId), buttonState);

    Serial.print("Button State: ");
    Serial.println(buttonState);
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {

    // Get time and date information
    // Zaman geldiğinde DHT11 sensöründen veri oku
    readDHT11();

    // Zamanı güncelle
    previousMillis = currentMillis;
  }

  if (millis() - lastMillis > 200)
  {
    int temp = localBrightness;
    // Add your brightness adjustment or other actions here
    localBrightness += encoderValue;

    if (localBrightness != temp)
    {

      Firebase.RTDB.getInt(&fbdo, resourcePath("color/brightness", homegroupId));
      int comingData = fbdo.to<int>();
      Serial.print("Data from database");
      Serial.println(comingData);

      localBrightness = comingData + encoderValue;
      if (localBrightness > 255)
      {
        localBrightness = 255;
      }
      if (localBrightness < 0)
      {
        localBrightness = 0;
      }

      updateFirebaseWithLocalData();
      Serial.print("Brightness -->  ");
      Serial.println(localBrightness);
    }

    // Reset encoder value after processing
    encoderValue = 0;

    lastMillis = millis();
  }
}