// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBeixoVRmvZPxc03ACDr5AfMab4aJ0XLjI"
#define DATABASE_URL "https://iot-smart-home-71d33-default-rtdb.europe-west1.firebasedatabase.app/"
// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "esp32-4h926s9a@iotsmarthome.com"
#define USER_PASSWORD "esp32-4h926s9a"

// Firebase objexts
FirebaseConfig config;
FirebaseAuth auth;
FirebaseData fbdo;

String devicePath = "/esp32-4H926S9A/smartLamp/";
String sensorPath = "/esp32-4H926S9A/sensor/";

String uid;
// Creates path to given resource variable.
String getSensorPath(const String &variable_path, const String &homegroupId)
{
    String result = "homegroups/";
    result.concat(homegroupId);
    result.concat(sensorPath);
    result.concat(variable_path);
    return result;
}
String resourcePath(const String &variable_path, const String &homegroupId)
{
    String result = "homegroups/";
    result.concat(homegroupId);
    result.concat(devicePath);
    result.concat(variable_path);
    return result;
}

void setupFirebase()
{
    // Assign the api key (required)
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;

    // Assign the user sign in credentials
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    // Assign the callback function for the long running token generation task
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

    // Assign the maximum retry of token generation
    config.max_token_generation_retry = 5;

    // Initialize the library with the Firebase authen and config
    Firebase.begin(&config, &auth);
}

void waitForUserUid()
{
    Serial.println("Getting User UID");
    while ((auth.token.uid) == "")
    {
        Serial.print('.');
        delay(1000);
    }
    // Print user UID
    uid = auth.token.uid.c_str();
    Serial.print("User UID: ");
    Serial.println(uid);
}