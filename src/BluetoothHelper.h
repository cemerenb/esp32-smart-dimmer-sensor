#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

String receivedData = "";
BLEServer *pServer;
BLECharacteristic *pCharacteristic;
const String hardcodedKey = "121212";
const int SSID_MAX_LENGTH = 32;
const int PASSWORD_MAX_LENGTH = 32;
String homegroupId;

void (*setupCallback)(void);

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        Serial.println("BLE Client Connected");
    }

    void onDisconnect(BLEServer *pServer)
    {
        Serial.println("BLE Client Disconnected");
        BLEAdvertising *pAdvertising = pServer->getAdvertising();
        pAdvertising->start();
    }
};

bool connectToWiFi(const String &ssid, const String &password)
{
    Serial.println("Connecting to Wi-Fi...");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);

    WiFi.begin(ssid.c_str(), password.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30)
    {
        delay(1000);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nConnected to Wi-Fi!");
        Serial.println("Writing to EEPROM");
        EEPROM.writeString(0, ssid);
        EEPROM.writeString(SSID_MAX_LENGTH, password);
        EEPROM.writeString(SSID_MAX_LENGTH * 2, homegroupId);
        EEPROM.commit();
        Serial.println("Write success");

        return true;
    }
    else
    {
        Serial.println("\nFailed to connect to Wi-Fi!");
        return false;
    }
}

void connectToWifiTask(void *parameter)
{
    std::pair<String, String> *credentials = (std::pair<String, String> *)parameter;
    String ssid = credentials->first;
    String password = credentials->second;

    int wifiConnectAttempts = 0;
    while (!connectToWiFi(ssid, password) && wifiConnectAttempts < 3)
    {
        wifiConnectAttempts++;
        delay(1000);
    }

    bool didConnect = wifiConnectAttempts < 3;
    if (didConnect)
    {
        Serial.println("WiFi connection success after receiving new credentials.");
        
        setupCallback();
        ESP.restart();
        
    }

    delete credentials;
    vTaskDelete(NULL);
}

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        String value = pCharacteristic->getValue().c_str();
        Serial.print("Received data: ");
        Serial.println(value);

        // Concatenate the received data
        receivedData += value;

        // Check if the received data contains "wifiinfo:"
        int wifiInfoIndex = receivedData.indexOf("wifiinfo:");
        if (wifiInfoIndex != -1)
        {
            // Find the position of the third colon separator
            int thirdColonIndex = receivedData.indexOf(':', wifiInfoIndex + 9);
            if (thirdColonIndex != -1)
            {
                // Extract SSID, password, key, and home group ID
                String ssid = receivedData.substring(wifiInfoIndex + 9, thirdColonIndex);
                String password = receivedData.substring(thirdColonIndex + 1, receivedData.indexOf(':', thirdColonIndex + 1));
                String receivedKey = receivedData.substring(receivedData.indexOf(':', thirdColonIndex + 1) + 1, receivedData.lastIndexOf(':'));
                homegroupId = receivedData.substring(receivedData.lastIndexOf(':') + 1);

                // Trim leading and trailing whitespaces
                ssid.trim();
                password.trim();
                receivedKey.trim();
                homegroupId.trim();
                Serial.println(homegroupId);
                Serial.print("Key: ");
                Serial.println(receivedKey);

                // Check if SSID is not empty
                if (!ssid.isEmpty() && !password.isEmpty() && !receivedKey.isEmpty())
                {
                    // Check if the received key matches the hardcoded key
                    if (receivedKey.equals(hardcodedKey))
                    {
                        // Create a new task to connect to Wi-Fi without blocking the BLE task
                        std::pair<String, String> *credentials = new std::pair<String, String>{ssid, password};
                        xTaskCreatePinnedToCore(
                            connectToWifiTask,
                            "ConnectToWifiTask",
                            8192, // Increased stack size
                            (void *)credentials,
                            1,
                            NULL,
                            0);
                    }
                    else
                    {
                        Serial.println("Invalid key. Ignoring.");
                        BLEAdvertising *pAdvertising = pServer->getAdvertising();
                        pAdvertising->start();
                    }
                }
                else
                {
                    Serial.println("Invalid SSID. Ignoring.");
                    BLEAdvertising *pAdvertising = pServer->getAdvertising();
                    pAdvertising->start();
                }
            }
            else
            {
                Serial.println("Third colon not found. Waiting for more data...");
                BLEAdvertising *pAdvertising = pServer->getAdvertising();
                pAdvertising->start();
            }

            // Clear receivedData for the next connection attempt
            receivedData = "";
        }
    }
};

void startBLE()
{
    Serial.println("Starting BLE server");
    BLEDevice::init("ESP32_BTServer");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(BLEUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b"));

    pCharacteristic = pService->createCharacteristic(
        BLEUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"),
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);

    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();

    Serial.println("Bluetooth Classic turned off");
    Serial.println("BLE Server Started");
}