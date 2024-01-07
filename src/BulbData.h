#include <Arduino.h>
#include <FastLED.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <FastLED.h>

struct RgbColor
{
    int red;
    int green;
    int blue;

    int toDecimal()
    {
        return (blue << 16) + (green << 8) + red;
    }

    CRGB toCRGB()
    {
        return CRGB(green, red, blue);
    }

    String prettyString()
    {
        String result = "RgbColor(r: ";
        result.concat(red);
        result.concat(", g: ");
        result.concat(green);
        result.concat(", b: ");
        result.concat(blue);
        result.concat(")");
        return result;
    }

    RgbColor(int colorDecimal)
    {
        red = (colorDecimal >> 16) & 0xFF;
        green = (colorDecimal >> 8) & 0xFF;
        blue = colorDecimal & 0xFF;
    }

    RgbColor()
    {
        red = 255;
        green = 255;
        blue = 255;
    }
};
struct AnimationData
{
    int animationIndex = 0;
    uint8_t brightness = 255;
    uint8_t speed = 255;
    bool enabled = true;
    bool containsCorrectedValue = true;

    String prettyString()
    {
        String result = "AnimationData(animationIndex: ";
        result.concat(animationIndex);
        result.concat(", brightness: ");
        result.concat(brightness);
        result.concat(", enabled: ");
        result.concat(enabled ? "true" : "false");
        result.concat(", speed: ");
        result.concat(speed);
        result.concat(", containsCorrectValue: ");
        result.concat(containsCorrectedValue ? "true" : "false");
        result.concat(")");
        return result;
    }

    AnimationData() {}

    AnimationData(FirebaseJson *json)
    {
        containsCorrectedValue = false;

        FirebaseJsonData animationIndexResult;
        bool readSuccessful = json->isMember("index") && json->get(animationIndexResult, "index") && animationIndexResult.typeNum == FirebaseJson::JSON_INT;
        if (readSuccessful)
        {
            int animationIndex = animationIndexResult.to<int>();
        }
        else
        {
            containsCorrectedValue = true;
        }

        FirebaseJsonData brightnessResult;
        readSuccessful = json->isMember("brightness") && json->get(brightnessResult, "brightness") && brightnessResult.typeNum == FirebaseJson::JSON_INT;
        if (readSuccessful)
        {
            brightness = brightnessResult.to<uint8_t>();
        }
        else
        {
            containsCorrectedValue = true;
        }

        FirebaseJsonData speedResult;
        readSuccessful = json->isMember("speed") && json->get(speedResult, "speed") && speedResult.typeNum == FirebaseJson::JSON_INT;
        if (readSuccessful)
        {
            speed = speedResult.to<uint8_t>();
        }
        else
        {
            containsCorrectedValue = true;
        }

        FirebaseJsonData enabledResult;
        readSuccessful = json->isMember("enabled") && json->get(enabledResult, "enabled") && enabledResult.typeNum == FirebaseJson::JSON_BOOL;
        if (readSuccessful)
        {
            enabled = enabledResult.to<bool>();
        }
        else
        {
            containsCorrectedValue = true;
        }
    }

    void processPartial(FirebaseStream data)
    {
        containsCorrectedValue = false;

        String path = data.dataPath();
        if (path == "/animation/index")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_integer)
            {
                animationIndex = data.intData();
            }
            else
            {
                containsCorrectedValue = true;
            }
        }
        else if (path == "/animation/brightness")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_integer)
            {
                brightness = data.intData();
            }
            else
            {
                containsCorrectedValue = true;
            }
        }
        else if (path == "/animation/speed")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_integer)
            {
                speed = data.intData();
            }
            else
            {
                containsCorrectedValue = true;
            }
        }
        else if (path == "/animation/enabled")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_boolean)
            {
                enabled = data.boolData();
            }
            else
            {
                containsCorrectedValue = true;
            }
        }
        else if (path == "/animation")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_json)
            {
                bool readSuccessful = false;
                FirebaseJson *json = &data.jsonObject();
                json->toString(Serial, true);

                if (json->isMember("index"))
                {
                    FirebaseJsonData animationIndexResult;
                    bool readSuccessful = json->isMember("index") && json->get(animationIndexResult, "index") && animationIndexResult.typeNum == FirebaseJson::JSON_INT;
                    if (readSuccessful)
                    {
                        int animationIndex = animationIndexResult.to<int>();
                    }
                    else
                    {
                        containsCorrectedValue = true;
                    }
                }

                if (json->isMember("brightness"))
                {
                    FirebaseJsonData brightnessResult;
                    readSuccessful = json->isMember("brightness") && json->get(brightnessResult, "brightness") && brightnessResult.typeNum == FirebaseJson::JSON_INT;
                    if (readSuccessful)
                    {
                        brightness = brightnessResult.to<uint8_t>();
                    }
                    else
                    {
                        containsCorrectedValue = true;
                    }
                }

                if (json->isMember("speed"))
                {
                    FirebaseJsonData speedResult;
                    readSuccessful = json->isMember("speed") && json->get(speedResult, "speed") && speedResult.typeNum == FirebaseJson::JSON_INT;
                    if (readSuccessful)
                    {
                        speed = speedResult.to<uint8_t>();
                    }
                    else
                    {
                        containsCorrectedValue = true;
                    }
                }

                if (json->isMember("enabled"))
                {
                    FirebaseJsonData enabledResult;
                    readSuccessful = json->isMember("enabled") && json->get(enabledResult, "enabled") && enabledResult.typeNum == FirebaseJson::JSON_BOOL;
                    if (readSuccessful)
                    {
                        enabled = enabledResult.to<bool>();
                    }
                    else
                    {
                        containsCorrectedValue = true;
                    }
                }
            }
            else
            {
                Serial.println("Got color object, but not json.");
                containsCorrectedValue = true;
            }
        }
    }
};

struct ColorData
{
    RgbColor color = RgbColor();
    uint8_t brightness = 255;
    bool enabled = true;
    bool containsCorrectedValue = true;

    String prettyString()
    {
        String result = "ColorData(color: ";
        result.concat(color.prettyString());
        result.concat(", brightness: ");
        result.concat(brightness);
        result.concat(", enabled: ");
        result.concat(enabled ? "true" : "false");
        result.concat(", containsCorrectValue: ");
        result.concat(containsCorrectedValue ? "true" : "false");
        result.concat(")");
        return result;
    }

    ColorData() {}

    ColorData(FirebaseJson *json)
    {
        containsCorrectedValue = false;
        FirebaseJsonData colorResult;
        bool readSuccessful = json->isMember("colorCode") && json->get(colorResult, "colorCode") && colorResult.typeNum == FirebaseJson::JSON_INT;
        if (readSuccessful)
        {
            int colorDecimal = colorResult.to<int>();
            color = RgbColor(colorDecimal);
        }
        else
        {
            containsCorrectedValue = true;
        }

        FirebaseJsonData brightnessResult;
        readSuccessful = json->isMember("brightness") && json->get(brightnessResult, "brightness") && brightnessResult.typeNum == FirebaseJson::JSON_INT;
        if (readSuccessful)
        {
            brightness = brightnessResult.to<uint8_t>();
        }
        else
        {
            Serial.print("couldn't read color brightness ");
            containsCorrectedValue = true;
        }

        FirebaseJsonData enabledResult;
        readSuccessful = json->isMember("enabled") && json->get(enabledResult, "enabled") && enabledResult.typeNum == FirebaseJson::JSON_BOOL;
        if (readSuccessful)
        {
            enabled = enabledResult.to<bool>();
        }
        else
        {
            containsCorrectedValue = true;
        }
    }

    void processPartial(FirebaseStream data)
    {
        containsCorrectedValue = false;

        String path = data.dataPath();
        if (path == "/color/colorCode")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_integer)
            {
                int colorDecimal = data.intData();
                color = RgbColor(colorDecimal);
            }
            else
            {
                containsCorrectedValue = true;
            }
        }
        else if (path == "/color/brightness")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_integer)
            {
                brightness = data.intData();
            }
            else
            {
                containsCorrectedValue = true;
            }
        }
        else if (path == "/color/enabled")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_boolean)
            {
                enabled = data.boolData();
            }
            else
            {
                containsCorrectedValue = true;
            }
        }
        else if (path == "/color")
        {
            if (data.dataTypeEnum() == firebase_rtdb_data_type_json)
            {
                bool readSuccessful = false;
                FirebaseJson *json = &data.jsonObject();

                if (json->isMember("colorCode"))
                {
                    FirebaseJsonData colorResult;
                    readSuccessful = json->isMember("colorCode") && json->get(colorResult, "colorCode");
                    if (readSuccessful)
                    {
                        int colorDecimal = colorResult.to<int>();
                        color = RgbColor(colorDecimal);
                    }
                    else
                    {
                        containsCorrectedValue = true;
                    }
                }

                if (json->isMember("brightness"))
                {

                    FirebaseJsonData brightnessResult;
                    readSuccessful = json->isMember("brightness") && json->get(brightnessResult, "brightness");
                    if (readSuccessful)
                    {
                        brightness = brightnessResult.to<uint8_t>();
                    }
                    else
                    {
                        Serial.print("couldn't read color brightness ");
                        containsCorrectedValue = true;
                    }
                }

                if (json->isMember("enabled"))
                {
                    FirebaseJsonData enabledResult;
                    readSuccessful = json->isMember("enabled") && json->get(enabledResult, "enabled");
                    if (readSuccessful)
                    {
                        enabled = enabledResult.to<bool>();
                    }
                    else
                    {
                        containsCorrectedValue = true;
                    }
                }
            }
            else
            {
                Serial.println("Got color object, but not json.");
                containsCorrectedValue = true;
            }
        }
    }
};

struct BulbData
{
    AnimationData animationData;
    ColorData colorData;

    bool animationNull() { return (&animationData == nullptr); };
    bool colorNull() { return (&colorData == nullptr); };

    String prettyString()
    {
        bool animationDeclared = (&animationData != nullptr);
        bool colorDeclared = (&colorData != nullptr);

        String result = "Bulb(containsCorrectValue: ";
        result.concat(containsCorrectedValue() ? "true" : "false");

        if (animationDeclared)
        {
            result.concat(", animationData: ");
            result.concat(animationData.prettyString());
        }

        if (colorDeclared)
        {
            result.concat(", colorData: ");
            result.concat(colorData.prettyString());
        }

        result.concat(")");
        return result;
    }

    BulbData() {}

    BulbData(FirebaseJson *json)
    {
        bool animationDataExists = json->isMember("animation");
        bool colorDataExists = json->isMember("color");
        if (colorDataExists)
        {
            FirebaseJsonData result;
            bool readResult = json->get(result, "color") && result.typeNum == FirebaseJson::JSON_OBJECT;
            if (readResult)
            {

                FirebaseJson colorJson;
                result.get<FirebaseJson>(colorJson);
                colorData = ColorData(&colorJson);
            }
        }

        if (animationDataExists)
        {
            FirebaseJsonData result;
            bool readResult = json->get(result, "animation") && result.typeNum == FirebaseJson::JSON_OBJECT;
            if (readResult)
            {

                FirebaseJson animationJson;
                result.get<FirebaseJson>(animationJson);
                animationData = AnimationData(&animationJson);
            }
        }

        Serial.print("Animation is ");
        Serial.println(animationNull() ? "null" : "not null.");
        Serial.print("Color is ");
        Serial.println(colorNull() ? "null" : "not null.");

        bool bothDeclared = !animationNull() && !colorNull();
        bool bothEnabled = bothDeclared && animationData.enabled && colorData.enabled;
        if (bothEnabled)
        {
            Serial.println("Both animation and color are enabled, choosing color");
            // Color has higher priority over animation
            animationData.enabled = false;
            animationData.containsCorrectedValue = true;
        }
        else if (animationNull() && colorNull())
        {
            Serial.println("Both animation and color are null");
            colorData = ColorData();
            colorData.containsCorrectedValue = true;
        }
    }

    FirebaseJson *toFirebaseJson()
    {
        FirebaseJson *json = new FirebaseJson();

        if (&colorData != nullptr)
        {
            // Convert ColorData to FirebaseJson
            FirebaseJson colorJson;
            colorJson.set("colorCode", colorData.color.toDecimal());
            colorJson.set("brightness", colorData.brightness);
            colorJson.set("enabled", colorData.enabled);

            json->set("color", colorJson);
        }

        if (&animationData != nullptr)
        {
            // Convert AnimationData to FirebaseJson
            FirebaseJson animationJson;
            animationJson.set("index", animationData.animationIndex);
            animationJson.set("brightness", animationData.brightness);
            animationJson.set("speed", animationData.speed);
            animationJson.set("enabled", animationData.enabled);

            json->set("animation", animationJson);
        }

        return json;
    }

    void clearCorrectedValue()
    {
        if (&colorData != nullptr)
        {
            colorData.containsCorrectedValue = false;
        }

        if (&animationData != nullptr)
        {
            animationData.containsCorrectedValue = false;
        }
    }

    bool containsCorrectedValue()
    {
        if (&colorData != nullptr)
        {
            return colorData.containsCorrectedValue;
        }

        else if (&animationData != nullptr)
        {
            return animationData.containsCorrectedValue;
        }
        else
        {
            return false;
        }
    }

    void processJson(FirebaseStream data)
    {
        clearCorrectedValue();

        if (!colorNull())
        {
            colorData.processPartial(data);
        }

        bool bothDeclared = !animationNull() && !colorNull();
        bool bothEnabled = bothDeclared && animationData.enabled && colorData.enabled;
        if (bothEnabled)
        {
            Serial.println("Both animation and color are enabled, choosing color");
            // Color has higher priority over animation
            animationData.enabled = false;
            animationData.containsCorrectedValue = true;
        }
        else if (animationNull() && colorNull())
        {
            Serial.println("Both animation and color are null");
            colorData = ColorData();
            colorData.containsCorrectedValue = true;
        }
    }
};