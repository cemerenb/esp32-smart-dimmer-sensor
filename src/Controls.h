#include <FastLED.h>

#define NUM_LEDS 12
#define NUM_LEDS 12
#define DATA_PIN 26
CRGB leds[NUM_LEDS];

int normalizeBrightnessValue(double brightness)
{
    // Ensure the brightness value is within the valid range
    int normalizedValue = static_cast<int>(min(max(brightness, 0.0), 255.0));

    // Normalize the brightness value to the range [0, 1]
    double normalizedRatio = normalizedValue / 255.0;

    // Apply gamma correction
    const double gamma = 2.0;
    double correctedValue = pow(normalizedRatio, gamma);

    // Convert the corrected value back to the original range
    return static_cast<int>(round(correctedValue * 255));
}
