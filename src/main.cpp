#include <Arduino.h>
#include "TriacController.h"

#define ZC_INPUT_PIN 14
#define TRIAC_OUTPUT_PIN 48

TriacController controller;

void powerChangingTask(void *pvParameters)
{
  float power = 0.0;
  float step = 1.0;
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  for (int i = 0; i < 400; i++)
  {
    float val = i / 10.0;
    controller.setPower(val);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  while (1)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  while (1)
  {
    // The user's original power-ramping code is preserved but commented out.
    power += step;
    if (power >= 20.0)
    {
      power = 20.0;
      step = -1.0;
    }
    else if (power <= 0.0)
    {
      power = 0.0;
      step = 1.0;
    }
    controller.setPower(power);
    // controller.setPower(30); // Set a fixed power for testing
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Reactive TRIAC Controller Test");

  uint8_t myFilterSize = 7;

  if (!controller.begin(ZC_INPUT_PIN, TRIAC_OUTPUT_PIN, 45.0, 65.0, myFilterSize))
  {
    Serial.println("Failed to initialize Triac Controller!");
    while (1)
      ;
  }

  // 1. Set the known hardware delay of your zero-cross detector
  // This is crucial for accurate timing.
  controller.setMeasurementDelay(3000);

  // 2. Set the low-pass filter strength (smooths jitter in frequency measurement)
  // A smaller alpha means more smoothing. 1.0 means OFF.
  // Good values to test are between 0.05 and 0.5. A high value makes it more reactive.
  float lpfAlpha = 0.99;
  controller.setLowPassFilterAlpha(lpfAlpha);

  // Enable the output
  controller.setPower(0);
  controller.enableOutput();

  xTaskCreate(powerChangingTask, "powerTask", 2048, NULL, 5, NULL);
}

void loop()
{
  // The controller.update() function is no longer needed.
  // All logic is now handled by hardware interrupts.

  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100)
  {
    lastPrintTime = millis();
    Serial.printf("Power: %.1f%%, Freq: %.2f Hz, Fault: %s\n",
                  controller.getCurrentPower(),
                  controller.getFrequency(),
                  controller.isFaulty() ? "YES" : "NO");
  }
}