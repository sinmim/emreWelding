#include <Arduino.h>
#include "TriacController.h"

#define ZC_INPUT_PIN 14
#define TRIAC_OUTPUT_PIN 48

TriacController controller;

void powerChangingTask(void *pvParameters)
{
  float power = 0.0;
  float step = 1.0;

  while (1)
  {
    // power += step;
    // if (power >= 50.0) {
    //     power = 50.0;
    //     step = -1.0;
    // } else if (power <= 0.0) {
    //     power = 0.0;
    //     step = 1.0;
    // }
    // controller.setPower(power);
    controller.setPower(30);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Predictive TRIAC Controller Test");

  uint8_t myFilterSize = 7;

  if (!controller.begin(ZC_INPUT_PIN, TRIAC_OUTPUT_PIN, 45.0, 65.0, myFilterSize))
  {
    Serial.println("Failed to initialize Triac Controller!");
    while (1)
      ;
  }

  // Set the known hardware delay of your zero-cross detector
  controller.setMeasurementDelay(3000);
  // For debugging the predictive timer:
  // controller._freqMonitor.setDebug(true);

  // 3. Set the low-pass filter strength (smooths jitter)
  // A smaller alpha means more smoothing. 1.0 means OFF.
  // Good values to test are between 0.05 and 0.5
  float lpfAlpha = 0.99;
  controller.setLowPassFilterAlpha(lpfAlpha);


  controller.setPower(0);
  controller.enableOutput();

  xTaskCreate(powerChangingTask, "powerTask", 2048, NULL, 5, NULL);
}

void loop()
{
  controller.update();

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
// hello git
