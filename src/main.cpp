#include <Arduino.h>
#include "TriacController.h"
#include <PID_v1.h> // Include the PID library
#include "sensor.h"
// Pin definitions
#define ZC_INPUT_PIN 14
#define TRIAC_OUTPUT_PIN 48
#define VOLTAGE_ADC_PIN 1

// --- PID Controller Setup ---
double Setpoint, Input, Output;

double Kp = 0.25;  
double Ki = 0.8; 
double Kd = 0; 

// Create a PID controller object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
TriacController controller;

double getCalibratedRMSVoltage()
{
  updateSensor();
  // Apply calibration factor
  return getVoltage() ;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("TRIAC PID Voltage Controller");

  initSensor();

  // Initialize TriacController
  uint8_t myFilterSize = 7;
  if (!controller.begin(ZC_INPUT_PIN, TRIAC_OUTPUT_PIN, 45.0, 65.0, myFilterSize))
  {
    Serial.println("Failed to initialize Triac Controller!");
    while (1)
      ; // Halt on failure
  }

  controller.setMeasurementDelay(3000);
  controller.setLowPassFilterAlpha(0.99);

  // --- Initialize PID Controller ---
  Setpoint = 0.0;                // Start with a target voltage of 0
  myPID.SetMode(AUTOMATIC);      // Turn the PID on
  myPID.SetSampleTime(50);       // Set PID compute interval to 50ms
  myPID.SetOutputLimits(0, 100); // Controller output is 0-100% power

  // Enable the TRIAC output
  controller.setPower(0);
  controller.enableOutput();

  Serial.println("Setup complete. Enter target voltage in Serial Monitor.");
}

void loop()
{
  // Check for new setpoint from Serial monitor
  if (Serial.available() > 0)
  {
    String inputString = Serial.readStringUntil('\n');
    float newSetpoint = inputString.toFloat();
    if (newSetpoint >= 0)
    {
      Setpoint = newSetpoint;
      Serial.print("New Setpoint received: ");
      Serial.println(Setpoint);
    }
  }

  // Update PID control loop
  Input = getCalibratedRMSVoltage(); // Read current voltage
  myPID.Compute();                   // Calculate required power
  controller.setPower(Output);       // Apply power to TRIAC

  // Print status periodically for debugging
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 200)
  {
    lastPrintTime = millis();
    Serial.printf("Setpoint: %.1fV, Current: %.1fV, PID Out (Power): %.1f%%, Freq: %.2fHz\n",
                  Setpoint,
                  Input,
                  Output,
                  controller.getFrequency());
  }
}