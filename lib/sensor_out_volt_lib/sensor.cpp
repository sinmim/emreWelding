// sensor.cpp
#include "sensor.h"
#include <Arduino.h>
#include <BL0942.h> // The underlying library for the sensor chip

// Define the hardware serial pins for the sensor
#define BL0942_RX 7
#define BL0942_TX 15

// Create an instance of the sensor library object
bl0942::BL0942 blSensor(Serial1);

// Global variables to store the most recent raw sensor data.
// 'volatile' is used to ensure the variables are read correctly from memory.
volatile float raw_voltage = 0.0;
volatile float raw_current = 0.0;

/**
 * @brief A private callback function that the BL0942 library calls when it has new data.
 * This function simply updates our global variables.
 * @param data A reference to the struct containing the new sensor data.
 */
void dataReceivedCallback(bl0942::SensorData &data)
{
  raw_voltage = data.voltage;
  raw_current = data.current;
}

/**
 * @brief Initializes the sensor hardware.
 */
void initSensor()
{
  // Start the hardware serial port connected to the sensor
  Serial1.begin(9600, SERIAL_8N1, BL0942_RX, BL0942_TX);

  // Initialize the sensor object
  blSensor.setup();

  // Tell the sensor library which function to call when new data is ready
  blSensor.onDataReceived(dataReceivedCallback);
}

/**
 * @brief Checks for new data from the sensor.
 * This must be called from the main loop() to process incoming serial data
 * and trigger the dataReceivedCallback if a full message has arrived.
 */
void updateSensor()
{
  blSensor.update();
  blSensor.loop();
}

/**
 * @brief Gets the latest raw voltage reading.
 */
float getVoltage()
{
  return raw_voltage;
}

/**
 * @brief Gets the latest raw current reading.
 */
float getCurrent()
{
  return raw_current;
}