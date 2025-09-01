// sensor.h
#ifndef SENSOR_H
#define SENSOR_H

/**
 * @brief Initializes the sensor hardware. Call this once in your setup().
 */
void initSensor();

/**
 * @brief Checks for new data from the sensor. Call this repeatedly in your main loop().
 */
void updateSensor();

/**
 * @brief Gets the latest raw voltage reading.
 * @return The voltage in Volts.
 */
float getVoltage();

/**
 * @brief Gets the latest raw current reading.
 * @return The current in Amps.
 */
float getCurrent();

#endif // SENSOR_H