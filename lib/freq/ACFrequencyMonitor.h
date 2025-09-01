#ifndef AC_FREQUENCY_MONITOR_H
#define AC_FREQUENCY_MONITOR_H

#include <Arduino.h>

class ACFrequencyMonitor
{
public:
    ACFrequencyMonitor();
    ~ACFrequencyMonitor();

    /**
     * @brief Initializes the monitor's buffers and settings.
     * @param filterSize The size of the median filter window. MUST BE AN ODD NUMBER (e.g., 3, 5, 7).
     * @param minFreq The minimum expected AC frequency for validation.
     * @param maxFreq The maximum expected AC frequency for validation.
     * @return True on success, false on failure.
     */
    bool begin(uint8_t filterSize = 5, float minFreq = 45.0, float maxFreq = 65.0);

    /**
     * @brief Processes a new raw period measurement.
     * @param rawPeriod_us The latest period measurement in microseconds.
     */
    void addNewPeriodSample(unsigned long rawPeriod_us);

    // --- Settings ---
    void setLowPassFilterAlpha(float alpha);
    void setDebug(bool enabled);

    // --- Status ---
    float getFrequency() const;
    unsigned long getPeriod() const;
    bool isFaulty() const;

private:
    void updateFilteredPeriod(unsigned long newPeriod);

    // Filtering variables
    uint8_t _filterSize = 0;
    unsigned long *_periodBuffer = nullptr;
    unsigned long *_sortedBuffer = nullptr;
    int _bufferIndex = 0;
    bool _bufferFull = false;
    float _lpfAlpha = 1.0;

    // Member Variables
    bool _debugEnabled = false;
    bool _isFaulty = true;
    unsigned long _currentPeriod_us = 20000; // Default to 50Hz
    unsigned long _maxPeriod_us;
    unsigned long _minPeriod_us;
};

#endif // AC_FREQUENCY_MONITOR_H