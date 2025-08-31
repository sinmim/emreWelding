#ifndef AC_FREQUENCY_MONITOR_H
#define AC_FREQUENCY_MONITOR_H

#include <Arduino.h>
#include <functional>

class ACFrequencyMonitor
{
public:
    ACFrequencyMonitor();
    ~ACFrequencyMonitor();

    /**
     * @brief Initializes the monitor.
     * @param zcPin The GPIO pin for the zero-cross detector input.
     * @param minFreq The minimum expected AC frequency.
     * @param maxFreq The maximum expected AC frequency.
     * @param filterSize The size of the median filter window. MUST BE AN ODD NUMBER (e.g., 3, 5, 7).
     * @return True on success, false on failure.
     */
    bool begin(int zcPin, float minFreq = 45.0, float maxFreq = 65.0, uint8_t filterSize = 5);

    void update();

    void attachZeroCrossCallback(std::function<void()> callback);
    void attachHalfCycleCallback(std::function<void()> callback);

    void setMeasurementDelay(unsigned int delay_us);
    void setLowPassFilterAlpha(float alpha); // New function
    void setDebug(bool enabled);

    float getFrequency() const;
    unsigned long getPeriod() const;
    bool isFaulty() const;

private:
    // Filtering variables
    uint8_t _filterSize = 0;
    unsigned long *_periodBuffer = nullptr;
    unsigned long *_sortedBuffer = nullptr;
    int _bufferIndex = 0;
    bool _bufferFull = false;
    float _lpfAlpha = 1.0; // New low-pass filter alpha. Default to 1.0 (no filtering).
    // ISRs
    static void IRAM_ATTR isr_handleHardwareZeroCross();
    static void IRAM_ATTR isr_predictiveCallback(void *arg);
    static void IRAM_ATTR isr_halfCycleCallback(void *arg);

    // Internal Methods
    void processNewPeriod();
    void updateFilteredPeriod(unsigned long newPeriod);

    // Member Variables
    int _zcPin = -1;
    unsigned int _measurementDelay_us = 0;
    bool _debugEnabled = false;
    bool _isFaulty = true;

    volatile unsigned long _lastHardwareZCTime_us = 0;
    volatile bool _newHardwareZcEvent = false;

    unsigned long _currentPeriod_us = 20000;
    float _filteredFrequency = 50.0;

    std::function<void()> _zeroCrossCallback = nullptr;
    std::function<void()> _halfCycleCallback = nullptr;

    esp_timer_handle_t _predictiveTimer = nullptr;
    esp_timer_handle_t _halfCycleTimer = nullptr;

    static ACFrequencyMonitor *_instance;
};

#endif // AC_FREQUENCY_MONITOR_H