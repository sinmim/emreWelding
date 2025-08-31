#ifndef AC_FREQUENCY_MONITOR_H
#define AC_FREQUENCY_MONITOR_H

#include <Arduino.h>
#include <functional>

#define MEDIAN_FILTER_SIZE 7

class ACFrequencyMonitor {
public:
    ACFrequencyMonitor();
    ~ACFrequencyMonitor();

    bool begin(int zcPin, float minFreq = 45.0, float maxFreq = 65.0);
    void update();

    void attachZeroCrossCallback(std::function<void()> callback);
    void attachHalfCycleCallback(std::function<void()> callback);
    
    void setMeasurementDelay(unsigned int delay_us);
    void setDebug(bool enabled);

    float getFrequency() const;
    unsigned long getPeriod() const;
    bool isFaulty() const;

private:
    // ISRs
    static void IRAM_ATTR isr_handleHardwareZeroCross();
    static void IRAM_ATTR isr_predictiveCallback(void* arg);
    static void IRAM_ATTR isr_halfCycleCallback(void* arg);

    // Internal Methods
    void processNewPeriod();

    // Member Variables
    int _zcPin = -1;
    unsigned int _measurementDelay_us = 0;
    bool _debugEnabled = false;
    bool _isFaulty = true;
    
    volatile unsigned long _lastHardwareZCTime_us = 0;
    volatile bool _newHardwareZcEvent = false;

    unsigned long _currentPeriod_us = 20000; // Default to 50Hz
    unsigned long _lastGoodPeriod_us = 20000;
    unsigned long _periodBuffer[MEDIAN_FILTER_SIZE];
    int _bufferIndex = 0;
    bool _bufferFull = false;
    float _filteredFrequency = 0.0;
    
    std::function<void()> _zeroCrossCallback = nullptr;
    std::function<void()> _halfCycleCallback = nullptr;

    esp_timer_handle_t _predictiveTimer = nullptr;
    esp_timer_handle_t _halfCycleTimer = nullptr;

    static ACFrequencyMonitor* _instance; 
};

#endif // AC_FREQUENCY_MONITOR_H