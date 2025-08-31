#include "ACFrequencyMonitor.h"
#include <algorithm> // For std::sort

ACFrequencyMonitor *ACFrequencyMonitor::_instance = nullptr;

ACFrequencyMonitor::ACFrequencyMonitor() {
    for (int i = 0; i < MEDIAN_FILTER_SIZE; ++i) {
        _periodBuffer[i] = 20000; // Pre-fill with 50Hz
    }
}

ACFrequencyMonitor::~ACFrequencyMonitor() {
    if (_predictiveTimer) esp_timer_delete(_predictiveTimer);
    if (_halfCycleTimer) esp_timer_delete(_halfCycleTimer);
    if (_zcPin >= 0) detachInterrupt(digitalPinToInterrupt(_zcPin));
}

bool ACFrequencyMonitor::begin(int zcPin, float minFreq, float maxFreq) {
    _instance = this;
    _zcPin = zcPin;

    const esp_timer_create_args_t predictive_timer_args = {.callback = &isr_predictiveCallback, .arg = this, .name = "predictive_timer"};
    esp_err_t err1 = esp_timer_create(&predictive_timer_args, &_predictiveTimer);

    const esp_timer_create_args_t half_cycle_timer_args = {.callback = &isr_halfCycleCallback, .arg = this, .name = "half_cycle_timer"};
    esp_err_t err2 = esp_timer_create(&half_cycle_timer_args, &_halfCycleTimer);

    pinMode(_zcPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_zcPin), isr_handleHardwareZeroCross, RISING);

    return (err1 == ESP_OK && err2 == ESP_OK);
}

void ACFrequencyMonitor::update() {
    if (_newHardwareZcEvent) {
        noInterrupts();
        _newHardwareZcEvent = false;
        interrupts();
        processNewPeriod();
    }
}

void IRAM_ATTR ACFrequencyMonitor::isr_handleHardwareZeroCross() {
    if (!_instance) return;
    _instance->_lastHardwareZCTime_us = micros();
    _instance->_newHardwareZcEvent = true;
}

void ACFrequencyMonitor::processNewPeriod() {
    static unsigned long last_processed_time = 0;
    unsigned long period = _lastHardwareZCTime_us - last_processed_time;
    last_processed_time = _lastHardwareZCTime_us;

    // Basic validation and filtering of the period measurement
    bool periodIsOk = (period > 15000 && period < 22222); // Corresponds to ~45-66 Hz
    if (periodIsOk) {
        _isFaulty = false;
        _currentPeriod_us = period;
        _lastGoodPeriod_us = period;
    } else {
        _isFaulty = true;
        _currentPeriod_us = _lastGoodPeriod_us; // Freewheel with the last known good period
    }

    if (_debugEnabled) {
        Serial.printf("[AC_MONITOR] Hardware ZC. Period: %lu us. OK: %d\n", period, periodIsOk);
    }
    
    // Arm the predictive timer to fire at the next TRUE zero-cross
    long delay_to_next_true_zc = (long)_currentPeriod_us - (long)_measurementDelay_us;

    if (delay_to_next_true_zc > 0) {
        esp_timer_start_once(_predictiveTimer, delay_to_next_true_zc);
        if (_debugEnabled) Serial.printf("[AC_MONITOR] Arming predictive timer for %ld us\n", delay_to_next_true_zc);
    }
}

void IRAM_ATTR ACFrequencyMonitor::isr_predictiveCallback(void* arg) {
    ACFrequencyMonitor* instance = static_cast<ACFrequencyMonitor*>(arg);
    
    // 1. Trigger the user's main zero-cross callback
    if (instance->_zeroCrossCallback) {
        instance->_zeroCrossCallback();
    }
    
    // 2. Immediately arm the timer for the next half-cycle event
    unsigned long half_period = instance->_currentPeriod_us / 2;
    if (half_period > 0) {
        esp_timer_start_once(instance->_halfCycleTimer, half_period);
    }
}

void IRAM_ATTR ACFrequencyMonitor::isr_halfCycleCallback(void* arg) {
    ACFrequencyMonitor* instance = static_cast<ACFrequencyMonitor*>(arg);
    if (instance->_halfCycleCallback) {
        instance->_halfCycleCallback();
    }
}

void ACFrequencyMonitor::attachZeroCrossCallback(std::function<void()> callback) { _zeroCrossCallback = callback; }
void ACFrequencyMonitor::attachHalfCycleCallback(std::function<void()> callback) { _halfCycleCallback = callback; }
void ACFrequencyMonitor::setMeasurementDelay(unsigned int delay_us) { _measurementDelay_us = delay_us; }
void ACFrequencyMonitor::setDebug(bool enabled) { _debugEnabled = enabled; }
unsigned long ACFrequencyMonitor::getPeriod() const { return _currentPeriod_us; }
bool ACFrequencyMonitor::isFaulty() const { return _isFaulty; }
float ACFrequencyMonitor::getFrequency() const {
    if (_currentPeriod_us == 0) return 0.0;
    return 1000000.0 / _currentPeriod_us;
}