#include "ACFrequencyMonitor.h"
#include <algorithm> // For std::sort

ACFrequencyMonitor *ACFrequencyMonitor::_instance = nullptr;

ACFrequencyMonitor::ACFrequencyMonitor() {}

ACFrequencyMonitor::~ACFrequencyMonitor()
{
    if (_predictiveTimer)
        esp_timer_delete(_predictiveTimer);
    if (_halfCycleTimer)
        esp_timer_delete(_halfCycleTimer);
    if (_zcPin >= 0)
        detachInterrupt(digitalPinToInterrupt(_zcPin));

    // Free dynamically allocated memory
    delete[] _periodBuffer;
    delete[] _sortedBuffer;
}

bool ACFrequencyMonitor::begin(int zcPin, float minFreq, float maxFreq, uint8_t filterSize)
{
    _instance = this;
    _zcPin = zcPin;

    // Ensure filter size is odd, default to 3 if not.
    _filterSize = (filterSize % 2 != 0) ? filterSize : 3;

    // Dynamically allocate buffers for the filter
    _periodBuffer = new unsigned long[_filterSize];
    _sortedBuffer = new unsigned long[_filterSize];
    if (!_periodBuffer || !_sortedBuffer)
        return false; // Allocation failed

    // Pre-fill buffer with a sensible default (50Hz)
    for (int i = 0; i < _filterSize; ++i)
    {
        _periodBuffer[i] = 20000;
    }

    const esp_timer_create_args_t predictive_timer_args = {.callback = &isr_predictiveCallback, .arg = this, .name = "predictive_timer"};
    esp_err_t err1 = esp_timer_create(&predictive_timer_args, &_predictiveTimer);

    const esp_timer_create_args_t half_cycle_timer_args = {.callback = &isr_halfCycleCallback, .arg = this, .name = "half_cycle_timer"};
    esp_err_t err2 = esp_timer_create(&half_cycle_timer_args, &_halfCycleTimer);

    pinMode(_zcPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_zcPin), isr_handleHardwareZeroCross, RISING);

    return (err1 == ESP_OK && err2 == ESP_OK);
}

void ACFrequencyMonitor::update()
{
    if (_newHardwareZcEvent)
    {
        noInterrupts();
        _newHardwareZcEvent = false;
        interrupts();
        processNewPeriod();
    }
}

void IRAM_ATTR ACFrequencyMonitor::isr_handleHardwareZeroCross()
{
    if (!_instance)
        return;
    _instance->_lastHardwareZCTime_us = micros();
    _instance->_newHardwareZcEvent = true;
}

void ACFrequencyMonitor::processNewPeriod()
{
    static unsigned long last_processed_time = 0;
    unsigned long raw_period = _lastHardwareZCTime_us - last_processed_time;
    last_processed_time = _lastHardwareZCTime_us;

    bool periodIsOk = (raw_period > 15000 && raw_period < 22222);
    if (periodIsOk)
    {
        _isFaulty = false;
        updateFilteredPeriod(raw_period);
    }
    else
    {
        _isFaulty = true;
    }

    if (_debugEnabled)
    {
        Serial.printf("[AC_MONITOR] Raw Period: %lu us. Filtered: %lu us. OK: %d\n", raw_period, _currentPeriod_us, periodIsOk);
    }

    long delay_to_next_true_zc = (long)_currentPeriod_us - (long)_measurementDelay_us;
    if (delay_to_next_true_zc > 0)
    {
        esp_timer_start_once(_predictiveTimer, delay_to_next_true_zc);
    }
}

// In ACFrequencyMonitor.cpp, replace the existing function with this one.

void ACFrequencyMonitor::updateFilteredPeriod(unsigned long newPeriod)
{
    // Add the new measurement to our buffer for the median filter.
    _periodBuffer[_bufferIndex++] = newPeriod;
    if (_bufferIndex >= _filterSize)
    {
        _bufferIndex = 0;
        _bufferFull = true;
    }

    unsigned long valueForLpf;

    if (_bufferFull)
    {
        // --- Stage 1: Median Filter (to reject spikes) ---
        // Once the buffer is full, we can calculate a median to get a more stable value.
        for (int i = 0; i < _filterSize; i++)
        {
            _sortedBuffer[i] = _periodBuffer[i];
        }
        std::sort(_sortedBuffer, _sortedBuffer + _filterSize);
        valueForLpf = _sortedBuffer[_filterSize / 2];
    }
    else
    {
        // Before the buffer is full, we use the raw period as the input for the LPF.
        valueForLpf = newPeriod;
    }

    // --- Stage 2: Low-Pass Filter (to smooth jitter) ---
    // This calculation is now ALWAYS performed, ensuring smooth output.
    // With lpfAlpha = 0.005, this line will produce a very slow-moving average.
    _currentPeriod_us = (_lpfAlpha * valueForLpf) + ((1.0 - _lpfAlpha) * _currentPeriod_us);
}
void IRAM_ATTR ACFrequencyMonitor::isr_predictiveCallback(void *arg)
{
    ACFrequencyMonitor *instance = static_cast<ACFrequencyMonitor *>(arg);
    if (instance->_zeroCrossCallback)
    {
        instance->_zeroCrossCallback();
    }
    unsigned long half_period = instance->_currentPeriod_us / 2;
    if (half_period > 0)
    {
        esp_timer_start_once(instance->_halfCycleTimer, half_period);
    }
}

void IRAM_ATTR ACFrequencyMonitor::isr_halfCycleCallback(void *arg)
{
    ACFrequencyMonitor *instance = static_cast<ACFrequencyMonitor *>(arg);
    if (instance->_halfCycleCallback)
    {
        instance->_halfCycleCallback();
    }
}

// --- Public Functions ---
void ACFrequencyMonitor::attachZeroCrossCallback(std::function<void()> callback) { _zeroCrossCallback = callback; }
void ACFrequencyMonitor::attachHalfCycleCallback(std::function<void()> callback) { _halfCycleCallback = callback; }
void ACFrequencyMonitor::setMeasurementDelay(unsigned int delay_us) { _measurementDelay_us = delay_us; }
void ACFrequencyMonitor::setLowPassFilterAlpha(float alpha)
{
    _lpfAlpha = constrain(alpha, 0.0, 1.0); // Ensure alpha is between 0.0 and 1.0
}
void ACFrequencyMonitor::setDebug(bool enabled) { _debugEnabled = enabled; }
unsigned long ACFrequencyMonitor::getPeriod() const { return _currentPeriod_us; }
bool ACFrequencyMonitor::isFaulty() const { return _isFaulty; }
float ACFrequencyMonitor::getFrequency() const
{
    if (_currentPeriod_us == 0)
        return 0.0;
    return 1000000.0 / _currentPeriod_us;
}