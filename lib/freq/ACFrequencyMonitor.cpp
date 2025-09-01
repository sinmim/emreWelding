#include "ACFrequencyMonitor.h"
#include <algorithm> // For std::sort

ACFrequencyMonitor::ACFrequencyMonitor() {}

ACFrequencyMonitor::~ACFrequencyMonitor()
{
    // Free dynamically allocated memory
    delete[] _periodBuffer;
    delete[] _sortedBuffer;
}

bool ACFrequencyMonitor::begin(uint8_t filterSize, float minFreq, float maxFreq)
{
    // Ensure filter size is odd, default to 3 if not.
    _filterSize = (filterSize % 2 != 0) ? filterSize : 3;

    // Dynamically allocate buffers for the filter
    _periodBuffer = new unsigned long[_filterSize];
    _sortedBuffer = new unsigned long[_filterSize];
    if (!_periodBuffer || !_sortedBuffer)
        return false; // Allocation failed

    // Pre-fill buffer with a sensible default (50Hz)
    unsigned long default_period = 1000000.0 / 50.0;
    for (int i = 0; i < _filterSize; ++i)
    {
        _periodBuffer[i] = default_period;
    }
    _currentPeriod_us = default_period;

    // Calculate period limits from frequency for validation
    _minPeriod_us = 1000000.0 / maxFreq;
    _maxPeriod_us = 1000000.0 / minFreq;

    return true;
}

void ACFrequencyMonitor::addNewPeriodSample(unsigned long rawPeriod_us)
{
    // Validate the raw period against the configured frequency range
    bool periodIsOk = (rawPeriod_us > _minPeriod_us && rawPeriod_us < _maxPeriod_us);

    if (periodIsOk)
    {
        _isFaulty = false;
        updateFilteredPeriod(rawPeriod_us);
    }
    else
    {
        _isFaulty = true;
    }

    if (_debugEnabled)
    {
        Serial.printf("[AC_MONITOR] Raw Period: %lu us. Filtered: %lu us. OK: %d\n", rawPeriod_us, _currentPeriod_us, periodIsOk);
    }
}

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
        // Once the buffer is full, we calculate a median to get a more stable value.
        for (int i = 0; i < _filterSize; i++)
        {
            _sortedBuffer[i] = _periodBuffer[i];
        }
        std::sort(_sortedBuffer, _sortedBuffer + _filterSize);
        valueForLpf = _sortedBuffer[_filterSize / 2];
    }
    else
    {
        // Before the buffer is full, use the raw period as the input for the LPF.
        valueForLpf = newPeriod;
    }

    // --- Stage 2: Low-Pass Filter (to smooth jitter) ---
    // This calculation is always performed, ensuring a smooth output.
    _currentPeriod_us = (_lpfAlpha * valueForLpf) + ((1.0 - _lpfAlpha) * _currentPeriod_us);
}

// --- Public Functions ---
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