#include "TriacController.h"
#include <Arduino.h>

TriacController::TriacController() {}

TriacController::~TriacController()
{
    // Clean up all created timers to prevent memory leaks
    if (_firingTimer)
        esp_timer_delete(_firingTimer);
    if (_stopPulseTimer)
        esp_timer_delete(_stopPulseTimer);
    if (_halfCycleTimer) // <-- ADDED
        esp_timer_delete(_halfCycleTimer);
    if (_zcPin >= 0)
        detachInterrupt(digitalPinToInterrupt(_zcPin));
}

bool TriacController::begin(int zcPin, int triacPin, float minFreq, float maxFreq, uint8_t filterSize)
{
    _zcPin = zcPin;
    _triacPin = triacPin;

    // 1. Configure LEDC PWM Peripheral for the pulse train
    ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcAttachPin(_triacPin, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, 0); // Ensure output is off initially

    // 2. Create the timer that will introduce the firing angle delay
    const esp_timer_create_args_t firing_timer_args = {
        .callback = &isr_fireTriac,
        .arg = this,
        .name = "firing_timer"};
    if (esp_timer_create(&firing_timer_args, &_firingTimer) != ESP_OK)
        return false;

    // 3. Create the timer that will stop the short pulse train
    const esp_timer_create_args_t stop_timer_args = {
        .callback = &isr_stopPulseTrain,
        .arg = this,
        .name = "stop_pulse_timer"};
    if (esp_timer_create(&stop_timer_args, &_stopPulseTimer) != ESP_OK)
        return false;

    // 4. Create the timer that will simulate the falling-edge zero-cross
    const esp_timer_create_args_t half_cycle_timer_args = {// <-- ADDED BLOCK
                                                           .callback = &isr_handleHalfCycle,
                                                           .arg = this,
                                                           .name = "half_cycle_timer"};
    if (esp_timer_create(&half_cycle_timer_args, &_halfCycleTimer) != ESP_OK)
        return false;

    // 5. Initialize the AC Frequency Monitor
    if (!_freqMonitor.begin(filterSize, minFreq, maxFreq))
    {
        return false;
    }

    // 6. Attach the hardware interrupt for the RISING-EDGE-ONLY zero-cross detector
    pinMode(_zcPin, INPUT_PULLUP);
    // Make sure this is set to RISING, not CHANGE
    attachInterruptArg(digitalPinToInterrupt(_zcPin), isr_handleHardwareZeroCross, this, RISING);

    // 7. Set initial state
    _outputEnabled = true;
    setPower(0);

    return true;
}

void TriacController::setPower(float power)
{
    _powerLevel = constrain(power, 0.0, 100.0);
    _firingAngle = _mapPowerToAngle(_powerLevel);
}

void TriacController::setMeasurementDelay(unsigned int delay_us)
{
    _measurementDelay_us = delay_us;
}

void TriacController::setLowPassFilterAlpha(float alpha)
{
    _freqMonitor.setLowPassFilterAlpha(alpha);
}

void TriacController::enableOutput()
{
    _outputEnabled = true;
}

void TriacController::disableOutput()
{
    _outputEnabled = false;
    _stopPulseTrain(); // Immediately stop any ongoing pulse for safety
}

// --- Status Functions ---
bool TriacController::isEnabled() const { return _outputEnabled; }
bool TriacController::isFaulty() const { return _freqMonitor.isFaulty(); }
float TriacController::getFrequency() const { return _freqMonitor.getFrequency(); }
float TriacController::getCurrentPower() const { return _powerLevel; }

// --- Private Methods ---
float TriacController::_mapPowerToAngle(float power)
{
    const float minAngle = 5.0;
    const float maxAngle = 175.0;
    return maxAngle - (power / 100.0) * (maxAngle - minAngle);
}

void IRAM_ATTR TriacController::isr_handleHardwareZeroCross(void *arg)
{
    TriacController *instance = static_cast<TriacController *>(arg);
    unsigned long now_us = micros();

    // Calculate raw period and update the frequency monitor
    unsigned long raw_period_us = now_us - instance->_lastZcTime_us;
    instance->_lastZcTime_us = now_us;
    instance->_freqMonitor.addNewPeriodSample(raw_period_us);

    // --- MODIFIED BLOCK ---
    // Trigger the firing logic for the rising edge (first half-cycle)
    instance->_onHardwareZeroCross();

    // Now, arm the timer to trigger again at the simulated falling edge
    unsigned long half_period_us = instance->_freqMonitor.getPeriod() / 2;
    long half_cycle_timer_delay = (long)half_period_us - (long)instance->_measurementDelay_us;
    if (half_cycle_timer_delay > 0)
    {
        esp_timer_start_once(instance->_halfCycleTimer, half_cycle_timer_delay);
    }
}

// NEW FUNCTION: Called when the half-cycle timer expires
void IRAM_ATTR TriacController::isr_handleHalfCycle(void *arg)
{
    static_cast<TriacController *>(arg)->_onHalfCycle();
}

void TriacController::_onHardwareZeroCross()
{
    if (!_outputEnabled || _freqMonitor.isFaulty())
    {
        esp_timer_stop(_firingTimer);
        return;
    }

    unsigned long half_period_us = _freqMonitor.getPeriod() / 2;
    if (half_period_us == 0)
        return;

    unsigned long angle_delay_us = (unsigned long)((_firingAngle / 180.0) * half_period_us);

    // For the hardware-detected ZC, we must compensate for the detector's delay
    long timer_delay_us = (long)angle_delay_us - (long)_measurementDelay_us;

    if (timer_delay_us > 50)
    {
        esp_timer_start_once(_firingTimer, timer_delay_us);
    }
    else
    {
        _fireTriac();
    }
}

// NEW FUNCTION: Handles the firing logic for the simulated falling edge
void TriacController::_onHalfCycle()
{
    if (!_outputEnabled || _freqMonitor.isFaulty())
    {
        esp_timer_stop(_firingTimer);
        return;
    }

    unsigned long half_period_us = _freqMonitor.getPeriod() / 2;
    if (half_period_us == 0)
        return;

    unsigned long angle_delay_us = (unsigned long)((_firingAngle / 180.0) * half_period_us);

    // For the simulated ZC, there is no hardware delay to compensate for.
    // We simply use the calculated angle delay directly.
    if (angle_delay_us > 50)
    {
        esp_timer_start_once(_firingTimer, angle_delay_us);
    }
    else
    {
        _fireTriac();
    }
}

void IRAM_ATTR TriacController::_fireTriac()
{
    ledcWrite(LEDC_CHANNEL, LEDC_DUTY_CYCLE);
    esp_timer_start_once(_stopPulseTimer, PULSE_TRAIN_DURATION_US);
}

void IRAM_ATTR TriacController::_stopPulseTrain()
{
    ledcWrite(LEDC_CHANNEL, 0);
}

void IRAM_ATTR TriacController::isr_fireTriac(void *arg)
{
    static_cast<TriacController *>(arg)->_fireTriac();
}

void IRAM_ATTR TriacController::isr_stopPulseTrain(void *arg)
{
    static_cast<TriacController *>(arg)->_stopPulseTrain();
}