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
}

bool TriacController::begin(int zcPin, int triacPin, float minFreq, float maxFreq, uint8_t filterSize)
{
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

    // 4. Initialize the AC Frequency Monitor, passing the filterSize to it
    if (!_freqMonitor.begin(zcPin, minFreq, maxFreq, filterSize))
    {
        return false;
    }

    // 5. Attach our internal handler to the monitor's predictive callbacks
    auto handler = [this]()
    { isr_handleTrueZeroCross(this); };
    _freqMonitor.attachZeroCrossCallback(handler);
    _freqMonitor.attachHalfCycleCallback(handler);

    // 6. Set initial state
    _outputEnabled = true;
    setPower(0);

    return true;
}

void TriacController::update()
{
    _freqMonitor.update();
}

void TriacController::setPower(float power)
{
    _powerLevel = constrain(power, 0.0, 100.0);
    // This class now manages the firing angle internally.
    // The monitor no longer needs to know about it.
    _firingAngle = _mapPowerToAngle(_powerLevel);
}

void TriacController::setMeasurementDelay(unsigned int delay_us)
{
    // Pass the setting down to the monitor
    _freqMonitor.setMeasurementDelay(delay_us);
}
void TriacController::setLowPassFilterAlpha(float alpha) {
    _freqMonitor.setLowPassFilterAlpha(alpha);
}
void TriacController::enableOutput()
{
    _outputEnabled = true;
}

void TriacController::disableOutput()
{
    _outputEnabled = false;
    // Immediately stop any ongoing pulse for safety
    _stopPulseTrain();
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

void IRAM_ATTR TriacController::_onTrueZeroCross()
{
    // This function is called by the monitor at the predicted TRUE zero-cross moment.
    if (!_outputEnabled || _freqMonitor.isFaulty())
    {
        return;
    }

    // Calculate the delay needed to reach the desired firing angle
    unsigned long half_period = _freqMonitor.getPeriod() / 2;
    unsigned long angle_delay_us = (unsigned long)((_firingAngle / 180.0) * half_period);

    // Arm the firing timer with this delay. The timer will call _fireTriac() when it expires.
    if (angle_delay_us > 50)
    { // Add small safety margin
        esp_timer_start_once(_firingTimer, angle_delay_us);
    }
    else
    {
        _fireTriac(); // Fire immediately if angle is at or near zero
    }
}

void IRAM_ATTR TriacController::_fireTriac()
{
    // This is called after the angle delay. It starts the high-frequency pulse train.
    ledcWrite(LEDC_CHANNEL, LEDC_DUTY_CYCLE);
    // Arm the stop timer to turn the pulse train off after a short duration.
    esp_timer_start_once(_stopPulseTimer, PULSE_TRAIN_DURATION_US);
}

void IRAM_ATTR TriacController::_stopPulseTrain()
{
    // This is called by the stop timer to end the pulse train.
    ledcWrite(LEDC_CHANNEL, 0);
}

// --- Static ISR Wrappers ---
// These C-style functions are required for the ESP-IDF timer and interrupt system.
// They simply cast the 'arg' pointer back to our class instance and call the appropriate member function.

void IRAM_ATTR TriacController::isr_handleTrueZeroCross(void *arg)
{
    static_cast<TriacController *>(arg)->_onTrueZeroCross();
}

void IRAM_ATTR TriacController::isr_fireTriac(void *arg)
{
    static_cast<TriacController *>(arg)->_fireTriac();
}

void IRAM_ATTR TriacController::isr_stopPulseTrain(void *arg)
{
    static_cast<TriacController *>(arg)->_stopPulseTrain();
}