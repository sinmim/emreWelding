#include "TriacController.h"
#include <Arduino.h>

TriacController::TriacController() {}
TriacController::~TriacController() {
    if (_firingTimer) esp_timer_delete(_firingTimer);
    if (_stopPulseTimer) esp_timer_delete(_stopPulseTimer);
}

bool TriacController::begin(int zcPin, int triacPin, float minFreq, float maxFreq) {
    _triacPin = triacPin;

    ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcAttachPin(_triacPin, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, 0);

    const esp_timer_create_args_t firing_timer_args = {.callback = &isr_fireTriac, .arg = this, .name = "firing_timer"};
    if (esp_timer_create(&firing_timer_args, &_firingTimer) != ESP_OK) return false;

    const esp_timer_create_args_t stop_timer_args = {.callback = &isr_stopPulseTrain, .arg = this, .name = "stop_pulse_timer"};
    if (esp_timer_create(&stop_timer_args, &_stopPulseTimer) != ESP_OK) return false;

    if (!_freqMonitor.begin(zcPin, minFreq, maxFreq)) return false;
    
    auto handler = [this]() { isr_handleTrueZeroCross(this); };
    _freqMonitor.attachZeroCrossCallback(handler);
    _freqMonitor.attachHalfCycleCallback(handler);
    
    _outputEnabled = true;
    setPower(0);
    return true;
}

void TriacController::update() { _freqMonitor.update(); }
void TriacController::setPower(float power) {
    _powerLevel = constrain(power, 0.0, 100.0);
    _firingAngle = _mapPowerToAngle(_powerLevel);
}
void TriacController::setMeasurementDelay(unsigned int delay_us) { _freqMonitor.setMeasurementDelay(delay_us); }
void TriacController::enableOutput() { _outputEnabled = true; }
void TriacController::disableOutput() {
    _outputEnabled = false;
    _stopPulseTrain();
}

bool TriacController::isEnabled() const { return _outputEnabled; }
bool TriacController::isFaulty() const { return _freqMonitor.isFaulty(); }
float TriacController::getFrequency() const { return _freqMonitor.getFrequency(); }
float TriacController::getCurrentPower() const { return _powerLevel; }

float TriacController::_mapPowerToAngle(float power) {
    const float minAngle = 5.0;
    const float maxAngle = 175.0;
    return maxAngle - (power / 100.0) * (maxAngle - minAngle);
}

void IRAM_ATTR TriacController::_onTrueZeroCross() {
    if (!_outputEnabled || _freqMonitor.isFaulty()) {
        return;
    }
    // We are at the true zero-cross. Calculate the delay to the firing angle.
    unsigned long half_period = _freqMonitor.getPeriod() / 2;
    unsigned long angle_delay_us = (unsigned long)((_firingAngle / 180.0) * half_period);

    // Arm the firing timer
    if (angle_delay_us > 0) {
        esp_timer_start_once(_firingTimer, angle_delay_us);
    } else {
        _fireTriac(); // Fire immediately if angle is 0
    }
}

void IRAM_ATTR TriacController::_fireTriac() {
    ledcWrite(LEDC_CHANNEL, LEDC_DUTY_CYCLE);
    esp_timer_start_once(_stopPulseTimer, PULSE_TRAIN_DURATION_US);
}

void IRAM_ATTR TriacController::_stopPulseTrain() {
    ledcWrite(LEDC_CHANNEL, 0);
}

void IRAM_ATTR TriacController::isr_handleTrueZeroCross(void* arg) {
    static_cast<TriacController*>(arg)->_onTrueZeroCross();
}
void IRAM_ATTR TriacController::isr_fireTriac(void* arg) {
    static_cast<TriacController*>(arg)->_fireTriac();
}
void IRAM_ATTR TriacController::isr_stopPulseTrain(void* arg) {
    static_cast<TriacController*>(arg)->_stopPulseTrain();
}