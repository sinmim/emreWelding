#ifndef TRIAC_CONTROLLER_H
#define TRIAC_CONTROLLER_H

#include "ACFrequencyMonitor.h"

#define LEDC_CHANNEL      0
#define LEDC_FREQ_HZ      10000
#define LEDC_RESOLUTION   8
#define LEDC_DUTY_CYCLE   128
#define PULSE_TRAIN_DURATION_US 500

class TriacController {
public:
    TriacController();
    ~TriacController();

    bool begin(int zcPin, int triacPin, float minFreq = 45.0, float maxFreq = 65.0);
    void update();
    void setPower(float power);
    void setMeasurementDelay(unsigned int delay_us);
    void enableOutput();
    void disableOutput();

    bool isEnabled() const;
    bool isFaulty() const;
    float getFrequency() const;
    float getCurrentPower() const;

private:
    ACFrequencyMonitor _freqMonitor;
    int _triacPin = -1;
    bool _outputEnabled = false;
    float _powerLevel = 0.0;
    float _firingAngle = 180.0;
    
    esp_timer_handle_t _firingTimer = nullptr;
    esp_timer_handle_t _stopPulseTimer = nullptr;

    float _mapPowerToAngle(float power);

    // Static ISR wrappers
    static void IRAM_ATTR isr_handleTrueZeroCross(void* arg);
    static void IRAM_ATTR isr_fireTriac(void* arg);
    static void IRAM_ATTR isr_stopPulseTrain(void* arg);

    // Member function implementations for ISRs
    void _onTrueZeroCross();
    void _fireTriac();
    void _stopPulseTrain();
};

#endif // TRIAC_CONTROLLER_H