#ifndef TRIAC_CONTROLLER_H
#define TRIAC_CONTROLLER_H

#include "ACFrequencyMonitor.h"

// --- Default Configuration for the Pulse Train ---
#define LEDC_CHANNEL      0       // ESP32 LEDC channel 0
#define LEDC_FREQ_HZ      10000   // 10 kHz pulse train frequency
#define LEDC_RESOLUTION   8       // 8-bit resolution (0-255)
#define LEDC_DUTY_CYCLE   128     // 50% duty cycle for the pulses
#define PULSE_TRAIN_DURATION_US 200 // Duration of the pulse burst in microseconds

class TriacController {
public:
    TriacController();
    ~TriacController();

    /**
     * @brief Initializes the controller.
     * @param zcPin The GPIO pin for the zero-cross detector input.
     * @param triacPin The GPIO pin for the TRIAC gate trigger output.
     * @param minFreq The minimum expected AC frequency.
     * @param maxFreq The maximum expected AC frequency.
     * @param filterSize The size of the median filter window. MUST BE AN ODD NUMBER (e.g., 3, 5, 7).
     * @return True on success, false on failure.
     */
    bool begin(int zcPin, int triacPin, float minFreq = 45.0, float maxFreq = 65.0, uint8_t filterSize = 5);
    
    /**
     * @brief Must be called repeatedly in the main loop to process frequency data.
     */
    void update();

    /**
     * @brief Sets the power output to the load.
     * @param power The desired power level from 0.0 (off) to 100.0 (full on).
     */
    void setPower(float power);

    /**
     * @brief Sets the known hardware delay of the zero-cross detector.
     * @param delay_us The delay in microseconds (e.g., 750).
     */
    void setMeasurementDelay(unsigned int delay_us);

    /**
     * @brief Enables the TRIAC pulse output. Output is enabled by default after begin().
     */
    void enableOutput();

    /**
     * @brief Disables the TRIAC pulse output immediately for safety.
     */
    void disableOutput();

    // --- Status Functions ---
    bool isEnabled() const;
    bool isFaulty() const;
    float getFrequency() const;
    float getCurrentPower() const;

private:
    // Internal instance of the frequency monitor
    ACFrequencyMonitor _freqMonitor;

    // Pin and state variables
    int _triacPin = -1;
    bool _outputEnabled = false;
    float _powerLevel = 0.0;
    float _firingAngle = 180.0;
    
    // ESP32 hardware timer handles
    esp_timer_handle_t _firingTimer = nullptr;
    esp_timer_handle_t _stopPulseTimer = nullptr;

    // Private helper methods
    float _mapPowerToAngle(float power);

    // Static ISR wrappers required for C-style callbacks
    static void IRAM_ATTR isr_handleTrueZeroCross(void* arg);
    static void IRAM_ATTR isr_fireTriac(void* arg);
    static void IRAM_ATTR isr_stopPulseTrain(void* arg);

    // Member function implementations for ISRs
    void _onTrueZeroCross();
    void _fireTriac();
    void _stopPulseTrain();
};

#endif // TRIAC_CONTROLLER_H