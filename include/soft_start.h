#ifndef SOFT_START_H
#define SOFT_START_H

#include <Arduino.h>

// Triyak soft-start ve zero-cross, frekans ölçüm, PID kontrol görevlerini içerir.

// Global parametreler (extern olarak kullanılacak)
extern volatile float KP;
extern volatile float KI;
extern volatile float KD;
extern volatile float TARGET_VOLTAGE;
extern volatile float frequency;
extern int out_start_type;
extern volatile float resistor_value;
extern volatile bool out_volt_read_err;
extern volatile bool out_cur_read_err;
void startFrequencyTask();

// Dışarıdan çağrılacak init fonksiyonları
/// Triyak soft-start ve zero-cross ISR işlemlerini başlatır
// void initSoftStart();

// /// AC frekans ölçüm görevini başlatır
// void initFreqTask();

// /// PID kontrollü voltaj stabilizasyon görevini başlatır
// void initPIDTask();

#endif // SOFT_START_H
