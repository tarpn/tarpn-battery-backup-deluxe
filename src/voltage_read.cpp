#include <Arduino.h>
#include "voltage_read.h"

VoltageRead::VoltageRead(float * reference_voltage, float * factor) {
  this->reference_voltage = reference_voltage;
  this->factor = factor;
  this->offset = NULL;
}

VoltageRead::VoltageRead(float * reference_voltage, float * factor, float * offset) {
  this->reference_voltage = reference_voltage;
  this->factor = factor;
  this->offset = offset;
}

float get_offset(float * ptr) {
  if (ptr == NULL) {
    return 0.0;
  } else {
    return *ptr;
  }
}

double VoltageRead::adcVoltageSlow() {
  return this->filter->output * *(this->reference_voltage) / 1024.0;
}

double VoltageRead::smoothedValueSlow() {
  double voltage = this->adcVoltageSlow();
  return (voltage - get_offset(this->offset)) / *(this->factor);  
}

double VoltageRead::adcVoltageFast() {
  return this->fastFilter->output * *(this->reference_voltage) / 1024.0;
}

double VoltageRead::smoothedValueFast() {
  double voltage = this->adcVoltageFast();
  return (voltage - get_offset(this->offset)) / *(this->factor);  
}

double VoltageRead::instantValue() {
  double voltage = this->sensorValue * *(this->reference_voltage) / 1024.0;
  return (voltage - get_offset(this->offset)) / *(this->factor);  
}