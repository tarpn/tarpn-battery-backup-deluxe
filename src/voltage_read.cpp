#include "voltage_read.h"

VoltageRead::VoltageRead(double reference_voltage, double factor) {
  this->reference_voltage = reference_voltage;
  this->factor = factor;
  this->offset = 0.0;
}

VoltageRead::VoltageRead(double reference_voltage, double factor, double offset) {
  this->reference_voltage = reference_voltage;
  this->factor = factor;
  this->offset = offset;
}

double VoltageRead::adcVoltageSlow() {
  return this->filter->output * this->reference_voltage / 1024.0;
}

double VoltageRead::smoothedValueSlow() {
  double voltage = this->adcVoltageSlow();
  return (voltage - this->offset) / this->factor;  
}

double VoltageRead::adcVoltageFast() {
  return this->fastFilter->output * this->reference_voltage / 1024.0;
}

double VoltageRead::smoothedValueFast() {
  double voltage = this->adcVoltageFast();
  return (voltage - this->offset) / this->factor;  
}

double VoltageRead::instantValue() {
  double voltage = this->sensorValue * this->reference_voltage / 1024.0;
  return (voltage - this->offset) / this->factor;  
}