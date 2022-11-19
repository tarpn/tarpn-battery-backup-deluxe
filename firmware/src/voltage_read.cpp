#include <Arduino.h>
#include "voltage_read.h"


void Sensor_Init(SensorRead * sensor, float * reference_voltage, float * factor, float * offset, double w) {
  sensor->reference_voltage = reference_voltage;
  sensor->factor = factor;
  sensor->offset = offset;
  sensor->filter = 0.0;
  sensor->first_sample = true;
  sensor->w = w;
  // Tau is amount of time for a unit step function to reach 63.2%
  // W = EXP(-1 * DT / Tau)
  // Clock 50Hz, 10Hz sample rate (5 ADCs). 0.1sec, DT = 100000, Tau = 390000, w = 0.7738
}

void Sensor_Update(SensorRead * sensor, uint16_t value) {
  if (sensor->first_sample) {
    sensor->filter = value;
    sensor->maxValue = value;
    sensor->minValue = value;
    sensor->first_sample = false;
  } else {
    
    //double dt = (double)(sample_time_us - sensor->time_us);
    //double x = dt / (double) sensor->tau;
    //double w = exp(-1.0 * x);
    sensor->filter = sensor->filter * sensor->w + (1.0 * value) * (1.0 - sensor->w);
  }
  sensor->sensorValue = value;
  sensor->maxValue = max(sensor->maxValue, value);
  sensor->minValue = min(sensor->minValue, value);
}

double _sensor_voltage(SensorRead * sensor, uint16_t val) {
  return val * *(sensor->reference_voltage) / 1024.0;
}

double Sensor_offsetAndScaleValue(SensorRead * sensor, uint16_t val) {
  double voltage = _sensor_voltage(sensor, val);
  if (sensor->offset == NULL) {
    return voltage / *(sensor->factor);  
  } else {
    return (voltage - *(sensor->offset)) / *(sensor->factor);  
  }
}

double Sensor_adcVoltageFast(SensorRead * sensor) {
  return _sensor_voltage(sensor, sensor->filter);
  //return sensor->filter * *(sensor->reference_voltage) / 1024.0;
}

double Sensor_smoothedValueFast(SensorRead * sensor) {
  return Sensor_offsetAndScaleValue(sensor, sensor->filter);
  //double voltage = Sensor_adcVoltageFast(sensor);
  //return (voltage - get_offset(sensor->offset)) / *(sensor->factor);  
}

double Sensor_instantValue(SensorRead * sensor) {
  return Sensor_offsetAndScaleValue(sensor, sensor->sensorValue);
  //double voltage = sensor->sensorValue * *(sensor->reference_voltage) / 1024.0;
  //return (voltage - get_offset(sensor->offset)) / *(sensor->factor);  
}

void Sensor_resetMinMax(SensorRead * sensor) {
  sensor->maxValue = 0;
  sensor->minValue = 1023;
}
