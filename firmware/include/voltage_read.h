#ifndef VOLTAGE_READ_H
#define VOLTAGE_READ_H

#include <stdint.h>

typedef struct {
  float * factor;                     // A factor to convert a voltage to a value
  float * offset;                     // An offset to subtract after scaling by the factor
  float * reference_voltage;          // The ADC reference voltage
  
  double filter;
  double w;
  bool first_sample;

  uint16_t sensorValue;               // Raw sensor data   
  uint16_t maxValue;                  // Max sensor data seen, no filtering   
  uint16_t minValue;                  // Min sensor data seen, no filtering  
} SensorRead;

void Sensor_Init(SensorRead * sensor, float * referernce_voltage, float * voltage_factor, float * voltage_offset, double w);

void Sensor_Update(SensorRead * sensor, uint16_t value);

 // The real voltage being read at the ADC input (fast filter)
double Sensor_adcVoltageFast(SensorRead * sensor);

 // The scaled and offset voltage being detected (fast filter)
double Sensor_smoothedValueFast(SensorRead * sensor);

// The instantaneous scaled and offset value (no filter)
double Sensor_instantValue(SensorRead * sensor);

// The scaled and offset max value read so far. Calling this resets the max to 0
void Sensor_resetMinMax(SensorRead * sensor);

double Sensor_offsetAndScaleValue(SensorRead * sensor, uint16_t val);

#endif