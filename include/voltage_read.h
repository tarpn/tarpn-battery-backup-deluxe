#ifndef VOLTAGE_READ_H
#define VOLTAGE_READ_H

#include <stdint.h>
#include <Ewma.h>

struct VoltageRead {
  double factor;                      // A factor to convert a voltage to a value
  double offset;                      // An offset to subtract after scaling by the factor
  uint16_t sensorValue;               // Raw sensor data
  double reference_voltage;           // The ADC reference voltage
  Ewma * filter = new Ewma(0.05);     // Slow moving average filter
  Ewma * fastFilter = new Ewma(0.5);  // Fast moving average filter

  /**
   * Initialize the voltage read object with the reference voltage and the voltage divider factor.
   * 
   * @param referernce_voltage
   * @param voltage_factor
  */
  VoltageRead(double, double);

  /**
   * Initialize the voltage read object with the reference voltage, the voltage divider factor, and 
   * a voltage offset
   * 
   * @param referernce_voltage
   * @param voltage_factor
   * @param voltage_offset
  */
  VoltageRead(double, double, double);
  
  double adcVoltageSlow();        // The real voltage being read at the ADC input (slow filter)
  double adcVoltageFast();        // The real voltage being read at the ADC input (fast filter)
  double smoothedValueSlow();     // The scaled and offset voltage being detected (slow filter)
  double smoothedValueFast();     // The scaled and offset voltage being detected (fast filter)
  double instantValue();          // The instantaneous scaled and offset value (no filter)
};
#endif