#ifndef METRICS_H
#define METRICS_H

typedef struct {
  /*
  Amp-seconds are integrated continuously by sampling the ADC and multiplying by the sample time (in seconds). 
  This gives a measurement of the total charge seen across the load resistor.
  */
  unsigned int adc_samples;
  unsigned int period_ms;

  double load_amp_sec;
  float load_amp_sec_delta;
  float load_amp_avg;
  float load_amp_max;
  float load_amp_min;

  double watt_sec;  // total
  float watt_sec_delta;
  float watt_avg;

  double battery_amp_sec;
  float battery_amp_sec_delta;
  float battery_amp_avg;
  float battery_watt_sec_delta;
  double battery_watt_sec;  // total
} MetricsReport;

typedef struct {
  // Last reported data
  unsigned long last_report_ms;
  MetricsReport * report;

  // Accumulators for current reporting interval
  unsigned int adc_samples;
  double current_amp_seconds;
  double battery_amp_seconds;
  double current_watt_seconds;
  double battery_watt_seconds_acc;
} Metrics;

void Metrics_Update(Metrics * metrics, double elapsed_sec);

void Metrics_Report(Metrics * metrics, unsigned long now);
#endif