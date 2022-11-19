Supply Voltage
Battery Voltage
Battery Current
Board Temperature



```
 -[B]-[P]-[L]-[G]-[G]-[G]-
|                         |
|                         |
|                         |
 -------------------------
```

Terminals:

* B: Battery Positive
* P: PSU Positive
* L: Load Positive
* G: Common Grounds

A 13.5V nominal power supply is attached to terminal P. In NORMAL operation, this directly powers the loads through a trace on the PCB. When power is lost on P, the controller detects a low voltage condition and closes the solid state switch which connects the battery positive B to the loads L.



Ham radio PSUs supply 13.5V, 