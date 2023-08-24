# Assembly

Required tools:
* Temperature controlled soldering station (e.g., Hakko FX888D, Weller WLC100)
* Hot air rework station
* Precision tweezers
* Silicone work mat
* Fume extractor (recommended)

Several assembly steps for the Power Board require very high heat. A silicone mat may be required to protect your work surface. 
Be careful handling the PCB in between assembly steps.

The general sequence of assembly for the Power Board is according to height. Shorter parts are assembled first.

* SMD parts 🔥
  * R3
  * Q1, Q2, Q5, Q6
  * Q1 to Q2 jumper (optional)
  * Q5 to Q6 jumper (optional)
* Resistors, diodes, thermistor, ceramic capacitors
* RN1
* U7, U6, U4 (optionally, sockets for each)
* D2, D4, D5, D6 (LEDs)
* TO-92 parts
* J12 (12 pin connector)
* J13 (Terminal screw block)
* 12V fan connector
* C7, C1 (electrolytic capactitors)
* Terminal Lugs 🔥
* F1 connectors 🔥
* U2, Q3 (TO-220)
* Heat sink (optional)

## BOM

The Bill-Of-Materials for Digi-Key is located at this address https://www.digikey.com/en/mylists/list/4BDBKJ7A9E. There are two
optional parts in the BOM (heatsink and push-pins) with quantity zero.

Some additional parts will need to be sourced from other vendors.

* Six (6) Terminal Lugs, part number B6A-PCB-RS from [LugsDirect](https://lugsdirect.com/IHI_HIGH_CURRENT_PCB-TERMINALS-_SELECTION_CHART2.htm)
* i2c 128x32 OLED Display, Amazon/eBay/etc. [Example Amazon Listing](https://www.amazon.com/Pieces-Display-Module-SSD1306-3-3V-5V/dp/B08L7QW7SR) 
* 12 pin rectangular ribbon connector [Example Amazon listing](https://www.amazon.com/gp/product/B07FKPXML5)

Optional parts:
* IC sockets [Example Amazon listing](https://www.amazon.com/Glarks-122Pcs-Sockets-Adaptor-Assortment/dp/B01GOLSUAU)
* 12V fan (in BOM)
* Heatsink (in BOM)
* Large thermal pad for heatsink [Example Amazon listing](https://www.amazon.com/gp/product/B09DC772PR)


## DPAK soldering

🔥Hot Parts🔥

![DPAK Mosfets](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/PowerBoardLowRes%20DPAK.jpg)

The four DPAK mosfet are best soldered with a high temperature (750-800F) iron with a large tip. Hot air can be useful as well.

The big pad should be pre-soldered and cleaned with alcohol or other flux remover. The solder will wick through
the thermal vias during pre-soldering, this is normal. After pre-soldering, apply flux to the pad and solder the
two legs. Once the legs are soldered, begin applying heat to the pad and tab. 

The Power Board has a copper plane on the top and bottom, and the mosfet pads are connected through to the back
with thermal vias. It will take some time for the board to heat up sufficiently for solder to flow. Once the 
existing solder starts to flow, apply fresh solder until adequate fillets are present all around the tab.

## Mosfet jumpers

🔥Hot Parts🔥

Two copper jumpers can be installed to improve the ampacity of the Power Board. A gauge is provided on the edge of the
Power Board to help form the jumpers. The jumper holes are 2.6mm which will accommodate 10 AWG wire. If the jumper is
installed, apply enough solder to create a fillet between the jumper and the mosfet leg.

## SMD Resistor (R3) soldering

🔥Hot Parts🔥

![R3](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/PowerBoardLowRes%20R3.jpg)

This resistor is essentially a chunk of copper with specific dimensions to give a good precision small resistance.
For best results, this part should be soldered to the four pads of the footprint as evenly as possible. For best
results, pre-solder a very small amount of solder to the four pads and clean them with IPA or flux remover. Visually
inspect them to see that they are around the same height. Place the SMD resistor on the pads and apply hot air.

## Resistors

Some of the resistors on the power board are high precision (0.1%)

* 49.9 kOhm: R1,R6,R8
* 10 kOhm: R2,R4,R9,R11
* 259 Ohm: R5,R10

These are mainly used as high precision voltage divider for accurate sensing of the temperature, voltage, and current.

The remaining resistors on the power board are pull-ups, pull-downs, and current limiting resistors. These are 5% precision
resistors.

1 kOhm: R15, R17, R24, R27
10 kOhm: R16, R21, R25, R28, R29, R30

## Diodes

* D10
* D1

There are two diodes on the power board. They are different part numbers, so be careful not to mix them up. D10 is a 
general recitfier diode, and D1 is a flyback diode for the fan.

## Thermistor

Install TH1. Try to ensure the part is flush with the board so it can accurately read the temperature of the PCB.

## Ceramic Capacitors

Install the 0.1µF ceramic capators: C2, C12, C13

## Resistor Network RN1

Install RN1. This resistor network is used by the status LEDs.

## Integrated Circuits

There are three ICs on the power board. Two of them (U6 and U7) have the same footprint, but are different parts. IC sockets 
can optionally be installed at this point.

## LEDs

There are four status LEDs on this board. The three green LEDs are for +12V, +5V, and RPi. The yellow LED is for Battery.
The long leg of the resistor goes on the right where the small "+" symbol is printed.

## TO-92 packages

There are four TO-92 parts. Three of them (Q4, Q7, Q8) are the same transistor and one is a precision voltage regulator (U1). 
The three transistors are used to drive power mosfets which are used for 12V switching. The voltage regulator is used
to provide a precise voltage to U4 in order to get accurate current sensing.

## 12V Connectors

Two connectors on the board provide 12V that can be switched off under low power conditions. These are used to power an 
offboard DROK (or other buck converter) used by the RPi and to power an optional fan for cooling. 

![IMG_1827](https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/91f7b736-8dd8-4073-9ee5-28d2dc243e90)

## Electrolytic Capacitors

Install C7 and C1.

## Terminal Lugs

🔥Hot Parts🔥

The six terminal lugs are designed to be wave soldered, but hand soldering is also possible. These parts require
the most heat of any in the assembly, so take care when handling the PCB during this step. Since these lugs are so
large, it is necessary to place some supporting material under the PCB in order to balance it and keep it level. Scrap
silicone work mat works nicely for this
 
![Terminal Lug Assembly Top](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/TerminalLugAssemblyTop.jpg)

![Terminal Lug Assembly Front](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/TerminalLugAssemblyFront.jpg)

Remove the bolts from the lugs and place one of them upside down on the work surface. Balance the PCB on top of it using
some supporting material as pictured above. Heat the soldering iron to 800-850F and apply some solder to the tip. 
Begin heating the terminal lug. This will take some time. Once the solder starts to transfer from the iron tip to the 
lug surface, begin heating the pad on the PCB. The terminal lug has very high thermal mass and will
stay hot for a long time after pre-heating. Once solder begins to flow on the pad, apply around 30mm of solder (roughly
three times the length of the pad). Drag the solder around the all edges where the terminal contacts the PCB. There should
be enough solder to form a nice fillet along the two long edges of the terminal and to fill the gaps on either end.

Repeat the process for the remaining 5 lugs.

Once the terminals and PCB have cooled enough to handle, clean the pads with IPA or flux cleaner. 

Here are some close-ups showing an acceptable solder joint for a terminal lug:

<img alt="Terminal Lug Close-up 1" src="https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/Lug1.jpg" width="400"/>
<img alt="Terminal Lug Close-up 2" src="https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/Lug2.jpg" width="400"/>
<img alt="Terminal Lug Close-up 3" src="https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/Lug3.jpg" width="400"/>
<img alt="Terminal Lug Close-up 4" src="https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/Lug4.jpg" width="400"/>

## Fuse (F1)

🔥Hot Parts🔥

The F1 fuse footprint accepts two quick connect terminals. Insert the terminals on the top of the PCB according to the
silkscreen and tack them into place from the top. This will secure the terminals well enough to fully solder from the bottom.
When soldering, start on the pin that wasn't tacked from the top.  

## TO-220 parts

There are two TO-220 parts which look the same, but are different parts. U2 is a voltage regular and Q3 is a power mosfet. 
Take care to not mix them up, also take care to install them in the correct orientation.

![IMG_1828](https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/e007351a-79aa-4d0b-9121-228a976ff920)

## Heatsink

To accommodate high current installations, the Power Board has mounting holes for a 50mm square heatsink. 
The hole pattern is 43mm by 43mm on center.

![Power Board Back](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/PowerBoardBackLowRes.jpg)

This mounting pattern will accommodate any of the 50x50mm pushPIN™ Heat Sink series from Advanced Thermal Solutions, Inc.
One such heatsink is included in the BOM (with a quantity of zero). If ordering a heat sink, be sure to also order two
units of the push pins. Each unit of push pins includes two pins, so two units gives the requisite four push pins.

Since the back of the PCB has exposed copper, a thermal pad is required to avoid shorting the mosfets with the 
heat sink. Also, since the heatsink area includes many through-hole pads, the thermal pad must be thick enough
to keep the heatsink flat. A pad of at least 1mm thickness is needed. The leads of the through-hole parts in the
heatsink area should be trimmed flush with the solder fillet (not flush with the board). Trimming the leads will
avoid having their sharp ends exposed which could pierce the thermal pad and contact the heatsink.









