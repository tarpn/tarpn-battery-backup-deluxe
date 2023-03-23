# Assembly

Required tools:
* Temperature controlled soldering station (e.g., Hakko FX888D)
* Hot air rework station
* Precision tweezers
* Silicone work mat
* Fume extractor (recommended)

Several assembly steps for the Power Board require very high heat. A silicone mat is needed to protect your work surface. 
Be careful handling the PCB in between assembly steps.

The general sequence of assembly for the Power Board is:

* SMD parts
  * R3
  * Q1, Q2, Q5, Q6
  * Q1 to Q2 jumper (optional)
  * Q5 to Q6 jumper (optional)
* Terminal Lugs
* F1 connectors
* Resistors
* Ceramic capacitors
* TO-92 parts
* TO-220 parts
* Electrolytic capacitors
* Heat sink (optional)

## BOM

The Bill-Of-Materials for Digi-Key is located at this address https://www.digikey.com/en/mylists/list/4BDBKJ7A9E. Two additional
parts will need to be sourced from other vendors.

* Six (6) Terminal Lugs, part number B6A-PCB-RS from [LugsDirect](https://lugsdirect.com/IHI_HIGH_CURRENT_PCB-TERMINALS-_SELECTION_CHART2.htm)
* i2c 128x32 OLED Display, Amazon/eBay/etc. [Example Amazon Listing](https://www.amazon.com/Pieces-Display-Module-SSD1306-3-3V-5V/dp/B08L7QW7SR) 
* Large thermal pad for heatsink [Example Amazon listing](https://www.amazon.com/gp/product/B09DC772PR)

## DPAK soldering

![DPAK Mosfets](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/PowerBoardLowRes%20DPAK.jpg)

The four DPAK mosfet are best soldered with a high temperature (750-800F) iron with a large tip. Hot air can be useful as well.

The big pad should be pre-soldered and cleaned with alcohol or other flux remover. The solder will wick through
the thermal vias during pre-soldering, this is normal. After pre-soldering, apply flux to the pad and solder the
two legs. Once the legs are soldered, begin applying heat to the pad and tab. 

The Power Board has a copper plane on the top and bottom, and the mosfet pads are connected through to the back
with thermal vias. It will take some time for the board to heat up sufficiently for solder to flow. Once the 
existing solder starts to flow, apply fresh solder until adequate fillets are present all around the tab.

## Mosfet jumpers

Two copper jumpers can be installed to improve the ampacity of the Power Board. A gauge is provided on the edge of the
Power Board to help form the jumpers. The jumper holes are 2.6mm which will accommodate 10 AWG wire. If the jumper is
installed, apply enough solder to create a fillet between the jumper and the mosfet leg.

## SMD Resistor (R3) soldering

![R3](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/PowerBoardLowRes%20R3.jpg)

This resistor is essentially a chunk of copper with specific dimensions to give a good precision small resistance.
For best results, this part should be soldered to the four pads of the footprint as evenly as possible. For best
results, pre-solder a very small amount of solder to the four pads and clean them with IPA or flux remover. Visually
inspect them to see that they are around the same height. Place the SMD resistor on the pads and apply hot air.

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









