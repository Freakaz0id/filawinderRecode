# filawinderRecode

This is a recreation of the filawinder firmware by Ian Johnson. The documentation for the project can be found here:
http://www.soliforum.com/topic/5088/filawinder-documentation/
Print files and BOM are published on thingiverse: https://www.thingiverse.com/thing:174383

## Setup

* Setup the electronics according to the schematic on thingiverse. 
* The mechanical setup is descibed on the documentation page.
* Upload the firmware using Arduino IDE (Tested on Arduino Nano)

## Usage

The Arduino serial monitor can be used for trouble shooting the electronics. Set the corresponding debugging variables to true/false to do so.
The winder can be configured and controlled using the three push buttons and switches.

* Turn the potentiometer down all the way
* Set the mode switch to "manual"
* Turn on the power switch
* Hold min-button and turn potentiometer till the guide is at the left limit
* Hold max-button and turn potentiometer till the guide is at the right limit
* Hold set-button and turn potentiometer till the guide is at the "current" position. Movement direction will be stored as well
* Hold min- and max-buttons for more than 0.5s to trigger the calibration of the QTR-sensor
* Move a piece of filament up and down between line laser and sensor to read min/max values. Calibration will take about 5s and the on-board LED will blink, when calibration is finished.
* Setup the filament and start winding in manual mode
* After finding a rather stable speed, switch to auto. The PID should take over.

## Issues
PID does not work to well and sometimes does not adjust fast enough. Considering changing to optical enstops instead for a more robust system.
