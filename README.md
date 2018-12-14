# Marlin Firmware for B2X300 by BEEVERYCREATIVE

Marlin firmware fork for B2X300 by BEEVERYCREATIVE.

For support, please visit https://beeverycreative.com/forum

---

## Changelog

#### `B2X300-20181214`
- Bugfix - corrected filament change skipping temperature stabilization
- Corrected compiler warnings
- Bed leveling now saves mesh to EEPROM after probing
- Changed EEPROM version and documented address map
- Dual Extrusion Offset Calibration feature
- Bugfix - activated leveling mesh on print restore
- Bugfix - removed unecessary enable steppers on power restore
- Bugfix - corrected dual extrusion restore offset for E2
- M119 now shows the state of the second filament sensor
- Improved float42 to string function
- Improved offset XY layout
- Bugfix - corrected restore print, auto-start when it was not suposed to
- Bugfix - changed print recovered message
- Bugfix - blower values mapped so the fan spins on all the range 0-255

**NEW FEATURES**: 
- Dual Extrusion Offset Calibration 
	- No longer requires diferent Gcodes between machines for dual extrusion
    - You can now save your offset using only your LCD screen
	- No need to save the offset on the slicer


#### `B2X300-20181128`
- Nozzle height now homes after finishing
- After finishing home X will move 20mm to the left
- After the process X carriage will move 20mm to the left
- On change filament temperature will stabilize at the set temperature (B2X300), not 10 degree before(helloBEEprusa)
- Nozzle offset live adjust is more easily acessible and saves to EEPROM
- Powerloss recovery recovers the correct bed temperature
- Powerloss recovery autorecovers print
- Fixed compiler warning caused by SERIAL_ECHO_BIN8

**NEW FEATURES**: 
- Power-loss recovery autorecovers print
    - After a power loss, if the temperature of the heated bed is less than 5 degrees below from the set temperature the print will automatically resume


---


<img align="right" src="buildroot/share/pixmaps/logo/marlin-250.png" />

## Marlin 1.1

Marlin 1.1 represents an evolutionary leap over Marlin 1.0.2. It is the result of over two years of effort by several volunteers around the world who have paid meticulous and sometimes obsessive attention to every detail. For this release we focused on code quality, performance, stability, and overall user experience. Several new features have also been added, many of which require no extra hardware.

For complete Marlin documentation click over to the [Marlin Homepage <marlinfw.org>](http://marlinfw.org/), where you will find in-depth articles, how-to videos, and tutorials on every aspect of Marlin, as the site develops. For release notes, see the [Releases](https://github.com/MarlinFirmware/Marlin/releases) page.


## Marlin Resources

- [Marlin Home Page](http://marlinfw.org/) - The Marlin Documentation Project. Join us!
- [RepRap.org Wiki Page](http://reprap.org/wiki/Marlin) - An overview of Marlin and its role in RepRap.
- [Marlin Firmware Forum](http://forums.reprap.org/list.php?415) - Find help with configuration, get up and running.
- [@MarlinFirmware](https://twitter.com/MarlinFirmware) on Twitter - Follow for news, release alerts, and tips & tricks. (Maintained by [@thinkyhead](https://github.com/thinkyhead).)

## License

Marlin is published under the [GPL license](https://github.com/COPYING.md) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.

