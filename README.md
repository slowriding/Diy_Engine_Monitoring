# Diy_Engine_Monitoring
Read and display Powertrain sensors and control outputs with DIY PCB, code and display.

Project started to facilitate features lost due to swapping in newer engine into older computer controlled vehicle.  
It overlays a secondary set of sensors and display unit showing gauges.  The controller drive outputs for variable speed loads like fans and fuel pump and switched loads.
Goals are for the system to be as simple as possible, hackable and start immediately without having to boot an operating system.

Current design uses a common Arduino module programmed in C.  This project welcomes submissions of improvements or complete redesigns.
OBDII and GM LAN integration are intended but not completed.

Test article implementation:
2002 Chevrolet Tahoe with 5.3L gas engine failed and Chevrolet Performance E-ROD LS3 powertrain was installed with 6-speed manual transmission necessitating ECU swap.  The communications network is too different to interconnect directly.  The original dashboard gauges and A/C became completely non-functional due to the communications bus changes and features not implemented in newer but generic engine controller.

This project side steps difficulties in attempting to build a protocol converter; for the time being.  Requirements are met by adding a layer of sensors and outputs to replace missing features and adding some extra functionality.  I (original contributor) intend to hack my 2002 Tahoe dash to driving of stock gauge needles.  This will also control new style variable A/C compressor based on pressure and temperature sensors. 

Design overview:
Sensors types:  // currently implemented but not all yet working
	Resistive (coolant & oil temp and fuel level).
	0-5V transducers (oil, fuel, evap. and A/C high pressure)
	0-5V ECU outputs (throttle and map)
	Pulse frequency (RPM and vehicle speed)
	Switched (ac & recirculation requests)
	1-wire temperature probes (outside air and A/C condenser temperatures)
 
Outputs types:
	PWM Gauges Drivers (fuel level, oil pressure, coolant temperature, MPH)
	PWM Loads (fan1, fan2, Variable A/C compressor, Fuel Pump speed)

Interface:
	Display - Nextion brand Intelligent LCD display used due to it’s simple serial port/text based connection.  All the graphics are stored and processed on the display. 
	I2c & SPI connections for future ODB2 interfaces.
Programming:
	Currently written in C with migration to RUST intended for future releases.
	The Tahoe.h file isolates MCU pin definitions, calibrations and variable definitions.
	

Please consider contributing to this evolving project as much can be improved
