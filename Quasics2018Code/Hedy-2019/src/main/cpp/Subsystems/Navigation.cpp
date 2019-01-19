#include "Navigation.h"
#include "../RobotMap.h"
#include <iostream>

Navigation::Navigation() : frc::Subsystem("Navigation") {


	try {
		/* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
		/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
		/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
		ahrs.reset(new AHRS(SPI::Port::kMXP));
		std::cout << "NavX-MP has been instantiated" << std::endl;
	} catch (const std::exception & ex ) {
		std::string cerr_string = "Error instantiating navX-MXP:  ";
		cerr_string += ex.what();
		DriverStation::ReportError(cerr_string.c_str());
	}
}
