/*
 * Navigation.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Developer
 */

#include "Navigation.h"
#include <iostream>

Navigation::Navigation()
: Subsystem("Navigation	")
{
	try {
		/* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
		/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
		/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
		ahrs.reset(new AHRS(SPI::Port::kMXP));
		std::cout << "NavX-MP has been instantiated" << std::endl;
	} catch (const std::exception & ex ) {
		std::string err_string = "Error instantiating navX-MXP:  ";
		err_string += ex.what();
		DriverStation::ReportError(err_string.c_str());
	}
}

