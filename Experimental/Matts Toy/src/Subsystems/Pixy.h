/*
 * Pixy.h
 *
 *  Created on: Feb 4, 2017
 *      Author: healym
 */

#ifndef SRC_SUBSYSTEMS_PIXY_H_
#define SRC_SUBSYSTEMS_PIXY_H_

#include <WPILib.h>
#include "../ThirdParty/pixy/PixyI2C.h"

class Pixy: public frc::Subsystem {
private:
	PixyI2C pixy;

public:
	Pixy();
	virtual ~Pixy();

	void getBlocks(std::vector<Block>& blocks);
};

#endif /* SRC_SUBSYSTEMS_PIXY_H_ */
