/*
 * Pixy.cpp
 *
 *  Created on: Feb 4, 2017
 *      Author: healym
 */

#include "Pixy.h"
#include <algorithm>
#include <iterator>

Pixy::Pixy() : Subsystem("Pixy") {
	// TODO Auto-generated constructor stub

}

Pixy::~Pixy() {
	// TODO Auto-generated destructor stub
}

void Pixy::getBlocks(std::vector<Block>& blocks) {
	const uint16_t count = pixy.GetBlocks();
	std::vector<Block> results;
	std::copy(pixy.blocks, pixy.blocks + count, std::back_insert_iterator<std::vector<Block>>(results));
	blocks.swap(results);
}
