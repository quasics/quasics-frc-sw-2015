//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with
// Pixy and Arduino.  This program simply prints the detected object blocks
// (including color codes) through the serial console.  It uses the Arduino's
// ICSP port.  For more information go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
//
// It prints the detected blocks once per second because printing all of the
// blocks for all 50 frames per second would overwhelm the Arduino's serial port.
//

#include <SPI.h>
#include <Pixy.h>

// This is the main Pixy object
Pixy pixy;

void setup()
{
  Serial.begin(115200); //Start The Serial port (uart/usb)
  Serial.print("Starting...\n");

  pixy.init(); //initialize the Pixy
}

void loop()
{
  static int i = 0; //loop Counter
  int j; //Block Cycler
  uint16_t blocks; //number of detected blocks
  char buf[32]; //character buffer (32 characters)

  // grab blocks!
  blocks = pixy.getBlocks(); //get the number of blocks detected by the pixy

  // If there are detect blocks, print them!
  if (blocks)
  {
    i++; //Increment 'i'

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i % 50 == 0)
    {
      sprintf(buf, "Detected %d:\n", blocks); //Store "Detected %d:\n" with blocks in for %d, in buf
      Serial.print(buf);  //print the buffer
      for (j = 0; j < blocks; j++) //from j = 0 to j < number of blocks (incrementing j by 1 each time)
      {
        sprintf(buf, "  block %d: ", j); //Store "block %d:" with block number (j) in for %d into buffer (buf)
        Serial.print(buf); //Serial Print buffer (buf)
        pixy.blocks[j].print(); //Print all data on block j
      }
    }
  }
}

