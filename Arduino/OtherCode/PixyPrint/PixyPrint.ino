#include <SerialInterface.h>
#include <SPI.h>
#include <Pixy.h>

const uint32_t numberOfLoops = 50;
const uint32_t targetSignature = 1;

// This is the main Pixy object
Pixy pixy;
SerialInterface * serialInterface;

void setup() {
  serialInterface = new SerialInterface ();
  pixy.init(); //initialize the Pixy
}

void loop() {
  static int i = 0; //loop Counter
  int j; //Block Cycler
  uint16_t blocks; //number of detected blocks
  char buf[32]; //character buffer (32 characters)

  //grab blocks
  blocks = pixy.getBlocks();

  //if there are blocks...
  if (blocks) {
    i++;
    if (i % numberOfLoops == 0) {
      uint32_t targetBlocks = 0;
      for (j = 0; j < blocks; j++) //from j = 0 to j < number of blocks (incrementing j by 1 each time)
      {
        if (pixy.blocks[j].signature == targetSignature) {
          targetBlocks++;
          sprintf(buf, "Targeted Block %d:\n", targetBlocks);
          serialInterface->SerialWrite(buf);
          sprintf(buf, "  Coordinates: ( %d , %d )\n", pixy.blocks[j].x, pixy.blocks[j].y);
          serialInterface->SerialWrite(buf);
          sprintf(buf, "  Height: %d\n", pixy.blocks[j].height);
          serialInterface->SerialWrite(buf);
          sprintf(buf, "  Width: %d\n", pixy.blocks[j].width);
          serialInterface->SerialWrite(buf);
          serialInterface->SerialWrite("\n \n");
        }
      }
    }
  }
}
