struct ColorDef {
  ColorDef(const char * name, unsigned char red, unsigned char green, unsigned char blue)
    : name_(name), red_(red), green_(green), blue_(blue) {}
 
  const char * const name_;
  unsigned char red_;
  unsigned char green_;
  unsigned char blue_;
};

/*************************************************************************/
// Output (PWM) pin definitions
const int redPin = 11;    // R petal on RGB LED module connected to digital pin 11
const int greenPin = 10;  // G petal on RGB LED module connected to digital pin 9
const int bluePin = 9;    // B petal on RGB LED module connected to digital pin 10

// Color definitions (i.e., what we'll cycle through).
// Note that these could also have been defined inside of "getNextColor()".
static const ColorDef cycle[] = {
  ColorDef("red", 255, 0, 0),
  ColorDef("orange", 237, 109, 0),
  ColorDef("yellow", 255, 215, 0),
  ColorDef("green", 34, 139, 34),
  ColorDef("blue", 0, 0, 255),
  ColorDef("indigo", 0, 46, 90),
  ColorDef("violet", 128, 0, 128),
  ColorDef("off", 0, 0, 0),
  ColorDef("white", 255, 255, 255),
  ColorDef("off", 0, 0, 0),
};
const unsigned int CycleArraySize = sizeof(cycle) / sizeof(cycle[0]);

/**************************************************************************/     
const ColorDef& getNextColor() {
  static unsigned int curColor = CycleArraySize - 1;
  curColor = (curColor + 1) % CycleArraySize;
  return cycle[curColor];
}

void setColor (const ColorDef& color) {
  Serial.print("Setting color to ");
  Serial.println(color.name_);
  analogWrite(redPin, color.red_);  
  analogWrite(bluePin, color.blue_);
  analogWrite(greenPin, color.green_);
}
void setup()
{
  Serial.begin(115200);

  pinMode(redPin, OUTPUT); // sets the redPin to be an output
  pinMode(greenPin, OUTPUT); // sets the greenPin to be an output
  pinMode(bluePin, OUTPUT); // sets the bluePin to be an output
}

/***************************************************************************/
void loop()  // run over and over again 
{
  setColor(getNextColor());
  delay(1000);
}    
/******************************************************/
