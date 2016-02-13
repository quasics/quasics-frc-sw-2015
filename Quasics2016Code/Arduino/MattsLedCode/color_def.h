#ifndef COLOR_DEF_H
#define COLOR_DEF_H

struct ColorDef {
  ColorDef(const char * name, unsigned char red, unsigned char green, unsigned char blue)
    : name_(name), red_(red), green_(green), blue_(blue) {}
 
  const char * const name_;
  const unsigned char red_;
  const unsigned char green_;
  const unsigned char blue_;

  static const ColorDef OFF;
  static const ColorDef WHITE;
  static const ColorDef RED;
  static const ColorDef GREEN;
  static const ColorDef BLUE;
  static const ColorDef YELLOW;
  static const ColorDef ORANGE;
  static const ColorDef INDIGO;
  static const ColorDef VIOLET;
};

#endif  // COLOR_DEF_H

