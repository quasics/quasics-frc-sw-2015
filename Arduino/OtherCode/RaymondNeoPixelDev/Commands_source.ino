void SetRangeRGB (uint32_t startPixel, uint32_t endPixel, uint32_t red, uint32_t green, uint32_t blue) {
  for (uint32_t pixel = startPixel; pixel <= endPixel; pixel++) {
    strip.setPixelColor (pixel, red, green, blue);
  }
}

void SetRangeHSV(uint32_t startPixel, uint32_t endPixel, uint32_t hue, uint32_t saturation, uint32_t value) {
  byte rgb[3];
  rgbConverter.hsvToRgb(hue, saturation, value, rgb);
  for (uint32_t pixel = startPixel; pixel <= endPixel; pixel++) {
    strip.setPixelColor (pixel, rgb[1], rgb[2], rgb[3]);
  }
}
