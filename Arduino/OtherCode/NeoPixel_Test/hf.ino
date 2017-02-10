void SetStripColor (uint32_t red, uint32_t green, uint32_t blue, uint32_t white){
  for(uint32_t pixel = 0; pixel <= lastPixel; pixel++){
    uint32_t combined = strip.Color(red, green, blue, white);
    strip.setPixelColor(pixel, combined);
  }
}

void SetStripColor (float h, float s, float v){
  double r = 0;
  double g = 0;
  double b = 0;
  
  int i = int(h * 6);
  double f = h * 6 - i;
  double p = v * (1 - s);
  double q = v * (1 - f * s);
  double t = v * (1 - (1 - f) * s);

  switch (i % 6) {
  case 0:
    r = v; g = t; b = p;
    break;
  case 1:
    r = q; g = v; b = p;
    break;
  case 2:
    r = p; g = v; b = t;
    break;
  case 3:
    r = p; g = q; b = v;
    break;
  case 4:
    r = t; g = p; b = v;
    break;
  case 5:
    r = v; g = p; b = q;
    break;
  }
  for(uint32_t pixel = 0; pixel <= lastPixel; pixel++){
    uint32_t combined = strip.Color(r * 255, g* 255, b* 255, 0);
    strip.setPixelColor(pixel, combined);
  }
}

void SetRangeColor (uint32_t startPixel, uint32_t endPixel, uint32_t red, uint32_t green, uint32_t blue, uint32_t white){
  if ((endPixel<= lastPixel)){
    for(uint32_t pixel = startPixel; pixel <= endPixel; pixel++){
     uint32_t combined = strip.Color(red, green, blue, white);
     strip.setPixelColor(pixel, combined);
    }
  } else {
    for(uint32_t pixel = startPixel; pixel <= lastPixel; pixel++){
     uint32_t combined = strip.Color(red, green, blue, white);
     strip.setPixelColor(pixel, combined);
    }
    for(uint32_t pixel = 0; pixel <= endPixel%lastPixel - 1; pixel++){
     uint32_t combined = strip.Color(red, green, blue, white);
     strip.setPixelColor(pixel, combined);
    }
  }
}

void SetRangeColor (uint32_t startPixel, uint32_t endPixel, float h, float s, float v){

  double r = 0;
  double g = 0;
  double b = 0;
  
  int i = int(h * 6);
  double f = h * 6 - i;
  double p = v * (1 - s);
  double q = v * (1 - f * s);
  double t = v * (1 - (1 - f) * s);

  switch (i % 6) {
  case 0:
    r = v; g = t; b = p;
    break;
  case 1:
    r = q; g = v; b = p;
    break;
  case 2:
    r = p; g = v; b = t;
    break;
  case 3:
    r = p; g = q; b = v;
    break;
  case 4:
    r = t; g = p; b = v;
    break;
  case 5:
    r = v; g = p; b = q;
    break;
  }
  
  if ((endPixel<= lastPixel)){
    for(uint32_t pixel = startPixel; pixel <= endPixel; pixel++){
     uint32_t combined = strip.Color(r* 255, g* 255, b* 255, 0);
     strip.setPixelColor(pixel, combined);
    }
  } else {
    for(uint32_t pixel = startPixel; pixel <= lastPixel; pixel++){
     uint32_t combined = strip.Color(r* 255, g* 255, b* 255, 0);
     strip.setPixelColor(pixel, combined);
    }
    for(uint32_t pixel = 0; pixel <= endPixel%lastPixel - 1; pixel++){
     uint32_t combined = strip.Color(r* 255, g* 255, b* 255, 0);
     strip.setPixelColor(pixel, combined);
    }
  }
}
