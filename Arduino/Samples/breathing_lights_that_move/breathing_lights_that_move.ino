void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  breathe(8);
  breathe(9);
  breathe(10);
}

void breathe(uint32_t pin){
  for(int a = 0; a < 255; a++){
    analogWrite(pin, a);
    delay(8);
  }
  for(int a = 255; a > 0; a--){
    analogWrite(pin, a);
    delay(8);
  }  
}
