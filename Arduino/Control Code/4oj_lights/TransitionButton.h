#ifndef TRANSITION_BUTTON_H
#define TRANSITION_BUTTON_H

/** 
 * Defines a simple class that will monitor a push-button, and can be used to
 * decide when it's been pressed and released.  This is meant to allow the
 * button to be used to move through a sequence of some sort (e.g., from one
 * light pattern to another, to another, etc.).
 * 
 * Based on https://www.arduino.cc/en/Tutorial/Debounce
 * 
 * @author mjhealy@alum.rit.edu
 */
class TransitionButton {
  private:
    const int buttonPin;
    const bool transitionOnPress;
    const unsigned long debounceDelayMsec;

    int lastButtonState = LOW;           // the previous (transient) reading from the input pin
    int buttonState = LOW;               // the last (stable) reading from the input pin
    unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  
  public:
    /**
     * @param buttonPin         the pin being monitored for button presses
     * @param transitionOnPress if true, advance on button being pressed down; otherwise, on button release
     * @param debounceDelayMsec delay used to make sure that the button is in a stable state
     */
    TransitionButton(int buttonPin, bool transitionOnPress = true, unsigned long debounceDelayMsec = 50)
      : buttonPin(buttonPin), transitionOnPress(transitionOnPress), debounceDelayMsec(debounceDelayMsec)
    {
      pinMode(buttonPin, INPUT);
    }

    bool advancePosition() {
      const unsigned long now = millis();
      int reading = digitalRead(buttonPin);
      
      if (reading != lastButtonState) {
        lastDebounceTime = now;
        lastButtonState = reading;
      }
    
      if ((now - lastDebounceTime) > debounceDelayMsec) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state.
        if (reading != buttonState) {
          buttonState = reading;
    
          // We'll assume that we transition states on button down.
          return (buttonState == transitionOnPress);
        }
      }
    
      return false;
    }
};

#endif  // TRANSITION_BUTTON_H

