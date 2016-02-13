#ifndef LIGHT_MGMT_H
#define LIGHT_MGMT_H

class LightManager {
public:
  enum Mode {
    eRedAlliance = 0x01,
    eBlueAlliance = 0x02,
    eDemoMode = 0x03,
    eErrorMode = 0xFF,
    eUndefinedMode = 0xFEED,
  };

  enum State {
    eDisabled = 0x00,
    eAuto = 0x01,
    eTeleOp = 0x02,
    eTest = 0x03,
  };

  LightManager() : currentMode_(eDemoMode) {}
  virtual ~LightManager() {}
  
  void setMode(Mode mode);
  void setState(State state);

  void updateLights();

private:
  /** Determines the color that should be displayed for the specified operating mode. */
  static const ColorDef& getColorForMode(Mode m);

  /** Determines the lighting intensity (0-100%), based on the current mode/state. */
  int calculateIntensity() const;

// May be overridden by derived classes.
protected:
  /** Actually "applies" the specified color/light intensity (%). */
  virtual void setLights(const ColorDef& color, int intensityPercent) = 0;
  
  /** Lets us change the timing in the algorithm, based on the actual controller type being used. */
  virtual unsigned long getClockSkew() const { return 1; }

private:
  Mode  currentMode_;   ///< Current operating mode.
  State currentState_;  ///< Current state (ignored in "error" and "undefined" mode).
};

/** 
 * A class that can be used to test the logic for lighting definitions, without needing
 * to drive actual LED hardware. 
 */
class DummyLightManager : public LightManager {
public:
  DummyLightManager();

protected:
  /** 
   * Writes new light settings to the console (Serial).
   * 
   * Note: If the difference is simply the intensity %, then we'll only report when
   *       if the (absolute) delta is more than 25%.
   */
  virtual void setLights(const ColorDef& color, int intensityPercent);
  virtual unsigned long getClockSkew() const { return 5; }

private:
  const ColorDef* lastColor_;
  int lastIntensity_;
};

/** 
 * A version of the LightManager class that can be used to drive an RGB LED (or strip)
 * using PWM.
 */
class PwmLightManager : public LightManager {
public:
  PwmLightManager(int redPin, int greenPin, int bluePin) :
    redPin_(redPin), greenPin_(greenPin), bluePin_(bluePin) {}

protected:
  // See base class.
  virtual void setLights(const ColorDef& color, int intensityPercent);

private:
  int redPin_;      ///< Pin used to drive red component.
  int greenPin_;    ///< Pin used to drive green component.
  int bluePin_;     ///< Pin used to drive blue component.
};

#endif  // LIGHT_MGMT_H

