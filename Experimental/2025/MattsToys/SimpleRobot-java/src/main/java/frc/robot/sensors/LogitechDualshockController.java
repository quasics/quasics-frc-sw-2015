// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from Logitech controllers in "Dualshock" mode, connected to the Driver Station.
 *
 * This class handles controller input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 *
 * Note: these values assume that the switch on the bottom of the Logitech
 * controller is in the "D" position, causing it to enumerate as a Logitech
 * Dualshock controller. In this mode, the right joystick X/Y axes are 2 and 3,
 * respectively, and the left and right triggers show up as *buttons* 7 and
 * 8.
 *
 * Note: this code is untested.
 */
public class LogitechDualshockController extends GenericHID {
  /** Represents an axis on an LogitechF310Controller. */
  public enum Axis {
    /** Left joystick X axis. */
    LeftX(0),
    /** Left joystick Y axis. */
    LeftY(1),
    /** Right joystick X axis. */
    RightX(2),
    /** Right joystick Y axis. */
    RightY(3);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    @Override
    public String toString() {
      return this.name() + "Axis";
    }
  }

  /** Represents a button on an LogitechF310Controller. */
  public enum Button {
    /** ID for the "X" button. */
    X(1),
    /** ID for the "A" button. */
    A(2),
    /** ID for the "B" button. */
    B(3),
    /** ID for the "Y" button. */
    Y(4),
    /** ID for the left shoulder button. */
    LeftShoulder(5),
    /** ID for the right shoulder button. */
    RightShoulder(6),
    /** Left trigger's X axis. */
    LeftTrigger(7),
    /** Right trigger's X axis. */
    RightTrigger(8),
    /** ID for the back button. */
    Back(9),
    /** ID for the start button. */
    Start(10),
    /** ID for the button clicked by pressing on the left joystick. */
    LeftStick(11),
    /** ID for the button clicked by pressing on the right joystick. */
    RightStick(12);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    @Override
    public String toString() {
      return this.name() + "Button";
    }
  }

  /**
   * Construct an instance of a controller.
   * @param port The port index on the Driver Station that the controller is plugged into (0-5)
   */
  public LogitechDualshockController(int port) {
    super(port);
  }

  /**
   * Get the value of the left X-axis.
   *
   * @return The value of the axis.
   */
  public double getLeftX() {
    return getRawAxis(Axis.LeftX.value);
  }

  /**
   * Get the value of the left X-axis.
   *
   * @return The value of the axis.
   */
  public double getRightX() {
    return getRawAxis(Axis.RightX.value);
  }

  /**
   * Get the value of the left Y-axis.
   *
   * @return The value of the axis.
   */
  public double getLeftY() {
    return getRawAxis(Axis.LeftY.value);
  }

  /**
   * Get the value of the left X-axis.
   *
   * @return The value of the axis.
   */
  public double getRightY() {
    return getRawAxis(Axis.RightY.value);
  }

  ////////////////////////////////////////////////////////////
  // Raw button presses
  ////////////////////////////////////////////////////////////

  /**
   * Read the value of the A button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getAButton() {
    return getRawButton(Button.A.value);
  }

  /**
   * Read the value of the B button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getBButton() {
    return getRawButton(Button.B.value);
  }

  /**
   * Read the value of the X button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getXButton() {
    return getRawButton(Button.X.value);
  }

  /**
   * Read the value of the Y button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getYButton() {
    return getRawButton(Button.Y.value);
  }

  /**
   * Read the value of the left shoulder button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getLeftShoulderButton() {
    return getRawButton(Button.LeftShoulder.value);
  }

  /**
   * Read the value of the right shoulder button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getRightShoulderButton() {
    return getRawButton(Button.RightShoulder.value);
  }

  /**
   * Read the value of the left trigger button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getLeftTriggerButton() {
    return getRawButton(Button.LeftTrigger.value);
  }

  /**
   * Read the value of the right trigger button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getRightTriggerButton() {
    return getRawButton(Button.RightTrigger.value);
  }

  /**
   * Read the value of the Back button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getBackButton() {
    return getRawButton(Button.Back.value);
  }

  /**
   * Read the value of the Start button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getStartButton() {
    return getRawButton(Button.Start.value);
  }

  /**
   * Read the value of the left stick button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getLeftStickButton() {
    return getRawButton(Button.LeftStick.value);
  }

  /**
   * Read the value of the right stick button on the controller.
   *
   * @return The value of the button.
   */
  public boolean getRightStickButton() {
    return getRawButton(Button.RightStick.value);
  }

  ////////////////////////////////////////////////////////////
  // Button pressed since last check
  ////////////////////////////////////////////////////////////

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return The value of the button.
   */
  public boolean getAButtonPressed() {
    return getRawButtonPressed(Button.A.value);
  }

  /**
   * Whether the B button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getBButtonPressed() {
    return getRawButtonPressed(Button.B.value);
  }

  /**
   * Whether the X button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getXButtonPressed() {
    return getRawButtonPressed(Button.X.value);
  }

  /**
   * Whether the Y button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getYButtonPressed() {
    return getRawButtonPressed(Button.Y.value);
  }

  /**
   * Whether the left shoulder button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getLeftShoulderButtonPressed() {
    return getRawButtonPressed(Button.LeftShoulder.value);
  }

  /**
   * Whether the right shoulder button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getRightShoulderButtonPressed() {
    return getRawButtonPressed(Button.RightShoulder.value);
  }

  /**
   * Whether the left trigger button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getLeftTriggerButtonPressed() {
    return getRawButtonPressed(Button.LeftTrigger.value);
  }

  /**
   * Whether the right trigger button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getRightTriggerButtonPressed() {
    return getRawButtonPressed(Button.RightTrigger.value);
  }

  /**
   * Whether the Back button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getBackButtonPressed() {
    return getRawButtonPressed(Button.Back.value);
  }

  /**
   * Whether the Start button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getStartButtonPressed() {
    return getRawButtonPressed(Button.Start.value);
  }

  /**
   * Whether the left stick button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getLeftStickButtonPressed() {
    return getRawButtonPressed(Button.LeftStick.value);
  }

  /**
   * Whether the right stick button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getRightStickButtonPressed() {
    return getRawButtonPressed(Button.RightStick.value);
  }

  ////////////////////////////////////////////////////////////
  // Button released since last check
  ////////////////////////////////////////////////////////////

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return The value of the button.
   */
  public boolean getAButtonReleased() {
    return getRawButtonReleased(Button.A.value);
  }

  /**
   * Whether the B button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getBButtonReleased() {
    return getRawButtonReleased(Button.B.value);
  }

  /**
   * Whether the X button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getXButtonReleased() {
    return getRawButtonReleased(Button.X.value);
  }

  /**
   * Whether the Y button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getYButtonReleased() {
    return getRawButtonReleased(Button.Y.value);
  }

  /**
   * Whether the left shoulder button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getLeftShoulderButtonReleased() {
    return getRawButtonReleased(Button.LeftShoulder.value);
  }

  /**
   * Whether the right shoulder button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getRightShoulderButtonReleased() {
    return getRawButtonReleased(Button.RightShoulder.value);
  }

  /**
   * Whether the left trigger button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getLeftTriggerButtonReleased() {
    return getRawButtonReleased(Button.LeftTrigger.value);
  }

  /**
   * Whether the right trigger button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getRightTriggerButtonReleased() {
    return getRawButtonReleased(Button.RightTrigger.value);
  }

  /**
   * Whether the Back button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getBackButtonReleased() {
    return getRawButtonReleased(Button.Back.value);
  }

  /**
   * Whether the Start button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getStartButtonReleased() {
    return getRawButtonReleased(Button.Start.value);
  }

  /**
   * Whether the left stick button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getLeftStickButtonReleased() {
    return getRawButtonReleased(Button.LeftStick.value);
  }

  /**
   * Whether the right stick button was pressed since the last check.   *
   *
   * @return The value of the button.
   */
  public boolean getRightStickButtonReleased() {
    return getRawButtonReleased(Button.RightStick.value);
  }

  ////////////////////////////////////////////////////////////
  // Event loop construction
  ////////////////////////////////////////////////////////////

  /**
   * Constructs an event instance around the X button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the X button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent x(EventLoop loop) {
    return button(Button.X.value, loop);
  }

  /**
   * Constructs an event instance around the Y button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Y button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent y(EventLoop loop) {
    return button(Button.Y.value, loop);
  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent a(EventLoop loop) {
    return button(Button.A.value, loop);
  }

  /**
   * Constructs an event instance around the B button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the B button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent b(EventLoop loop) {
    return button(Button.B.value, loop);
  }

  /**
   * Constructs an event instance around the left shoulder button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left shoulder button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent leftShoulder(EventLoop loop) {
    return button(Button.LeftShoulder.value, loop);
  }

  /**
   * Constructs an event instance around the right shoulder button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the right shoulder button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent rightShoulder(EventLoop loop) {
    return button(Button.RightShoulder.value, loop);
  }

  /**
   * Constructs an event instance around the left trigger button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left trigger button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent leftTrigger(EventLoop loop) {
    return button(Button.LeftTrigger.value, loop);
  }

  /**
   * Constructs an event instance around the right trigger button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the right trigger button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent rightTrigger(EventLoop loop) {
    return button(Button.RightTrigger.value, loop);
  }

  /**
   * Constructs an event instance around the back button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the back button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent back(EventLoop loop) {
    return button(Button.Back.value, loop);
  }

  /**
   * Constructs an event instance around the start button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the start button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent start(EventLoop loop) {
    return button(Button.Start.value, loop);
  }

  /**
   * Constructs an event instance around the left stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left stick button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent leftStick(EventLoop loop) {
    return button(Button.LeftStick.value, loop);
  }

  /**
   * Constructs an event instance around the right stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the right stick button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent rightStick(EventLoop loop) {
    return button(Button.RightStick.value, loop);
  }
}
