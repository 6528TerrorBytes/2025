package frc.utils;

import edu.wpi.first.wpilibj.GenericHID;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickAnalogButton extends Trigger {
  /**
   * Creates a joystick button for triggering commands when the analog input passes a threshold
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickAnalogButton(GenericHID joystick, int axisNumber, double lowerThreshold, double upperThreshold) {
    super(() -> (joystick.getRawAxis(axisNumber) > lowerThreshold && joystick.getRawAxis(axisNumber) < upperThreshold));
    requireNonNullParam(joystick, "joystick", "JoystickAnalogButton");
  }
}