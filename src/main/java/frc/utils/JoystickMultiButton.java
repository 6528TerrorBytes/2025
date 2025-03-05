package frc.utils;

import edu.wpi.first.wpilibj.GenericHID;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickMultiButton extends Trigger {
  /**
   * Creates a joystick button for triggering commands when the analog input passes a threshold
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickMultiButton(GenericHID joystick, int masterButton, int mainButton) {
    super(() -> (joystick.getRawButton(masterButton) && joystick.getRawButton(mainButton)));
    requireNonNullParam(joystick, "joystick", "JoystickMultiButton");
  }
}