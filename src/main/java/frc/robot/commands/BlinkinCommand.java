// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.CoralDetector;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlinkinCommand extends Command {
  private final Blinkin m_blinkin;
  private final CoralDetector m_coralDetector;
  
  private static boolean m_isTeleop;
  
  private final double timeToWarn = 20; // Warn at 20 seconds left
  private final double warnEnd = 17; // Time when the flashing ends

  /** Creates a new BlinkinCommand. */
  public BlinkinCommand(Blinkin blinkin, CoralDetector coralDetector) {
    m_blinkin = blinkin;
    m_coralDetector = coralDetector;

    addRequirements(blinkin);
  }

  @Override
  public void execute() {
    double secondsLeftInPeriod = Utility.getMatchTime();

    // Between the time where the flash occurs
    if (m_isTeleop && (warnEnd < secondsLeftInPeriod) && (secondsLeftInPeriod < timeToWarn)) {
      m_blinkin.resetToTeamColor( // Set to strobe colors
        Constants.BlinkinConfig.strobeBlue,
        Constants.BlinkinConfig.strobeRed
      );
      return;
    }
    
    // Sets to green if it has a note, otherwise to the team color
    if (m_coralDetector.detected()) {
      m_blinkin.setColor(Constants.BlinkinConfig.green);
    } else {
      m_blinkin.resetToTeamColor(
        Constants.BlinkinConfig.blue,
        Constants.BlinkinConfig.red
      );
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
  public static void setTeleop(boolean isTeleop) {
    m_isTeleop = isTeleop;
  }
}
