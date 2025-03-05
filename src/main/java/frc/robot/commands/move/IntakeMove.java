// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.IntakeMotor;

public class IntakeMove extends Command {
  private final IntakeMotor m_intakeMotor;
  private final CoralDetector m_coralDetector;

  private boolean m_startDetected;

  private double m_stopDelay; // seconds delay between the beam state changing and the intake motor stopping
  private double m_stopTime;

  private boolean m_reverse;

  public IntakeMove(IntakeMotor intakeMotor, CoralDetector coralDetector, double stopDelay, boolean reverse) {
    m_intakeMotor = intakeMotor;
    m_coralDetector = coralDetector;
    m_stopDelay = stopDelay;
    m_stopTime = 0;
    m_reverse = reverse;

    addRequirements(m_intakeMotor);
  }

  @Override
  public void initialize() {
    m_stopTime = 0;
    m_startDetected = m_coralDetector.detected();

    if (m_reverse) {
      m_intakeMotor.onBackwards();
    } else {
      m_intakeMotor.onForward(); // drives intake motor
    }
  }

  @Override
  public void execute() {
    if (m_stopTime != 0) return;

    if (m_coralDetector.detected() != m_startDetected) { // beam status changed
      m_stopTime = Utility.getTime() + m_stopDelay; // set the stop time
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeMotor.disable();
  }

  @Override
  public boolean isFinished() {
    return (m_stopTime != 0) && (Utility.getTime() >= m_stopTime);
  }
}
