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

  private final double m_stopDelay = 0.2; // seconds delay between the beam state changing and the intake motor stopping
  private double m_stopTime;

  public IntakeMove(IntakeMotor intakeMotor, CoralDetector coralDetector) {
    m_intakeMotor = intakeMotor;
    m_coralDetector = coralDetector;
    m_stopTime = 0;

    addRequirements(m_intakeMotor);
  }

  @Override
  public void initialize() {
    m_startDetected = m_coralDetector.detected();
    m_intakeMotor.onForward(); // drives intake motor
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
    return (m_stopTime != 0) && (Utility.getTime() > m_stopTime);
  }
}
