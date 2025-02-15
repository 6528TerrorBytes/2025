// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.IntakeMotor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeMove extends Command {
  private final IntakeMotor m_intakeMotor;
  private final CoralDetector m_coralDetector;

  private boolean m_startDetected;

  public IntakeMove(IntakeMotor intakeMotor, CoralDetector coralDetector) {
    m_intakeMotor = intakeMotor;
    m_coralDetector = coralDetector;

    addRequirements(m_intakeMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startDetected = m_coralDetector.detected();
    m_intakeMotor.onForward();
  }

  @Override
  public void execute() {
    if (!m_coralDetector.detected()) m_startDetected = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeMotor.disable();
  }

  // Command finishes when coral is detected
  @Override
  public boolean isFinished() {
    return !m_startDetected && m_coralDetector.detected();
  }
}
