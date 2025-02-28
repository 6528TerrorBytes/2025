// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TailIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TailIntakeMove extends Command {
  private final TailIntake m_tailIntake;
  private final double m_power;

  /** Creates a new TailIntakeMove. */
  public TailIntakeMove(TailIntake tailIntake, double power) {
    m_tailIntake = tailIntake;
    m_power = power;
    
    addRequirements(m_tailIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tailIntake.set(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tailIntake.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
