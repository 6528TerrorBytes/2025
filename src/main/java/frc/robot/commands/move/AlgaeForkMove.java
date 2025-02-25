// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WPIPID.AlgaeFork;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeForkMove extends Command {
  private final AlgaeFork m_algaeFork;
  private final double m_setpoint;

  /** Creates a new AlgaeForkMove. */
  public AlgaeForkMove(AlgaeFork algaeFork, double setpoint) {
    m_algaeFork = algaeFork;
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algaeFork.enable();
    m_algaeFork.setGoal(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeFork.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
