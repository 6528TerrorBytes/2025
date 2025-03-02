// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.WPIPID.AlgaeFork;
import frc.robot.subsystems.WPIPID.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeForkMove extends Command {
  private final AlgaeFork m_algaeFork;
  private final Elevator m_elevator;
  private final double m_setpoint;
  
  private boolean m_hasMoved;

  /** Creates a new AlgaeForkMove. */
  public AlgaeForkMove(AlgaeFork algaeFork, Elevator elevator, double setpoint) {
    m_algaeFork = algaeFork;
    m_elevator = elevator;
    m_setpoint = setpoint;

    addRequirements(algaeFork);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasMoved = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_hasMoved && m_elevator.getPos() > Constants.Setpoints.elevatorAlgaeFlapMovePos) {
      m_hasMoved = true;

      m_algaeFork.enable();
      m_algaeFork.setGoal(m_setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_algaeFork.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hasMoved && m_algaeFork.atGoal();
  }
}
