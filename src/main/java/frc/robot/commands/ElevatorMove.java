// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WPIElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {
  private final WPIElevator m_elevator;
  private final double m_setPoint;

  /** Creates a new ElevatorMove. */
  public ElevatorMove(WPIElevator elevator, double setPoint) {
    m_elevator = elevator;
    m_setPoint = setPoint;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.enable();
    m_elevator.setGoal(m_setPoint);
  }

  @Override
  public void execute() {}
  
  @Override
  public void end(boolean interrupted) {
    m_elevator.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
