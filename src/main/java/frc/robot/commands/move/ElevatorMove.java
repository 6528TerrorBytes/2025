// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.WPIPID.Elevator;
import frc.robot.subsystems.WPIPID.IntakeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {
  private final Elevator m_elevator;
  private final IntakeArm m_arm;
  private final double m_setPoint;

  private boolean m_hasMoved;

  /** Creates a new ElevatorMove. */
  public ElevatorMove(Elevator elevator, IntakeArm arm, double setPoint) {
    m_elevator = elevator;
    m_arm = arm;
    m_setPoint = setPoint;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasMoved = false;
  }
  
  @Override
  public void execute() {
    // If the elevator has not been set to move to its setpoint yet, then check if the arm has beyond the safety angle.
    // If the arm is beyond the safety angle, then move the elevator.

    if (!m_hasMoved && m_arm.getPos() > Constants.Setpoints.armElevatorMoveAngle) {
      m_hasMoved = true;
      
      m_elevator.enable();
      m_elevator.setGoal(m_setPoint);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    // m_elevator.disable();
    System.out.println("Elevator position reached");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hasMoved && m_elevator.atGoal();
  }
}
