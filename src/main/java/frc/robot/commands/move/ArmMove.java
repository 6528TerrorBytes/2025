// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.WPIPID.Elevator;
import frc.robot.subsystems.WPIPID.IntakeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmMove extends Command {
  private final IntakeArm m_arm; 
  private final Elevator m_elevator;
  private final double m_setPoint;

  private boolean m_hasMoved;

  /** Creates a new ArmMove. */
  public ArmMove(IntakeArm arm, Elevator elevator, double setPoint) {
    m_arm = arm;
    m_elevator = elevator;
    m_setPoint = setPoint;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasMoved = true;
    m_arm.enable();
    m_arm.setGoal(m_setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!m_hasMoved) {
    //   // if arm is being set to vertical down, then don't set it until the elevator is all the way down
    //   if (m_setPoint < Constants.Setpoints.armElevatorMoveAngle &&
    //       m_elevator.getPos() > Constants.Setpoints.elevatorZero + Constants.MotorConfig.elevatorTolerance)
    //     return;

    //   System.out.println("setting arm angle");
      
    //   m_arm.enable();
    //   m_arm.setGoal(m_setPoint);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_arm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atGoal();
  }
}
