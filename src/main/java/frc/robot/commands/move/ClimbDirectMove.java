// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SparkPID.Climb;
import frc.robot.subsystems.SparkPID.SparkMove;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbDirectMove extends Command {
  private final double m_angle;

  private SparkMove m_motor;
  private boolean m_inner;

  /** Creates a new ClimbManualMove. */
  public ClimbDirectMove(Climb climb, boolean isInnerMotor, double angle) {
    m_angle = angle;

    m_inner = isInnerMotor;

    if (isInnerMotor) {
      m_motor = climb.getInnerMotor();
    } else {
      m_motor = climb.getOuterMotor();
    }

    // addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_motor.setPower(0.9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motor.disable();
    System.out.println("ended " + m_inner);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_motor.getPos() + " " + m_inner);
    return m_motor.getPos() > m_angle;
  }
}
