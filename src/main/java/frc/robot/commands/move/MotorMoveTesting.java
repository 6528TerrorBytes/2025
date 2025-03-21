// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

// Moves a motor at a set speed
public class MotorMoveTesting extends Command {
  private final SparkMax m_motor;
  private final double m_speed;
  /** Creates a new MotorMoveTesting. */
  public MotorMoveTesting(SparkMax motor, double speed) {
    m_motor = motor;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_motor.set(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
