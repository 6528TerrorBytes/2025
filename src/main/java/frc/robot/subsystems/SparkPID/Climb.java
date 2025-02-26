// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SparkPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private final SparkMove m_outerMotor;
  private final SparkMove m_innerMotor;

  /** Creates a new Climb. */
  public Climb() {
    m_outerMotor = new SparkMove(Constants.MotorIDs.climbOuterID, Constants.MotorConfig.outerClimbConfig);
    m_innerMotor = new SparkMove(Constants.MotorIDs.climbInnerID, Constants.MotorConfig.innerClimbConfig);

    // Set tolerances
    m_outerMotor.setTolerance(10);
    m_innerMotor.setTolerance(10);
  }

  public void setGoal(double angle) {
    m_outerMotor.setGoal(angle); // test one at a time
    m_innerMotor.setGoal(angle);
  }

  public boolean atGoal() {
    return m_outerMotor.atGoal() && m_innerMotor.atGoal();
  }

  public void disable() {
    m_outerMotor.disable();
    m_innerMotor.disable();
  }

  public void setPower(double power) {
    m_outerMotor.setPower(power);
    m_innerMotor.setPower(power);
  }

  public double getPos() {
    return m_outerMotor.getPos();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("OuterClimb", m_outerMotor.getPos());
    SmartDashboard.putNumber("InnerClimb", m_innerMotor.getPos());
  }
}
