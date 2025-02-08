// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Motors;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private final SparkMove m_outerMotor;
  // private final SparkMove m_innerMotor; 

  /** Creates a new Climb. */
  public Climb() {
    Constants.MotorConfig.climbConfig.absoluteEncoder.inverted(true);
    m_outerMotor = new SparkMove(1, Constants.MotorConfig.climbConfig);

    // Inner is not inverted
    Constants.MotorConfig.climbConfig.absoluteEncoder.inverted(false);
    // m_innerMotor = new SparkMove(2, Constants.MotorConfig.innerClimbConfig);

    // Set tolerances
    m_outerMotor.setTolerance(10);
    // m_innerMotor.setTolerance(10);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void setGoal(double angle) {
    m_outerMotor.setGoal(angle); // test one at a time
    // m_innerMotor.setGoal(angle);
  }

  public boolean atGoal() {
    return m_outerMotor.atGoal(); // && m_innerMotor.atGoal();
  }

  public void disable() {
    m_outerMotor.disable();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("OuterClimb", m_outerMotor.getPos());
    // SmartDashboard.putNumber("InnerClimb", m_innerMotor.getPos());
  }
}
