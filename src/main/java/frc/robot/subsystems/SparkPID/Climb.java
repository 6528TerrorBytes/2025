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

  public SparkMove getOuterMotor() {
    return m_outerMotor;
  }
  
  public SparkMove getInnerMotor() {
    return m_innerMotor;
  }
}
