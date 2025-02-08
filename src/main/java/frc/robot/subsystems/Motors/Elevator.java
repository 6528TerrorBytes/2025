// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Motors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Elevator extends SparkMoveRelative {
  /** Creates a new Elevator. */
  public Elevator() {
    super(4, Constants.MotorConfig.elevatorConfig);
    setTolerance(0.5); // test
  }

  @Override
  public void periodic() {
    
  }
}
