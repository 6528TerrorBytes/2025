// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Motors;

import frc.robot.Constants;

public class Elevator extends SparkMove {
  /** Creates a new Elevator. */
  public Elevator() {
    super(9, Constants.MotorConfig.elevatorConfig);
    setTolerance(0.5); // test
  }
}
