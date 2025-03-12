// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WPIPID;

import frc.robot.Constants;

public class IntakeArm extends WPIArm {
  public IntakeArm() {
    super(
      Constants.MotorIDs.armID, Constants.MotorConfig.armConfig, Constants.MotorConfig.armTolerance,
      1000, 1000, // Max velocity, Max acceleration
      0.035, 0, 0, // PID
      0, 0.6, 0 // S, G, V (Arm Feedforward)
    );

    enable();
    setGoal(Constants.Setpoints.armAngleInitial);
  }

  @Override
  public double getAngleFromHorizontal(double encoderPos) {
    return encoderPos - Constants.Setpoints.armAngleHorizontal - (Constants.Setpoints.armAngleAtRest - Constants.Setpoints.armAngleStore);
  }
}