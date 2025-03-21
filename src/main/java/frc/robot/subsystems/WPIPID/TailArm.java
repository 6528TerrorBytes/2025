// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WPIPID;

import frc.robot.Constants;

public class TailArm extends WPIArm {
  /** Creates a new TailArm. */
  public TailArm() {
    super(
      Constants.MotorIDs.tailArmID, Constants.MotorConfig.tailArmConfig, Constants.MotorConfig.tailArmTolerance,
      300, 300,
      0.15, 0, 0,
      0, 0, 0
    );
    
    enable();
    setGoal(Constants.Setpoints.tailArmPreintake);
  }

  @Override
  public double getAngleFromHorizontal(double encoderPos) {
    return encoderPos - Constants.Setpoints.tailArmHorizontal;
  }
}
