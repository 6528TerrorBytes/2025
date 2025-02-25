// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WPIPID;

import frc.robot.Constants;

public class AlgaeFork extends WPIArm {
  /** Creates a new AlgaeFork. */
  public AlgaeFork() {
    super(
      Constants.MotorIDs.algaeForkID, Constants.MotorConfig.algaeForkConfig, Constants.MotorConfig.algaeForkTolerance,
      1000, 1000,
      10, 0, 0,
      0, 50, 0
    );
  }

  @Override
  public double getAngleFromHorizontal(double encoderPos) {
    return encoderPos - Constants.Setpoints.algaeForkHorizontal;
  }
}
