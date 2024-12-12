// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public final class Utility {
  public static boolean teamColorIsRed() {
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red; 
  }
}
