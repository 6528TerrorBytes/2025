// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveSpeedChange extends Command {
  public final double m_newSpeed;
  
  public DriveSpeedChange(double newSpeed) {
    m_newSpeed = newSpeed;
  }

  @Override
  public void initialize() {
    DriveSubsystem.speedMultiplier = m_newSpeed;
  }

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.speedMultiplier = 1;
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
