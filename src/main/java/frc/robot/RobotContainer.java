// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  public final Joystick leftJoystick = new Joystick(0);
  public final Joystick rightJoystick = new Joystick(1);

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public RobotContainer() {
    registerPathplannerCommands();
    setDriveCommand();
    configureBindings();
  }

  public void setDriveCommand() {
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(rightJoystick.getY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(rightJoystick.getX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(leftJoystick.getZ(), OIConstants.kDriveDeadband),
          true, true, true),
        m_robotDrive 
      )
    ); // Call of duty (:<
  }
  
  private void configureBindings() {

  }

  private void registerPathplannerCommands() {
    // register pathplanner commands here when we get there
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new RunCommand(() -> {});
  }
}
