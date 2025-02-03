// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveSpeedChange;
import frc.robot.commands.ElevatorMove;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Motors.Elevator;

public class RobotContainer {
  // Joysticks
  public final Joystick leftJoystick = new Joystick(0);
  public final Joystick rightJoystick = new Joystick(1);
  public final Joystick otherJoystick = new Joystick(3);

  private SendableChooser<String> m_pathPlannerChooser = new SendableChooser<String>();

  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_elevator = new Elevator();
  
  public RobotContainer() {
    registerPathplannerCommands();
    setupPathplannerSelector();

    setDriveCommand();
    configureControllerBindings();
  }

  public void setDriveCommand() {
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(Math.pow(rightJoystick.getY(), 1) / 2, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(Math.pow(rightJoystick.getX(), 1) / 2, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(Math.pow(leftJoystick.getZ(),  1) / 2, OIConstants.kDriveDeadband),
          true, true, true),
        m_robotDrive 
      )
    ); // Call of duty (:<
  }
  
  private void configureControllerBindings() {
    new JoystickButton(rightJoystick, 2).whileTrue(new DriveSpeedChange(0.5));

    new JoystickButton(leftJoystick, 1).whileTrue(new ElevatorMove(m_elevator, 20));
    new JoystickButton(leftJoystick, 2).whileTrue(new ElevatorMove(m_elevator, 0));

    // new JoystickButton(leftJoystick, 3).whileTrue(new DriveToAprilTag(m_robotDrive, false));
    
    // Reset gyro
    new JoystickButton(leftJoystick, 11).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
  }

  private void registerPathplannerCommands() {
    // register pathplanner commands here when we get there
  }

  private void setupPathplannerSelector() {
    // sets up the Smartdashboard dropdown selector to pick which autonomous to run
    
    // List all the autos from Pathplanner here:
    m_pathPlannerChooser.addOption("option 1", "yourmomauto1");
    m_pathPlannerChooser.addOption("option 2", "yourmomauto2");

    SmartDashboard.putData("Select Pathplanner Auton", m_pathPlannerChooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String selectedAuto = m_pathPlannerChooser.getSelected();

    if (selectedAuto == null) {
      return null;
    }

    PathPlannerAuto auto = new PathPlannerAuto(selectedAuto);
    m_robotDrive.resetOdometry(auto.getStartingPose()); // Sets the odometry to the starting position of the robot in the autonomous
    return auto;
  }

  public void updateSmartDashboard() {
    m_robotDrive.updateSmartDashboard();
    SmartDashboard.putBoolean("April Tag In View", Utility.aprilTagInView());
  }
}
