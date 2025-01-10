// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // Joysticks
  public final Joystick leftJoystick = new Joystick(0);
  public final Joystick rightJoystick = new Joystick(1);
  public final Joystick otherJoystick = new Joystick(3);

  private SendableChooser<String> m_pathPlannerChooser = new SendableChooser<String>();
  private Field2d m_field;

  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  

  public RobotContainer() {
    registerPathplannerCommands();
    setupPathplannerSelector();
    setPathplannerFieldWidget();

    setDriveCommand();
    configureControllerBindings();
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
  
  private void configureControllerBindings() {

  }

  private void setPathplannerFieldWidget() {
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    // IDK if this works lol
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      m_field.getObject("target pose").setPose(pose);
    });
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

    return new PathPlannerAuto(selectedAuto);
  }
}
