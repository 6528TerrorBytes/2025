// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveSpeedChange;
import frc.robot.commands.move.ArmMove;
import frc.robot.commands.move.ClimbMove;
import frc.robot.commands.move.ElevatorMove;
import frc.robot.commands.move.IntakeMove;
import frc.robot.commands.move.MotorMoveTesting;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotor;
import frc.robot.subsystems.WPIPID.IntakeArm;
import frc.robot.subsystems.WPIPID.Elevator;
import frc.robot.subsystems.SparkPID.Climb;
import frc.utils.JoystickAnalogButton;

public class RobotContainer {
  // Joysticks
  public final Joystick leftJoystick = new Joystick(0);
  public final Joystick rightJoystick = new Joystick(1);
  public final Joystick otherJoystick = new Joystick(2);

  private SendableChooser<String> m_pathPlannerChooser = new SendableChooser<String>();

  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final Climb m_climb = new Climb();
  private final Elevator m_elevator = new Elevator();
  private final IntakeArm m_arm = new IntakeArm();
  private final IntakeMotor m_intakeMotor = new IntakeMotor();

  private final CoralDetector m_coralDetector = new CoralDetector();
  
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
    );
    // ); // Call of duty (:<
  }
  
  private void configureControllerBindings() {
    // new JoystickButton(rightJoystick, 2).whileTrue(new DriveSpeedChange(0.5));

    // new JoystickButton(leftJoystick, 1).whileTrue(new ElevatorMove(m_elevator, 20));
    // new JoystickButton(leftJoystick, 2).whileTrue(new ElevatorMove(m_elevator, 0));

    // new JoystickButton(leftJoystick, 3).whileTrue(new DriveToAprilTag(m_robotDrive, false));
    
    // Reset gyro
    new JoystickButton(rightJoystick, 14).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
    // new JoystickButton(rightJoystick, 13).onTrue(new InstantCommand(() -> m_robotDrive.setX()));

    new JoystickButton(otherJoystick, 3).whileTrue(new ElevatorMove(m_elevator, 5.3));
    new JoystickButton(otherJoystick, 2).whileTrue(new ElevatorMove(m_elevator, 2.15));
    new JoystickButton(otherJoystick, 1).whileTrue(new ElevatorMove(m_elevator, 2));

    new JoystickButton(otherJoystick, 4).whileTrue(new ArmMove(m_arm, Constants.MotorConfig.armAngleVerticalDown + 37));
    new JoystickButton(otherJoystick, 5).whileTrue(new ArmMove(m_arm, Constants.MotorConfig.armAngleVerticalDown));
    new JoystickButton(otherJoystick, 6).whileTrue(new ArmMove(m_arm, Constants.MotorConfig.armAngleHorizontal + 60));
 
    // new JoystickButton(otherJoystick, 3).whileTrue(new MotorMoveTesting(m_elevator.m_motor, 0.5));
    // new JoystickButton(otherJoystick, 4).whileTrue(new MotorMoveTesting(m_elevator.m_motor, -0.2));

    // Triggers
    new JoystickAnalogButton(otherJoystick, 3, 0.5).whileTrue(new IntakeMove(m_intakeMotor, m_coralDetector));
    new JoystickAnalogButton(otherJoystick, 2, 0.5).whileTrue(new InstantCommand(() -> m_elevator.zeroEncoder()));
    
    // new JoystickButton(otherJoystick, 3).whileTrue(new MotorMoveTesting(climbinner, 0.35));
    // new JoystickButton(otherJoystick, 4).whileTrue(new MotorMoveTesting(climbinner, -0.35));

    // new JoystickButton(otherJoystick, 7).whileTrue(new MotorMoveTesting(climbouter, 0.35));
    // new JoystickButton(otherJoystick, 8).whileTrue(new MotorMoveTesting(climbouter, -0.35));



    // Climb
    // new JoystickButton(otherJoystick, 1).whileTrue(new ClimbMove(m_climb, 20));
    // new JoystickButton(otherJoystick, 2).whileTrue(new ClimbMove(m_climb, 140));
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
    SmartDashboard.putNumber("elevator", m_elevator.m_encoder.getPosition());
    SmartDashboard.putBoolean("Coral Detected", m_coralDetector.detected());
    SmartDashboard.putNumber("Arm Encoder", m_arm.m_encoder.getPosition());
    m_climb.updateSmartDashboard();
  }
}
