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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.move.AlgaeForkMove;
import frc.robot.commands.move.ArmMove;
import frc.robot.commands.move.ClimbDirectMove;
import frc.robot.commands.move.ElevatorMove;
import frc.robot.commands.move.IntakeMove;
import frc.robot.commands.move.MotorMoveTesting;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.IntakeMotor;
import frc.robot.subsystems.WPIPID.IntakeArm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.JoystickAnalogButton;
import frc.robot.subsystems.WPIPID.AlgaeFork;
import frc.robot.subsystems.WPIPID.Elevator;
import frc.robot.subsystems.SparkPID.Climb;

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
  private final AlgaeFork m_algaeFork = new AlgaeFork();
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
    // new JoystickButton(rightJoystick, 15).onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry()));
    new JoystickButton(rightJoystick, 16).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
    // new JoystickButton(rightJoystick, 13).onTrue(new InstantCommand(() -> m_robotDrive.setX()));

    // ELEVATOR
    new JoystickButton(leftJoystick, 14).whileTrue(new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabSecond));
    new JoystickButton(leftJoystick, 13).whileTrue(new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorHigh));
    new JoystickButton(leftJoystick, 12).whileTrue(new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorMedium));
    new JoystickButton(leftJoystick, 11).whileTrue(new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero));
    
    // Zero elevator
    new JoystickButton(leftJoystick, 16).whileTrue(new InstantCommand(() -> m_elevator.zeroEncoder()));

    // MANUAL ELEVATOR
    new JoystickButton(otherJoystick, 3).whileTrue(new MotorMoveTesting(m_elevator.m_motor, 0.4));
    new JoystickButton(otherJoystick, 4).whileTrue(new MotorMoveTesting(m_elevator.m_motor, -0.15));
    new JoystickAnalogButton(leftJoystick, 3, 0.5).whileTrue(new ElevatorMove(m_elevator, m_arm, m_elevator.getPos()));

    // High shooting mockup automation
    // new SequentialCommandGroup(
    //   new ArmMove(m_arm, Constants.Setpoints.armAngleHigh),
    //   new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorHigh),

    //   // here: position the robot next to the reef properly
    
    //   new ArmMove(m_arm, Constants.Setpoints.armAngleHorizontal),
      
    //   new ParallelDeadlineGroup( // Shoot then pull arm back
    //     new ArmMove(m_arm, Constants.Setpoints.armAngleHigh),
    //     new IntakeMove(m_intakeMotor, m_coralDetector)
    //   )
    // );

    // Sequential command testing
    new JoystickButton(leftJoystick, 2).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabSecond),
        new ArmMove(m_arm, Constants.Setpoints.armAngleHigh),

        new ParallelCommandGroup( // Wait half a second before moving out algae fork
          new WaitCommand(0.3),
          new AlgaeForkMove(m_algaeFork, Constants.Setpoints.algaeForkHorizontal)
        )
      )
    ));

    new JoystickButton(leftJoystick, 3).whileTrue(new SequentialCommandGroup(
      new ArmMove(m_arm, Constants.Setpoints.armAngleHoldAlgae),
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabSecond + 0.4))
    );

    new JoystickButton(leftJoystick, 1).whileTrue(new DriveToAprilTag(m_robotDrive, true));

    // ARM
    new JoystickButton(leftJoystick, 5).whileTrue(new ArmMove(m_arm, Constants.Setpoints.armAngleVerticalDown));
    new JoystickButton(leftJoystick, 6).whileTrue(new ArmMove(m_arm, Constants.Setpoints.armAngleMedium));
    new JoystickButton(leftJoystick, 7).whileTrue(new ArmMove(m_arm, Constants.Setpoints.armAngleHigh));
    new JoystickButton(leftJoystick, 8).whileTrue(new ArmMove(m_arm, Constants.Setpoints.armAngleHorizontal));
    new JoystickButton(leftJoystick, 9).whileTrue(new ArmMove(m_arm, Constants.Setpoints.armAngleHoldAlgae));

    // ALGAE FORK
    new JoystickButton(rightJoystick, 5).whileTrue(new AlgaeForkMove(m_algaeFork, Constants.Setpoints.algaeForkZero));
    new JoystickButton(rightJoystick, 6).whileTrue(new AlgaeForkMove(m_algaeFork, Constants.Setpoints.algaeForkHorizontal - 25));
    new JoystickButton(rightJoystick, 7).whileTrue(new AlgaeForkMove(m_algaeFork, Constants.Setpoints.algaeForkHorizontal));

    // CLIMB SETPOINT
    // new JoystickButton(otherJoystick, 1).whileTrue(new ClimbMove(m_climb, 29));
    // new JoystickButton(otherJoystick, 2).whileTrue(new ClimbMove(m_climb, 145));

    // CLIMB DIRECT MOVE
    new JoystickButton(otherJoystick, 1).whileFalse(new ClimbDirectMove(m_climb, 140));

    // INTAKE
    new JoystickButton(rightJoystick, 1).whileTrue(new IntakeMove(m_intakeMotor, m_coralDetector));
    
    // MANUAL CLIMB
    // new JoystickButton(otherJoystick, 3).whileTrue(new MotorMoveTesting(climbinner, 0.35));
    // new JoystickButton(otherJoystick, 4).whileTrue(new MotorMoveTesting(climbinner, -0.35));

    // new JoystickButton(otherJoystick, 7).whileTrue(new MotorMoveTesting(climbouter, 0.35));
    // new JoystickButton(otherJoystick, 8).whileTrue(new MotorMoveTesting(climbouter, -0.35));
  }

  private void registerPathplannerCommands() {
    // register pathplanner commands here when we get there
  }

  private void setupPathplannerSelector() {
    // sets up the Smartdashboard dropdown selector to pick which autonomous to run
    
    // List all the autos from Pathplanner here:
    m_pathPlannerChooser.addOption("Pos testing", "Pos testing auto");

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
    SmartDashboard.putBoolean("April Tag In View", Utility.aprilTagInView("limelight-two"));
    SmartDashboard.putNumber("elevator", m_elevator.m_encoder.getPosition());
    SmartDashboard.putBoolean("Coral Detected", m_coralDetector.detected());
    SmartDashboard.putNumber("Arm Encoder", m_arm.m_encoder.getPosition());
    // SmartDashboard.putNumber("Algae Forks", m_algaeFork.m_encoder.getPosition());
    SmartDashboard.putNumber("Gyro", m_robotDrive.getRawAngle());
    m_climb.updateSmartDashboard();
  }
}
