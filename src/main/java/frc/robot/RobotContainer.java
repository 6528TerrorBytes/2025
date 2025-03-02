// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BlinkinCommand;
import frc.robot.commands.DriveSpeedChange;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.move.AlgaeForkMove;
import frc.robot.commands.move.ArmMove;
import frc.robot.commands.move.ClimbDirectMove;
import frc.robot.commands.move.ElevatorMove;
import frc.robot.commands.move.IntakeMove;
import frc.robot.commands.move.MotorMoveTesting;
import frc.robot.commands.move.TailArmMove;
import frc.robot.commands.move.TailIntakeMove;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.IntakeMotor;
import frc.robot.subsystems.TailIntake;
import frc.robot.subsystems.WPIPID.IntakeArm;
import frc.robot.subsystems.WPIPID.TailArm;
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
  private final TailArm m_tailArm = new TailArm();
  private final TailIntake m_tailIntake = new TailIntake();

  private final CoralDetector m_coralDetector = new CoralDetector();

  private final Blinkin m_blinkin = new Blinkin();
  private final BlinkinCommand m_blinkinCommand = new BlinkinCommand(m_blinkin, m_coralDetector);

  // Shared commands -- used in both joysticks and autons
  private Command c_dunkScore;
  
  public RobotContainer() {
    configureSharedCommands();

    setDriveCommand();
    configureControllerBindings();
    
    registerPathplannerCommands();
    setupPathplannerSelector();

    m_blinkin.setDefaultCommand(m_blinkinCommand);
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

  // Commands that are used in both autos and in joystick buttons
  private void configureSharedCommands() {
    
    // Dunk the coral L4
    c_dunkScore = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorHigh),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      ),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHorizontal),
      new ParallelCommandGroup(
        new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      ),
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero)
    );

  }
  
  private void configureControllerBindings() {
    new JoystickButton(leftJoystick, 1).whileTrue(new DriveSpeedChange(0.5));

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
    new JoystickButton(leftJoystick, 12).whileTrue(new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorIntake));
    new JoystickButton(leftJoystick, 11).whileTrue(new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero));
    
    // Zero elevator
    // new JoystickButton(leftJoystick, 16).whileTrue(new InstantCommand(() -> m_elevator.zeroEncoder()));

    // MANUAL ELEVATOR
    new JoystickButton(otherJoystick, 3).whileTrue(new MotorMoveTesting(m_elevator.m_motor, 0.4));
    new JoystickButton(otherJoystick, 4).whileTrue(new MotorMoveTesting(m_elevator.m_motor, -0.15));
    new JoystickAnalogButton(leftJoystick, 3, 0.5).whileTrue(new ElevatorMove(m_elevator, m_arm, m_elevator.getPos()));

    // High shooting mockup automation
    // new SequentialCommandGroup(
    //   new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh),
    //   new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorHigh),

    //   // here: position the robot next to the reef properly
    
    //   new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHorizontal),
      
    //   new ParallelDeadlineGroup( // Shoot then pull arm back
    //     new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh),
    //     new IntakeMove(m_intakeMotor, m_coralDetector)
    //   )
    // );

    // Algae 2nd level grab start position
    new JoystickButton(leftJoystick, 2).whileTrue(new ParallelCommandGroup(
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabSecond),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh),
      new AlgaeForkMove(m_algaeFork, m_elevator, Constants.Setpoints.algaeForkHorizontal - 5)
    ));

    // Algae 2nd level grab and lift up
    new JoystickButton(leftJoystick, 3).whileTrue(new ParallelCommandGroup(
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHoldAlgae),

      new SequentialCommandGroup(
        new WaitCommand(0.35),
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabSecond + 0.8)
      )
    ));

    // Coral intake arm + elevator position
    new JoystickButton(leftJoystick, 4).whileTrue(new ParallelCommandGroup(
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorIntake),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleIntake)
    ));

    // Dunk the coral L4
    new JoystickButton(rightJoystick, 2).whileTrue(c_dunkScore);

    // Score L3
    new JoystickButton(rightJoystick, 3).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorScoreMiddle),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleMiddleHigh)
      ),
      new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk),
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      )
    ));

    // Score L2
    new JoystickButton(rightJoystick, 4).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorScoreLow),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleLowScore)
      )
      // new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk),
      // new ParallelCommandGroup(
      //   new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero),
      //   new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      // )
    ));

    // Drive to score coral, left and right sides
    new JoystickButton(otherJoystick, 5).whileTrue(new DriveToAprilTag(m_robotDrive, Constants.AprilTags.coralOffsetLeft, "limelight-two", false));
    new JoystickButton(otherJoystick, 6).whileTrue(new DriveToAprilTag(m_robotDrive, Constants.AprilTags.coralOffsetRight, "limelight-two", false));
    
    new JoystickButton(leftJoystick, 1).whileTrue(new DriveToAprilTag(m_robotDrive, Constants.AprilTags.coralOffsetLeftLow, "limelight-two", false));
    // new JoystickButton(rightJoystick, 1).whileTrue(new DriveToAprilTag(m_robotDrive, Constants.AprilTags.coralOffsetRightLow, "limelight-two", false));

    new JoystickButton(rightJoystick, 14).whileTrue(new DriveToAprilTag(m_robotDrive, Constants.AprilTags.coralCollectOffset, "limelight-four", true));

    // ARM
    new JoystickButton(leftJoystick, 5).whileTrue(new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleVerticalDown));
    new JoystickButton(leftJoystick, 6).whileTrue(new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleIntake));
    new JoystickButton(leftJoystick, 7).whileTrue(new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh));
    new JoystickButton(leftJoystick, 8).whileTrue(new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHorizontal));
    new JoystickButton(leftJoystick, 9).whileTrue(new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHoldAlgae));

    // ALGAE FORK
    new JoystickButton(rightJoystick, 5).whileTrue(new AlgaeForkMove(m_algaeFork, m_elevator, Constants.Setpoints.algaeForkZero));
    new JoystickButton(rightJoystick, 6).whileTrue(new AlgaeForkMove(m_algaeFork, m_elevator, Constants.Setpoints.algaeForkHorizontal - 25));
    new JoystickButton(rightJoystick, 7).whileTrue(new AlgaeForkMove(m_algaeFork, m_elevator, Constants.Setpoints.algaeForkHorizontal));

    // TAIL ARM
    new JoystickButton(rightJoystick, 10).whileTrue(new TailArmMove(m_tailArm, Constants.Setpoints.tailArmHorizontal));
    new JoystickButton(rightJoystick, 9).whileTrue(new TailArmMove(m_tailArm, Constants.Setpoints.tailArmScore));
    new JoystickButton(rightJoystick, 8).whileTrue(new TailArmMove(m_tailArm, Constants.Setpoints.tailArmStartingAngle));


    // Tail Intake
    new JoystickAnalogButton(otherJoystick, 3, 0.5).whileTrue(new TailIntakeMove(m_tailIntake, 1));
    new JoystickAnalogButton(otherJoystick, 4, 0.5).whileTrue(new TailIntakeMove(m_tailIntake, -1));


    // CLIMB SETPOINT
    // new JoystickButton(otherJoystick, 1).whileTrue(new ClimbMove(m_climb, 29));
    // new JoystickButton(otherJoystick, 2).whileTrue(new ClimbMove(m_climb, 145));

    // CLIMB DIRECT MOVE
    new JoystickButton(otherJoystick, 1).whileFalse(new ParallelCommandGroup(
      new ClimbDirectMove(m_climb, true, 135),
      new ClimbDirectMove(m_climb, false, 160)
    ));

    // INTAKE
    new JoystickButton(rightJoystick, 1).whileTrue(new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayPickup));
    
    // MANUAL CLIMB
    // new JoystickButton(otherJoystick, 3).whileTrue(new MotorMoveTesting(climbinner, 0.35));
    // new JoystickButton(otherJoystick, 4).whileTrue(new MotorMoveTesting(climbinner, -0.35));

    // new JoystickButton(otherJoystick, 7).whileTrue(new MotorMoveTesting(climbouter, 0.35));
    // new JoystickButton(otherJoystick, 8).whileTrue(new MotorMoveTesting(climbouter, -0.35));
  }

  private void registerPathplannerCommands() {
    // Register commands used in Pathplanner autos
    NamedCommands.registerCommand("tagPositionLeft", new DriveToAprilTag(m_robotDrive, Constants.AprilTags.coralOffsetLeft, "limelight-two", false));
    NamedCommands.registerCommand("tagPositionRight", new DriveToAprilTag(m_robotDrive, Constants.AprilTags.coralOffsetRight, "limelight-two", false));

    NamedCommands.registerCommand("armHigh", new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh));

    NamedCommands.registerCommand("dunkScoreL4", c_dunkScore);
  }

  private void setupPathplannerSelector() {
    // sets up the Smartdashboard dropdown selector to pick which autonomous to run
    
    // List all the autos from Pathplanner here:
    m_pathPlannerChooser.addOption("Pos testing", "Pos testing auto");
    m_pathPlannerChooser.addOption("Pos testing 2", "Pos testing 2");
    m_pathPlannerChooser.addOption("Pos testing 3", "Pos testing 3");
    m_pathPlannerChooser.addOption("Center Score", "Center Score");

    SmartDashboard.putData("Select Auto", m_pathPlannerChooser);

    System.out.println("select auto placed");
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
    // m_climb.updateSmartDashboard();
  }
}
