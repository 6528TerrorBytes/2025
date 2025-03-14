// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BlinkinCommand;
import frc.robot.commands.CancelDriveCommand;
import frc.robot.commands.DriveSpeedChange;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.DriveToAprilTagAuto;
import frc.robot.commands.DriveToAprilTagWPILib;
import frc.robot.commands.WPILibPath;
import frc.robot.commands.move.AlgaeForkMove;
import frc.robot.commands.move.ArmMove;
import frc.robot.commands.move.ClimbDirectMove;
import frc.robot.commands.move.ElevatorMove;
import frc.robot.commands.move.IntakeMove;
import frc.robot.commands.move.MotorMoveTesting;
import frc.robot.commands.move.TailArmMove;
// import frc.robot.commands.move.TailIntakeMove;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.IntakeMotor;
// import frc.robot.subsystems.TailIntake;
import frc.robot.subsystems.WPIPID.IntakeArm;
import frc.robot.subsystems.WPIPID.TailArm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.JoystickAnalogButton;
import frc.utils.JoystickMultiAnalogButton;
import frc.utils.JoystickMultiButton;
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
  // private final TailIntake m_tailIntake = new TailIntake();

  private final CoralDetector m_coralDetector = new CoralDetector();

  private final Blinkin m_blinkin = new Blinkin();
  private final BlinkinCommand m_blinkinCommand = new BlinkinCommand(m_blinkin, m_coralDetector);

  // Shared commands -- used in both joysticks and autons
  private Command c_dunkScore;

  private Command m_path; // for testing
  
  public RobotContainer() {
    configureSharedCommands();

    setDriveCommand();
    configureOFFICIALBindings();
    
    registerPathplannerCommands();
    setupPathplannerSelector();

    m_blinkin.setDefaultCommand(m_blinkinCommand);

    WPILibPath.warmupTrajectoryGeneration();
    System.out.println("trajectory warmup finished");
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
        new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk, true),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      ),
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero)
    );
  }
  
  private void configureOFFICIALBindings() {
    // ---- DRIVER BINDINGS ----

    // we drunk driving bois :tada:
    
    // Drive to score coral, left and right sides, L3 or L4
    new JoystickButton(leftJoystick, 1).whileTrue(new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralOffsetLeft, Constants.AprilTags.coralXTagOffset, "limelight-two", false, true));
    new JoystickButton(rightJoystick, 1).whileTrue(new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralOffsetRight, Constants.AprilTags.coralXTagOffset, "limelight-two", false, true));
    
    // Drive to score L2
    new JoystickButton(leftJoystick, 2).whileTrue(new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralOffsetLeftLow, Constants.AprilTags.coralXTagOffset, "limelight-two", false, true));
    new JoystickButton(rightJoystick, 2).whileTrue(new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralOffsetRightLow, Constants.AprilTags.coralXTagOffset, "limelight-two", false, true));
    
    // Drive to centered for algae
    new JoystickButton(rightJoystick, 3).whileTrue(new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralOffsetCentered, 0, "limelight-two", false, true));
    
    // Drive to intake and pickup
    // new JoystickButton(leftJoystick, 4).whileTrue(new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralCollectOffset, 0, "limelight-four", true));
    new JoystickButton(leftJoystick, 4).whileTrue(new CancelDriveCommand(m_robotDrive));

    // REZERO the bot right button thingy
    new JoystickButton(rightJoystick, 4).whileTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
    
    // Reverse intake
    new JoystickButton(leftJoystick, 3).whileTrue(new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk, true));
    
    // ---- SCORE CORAL, ALL LEFT TRIGGER (2) ----

    // Score L4 button Y
    new JoystickMultiAnalogButton(otherJoystick, 2, 0.3, 4).whileTrue(c_dunkScore);
  
    // Score L3 button X
    new JoystickMultiAnalogButton(otherJoystick, 2, 0.3, 3).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorScoreMiddle),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleMiddleHigh)
      ),
      new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk, true),
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      )
    ));

    // Score L2 button B
    new JoystickMultiAnalogButton(otherJoystick, 2, 0.3, 2).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorScoreLow),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleL2)
      ),
      new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk, true),
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      )
    ));

    // Score L1 button A
    new JoystickMultiAnalogButton(otherJoystick, 2, 0.3, 1).whileTrue(new SequentialCommandGroup(
      // new ParallelCommandGroup(
      //   new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorScoreLow),
      //   new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleL1Stage1)
      // ),
      // new ParallelCommandGroup(
      //   new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk, true),
      //   new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleL1Stage2)
      // ),
      // new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero)

      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleL1Stage1)
      ),
      new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk, false)
    ));



    // ---- PICKUP ALGAE, ALL RIGHT TRIGGER (3)

    // HOME ALGAE HIGH button Y 
    new JoystickMultiAnalogButton(otherJoystick, 3, 0.3, 4).whileTrue(new ParallelCommandGroup(
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabHigh),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh),
      new AlgaeForkMove(m_algaeFork, m_elevator, Constants.Setpoints.algaeForkHorizontal - 5)
    ));

    // PICKUP ALGAE HIGH button X
    new JoystickMultiAnalogButton(otherJoystick, 3, 0.3, 3).whileTrue(new ParallelCommandGroup(
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHoldAlgae),

      new SequentialCommandGroup(
        new WaitCommand(0.35),
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabHigh + Constants.Setpoints.elevatorPickupMoveUp)
      )
    ));

    // HOME ALGAE LOW button B
    new JoystickMultiAnalogButton(otherJoystick, 3, 0.3, 2).whileTrue(new ParallelCommandGroup(
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabLow),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh),
      new AlgaeForkMove(m_algaeFork, m_elevator, Constants.Setpoints.algaeForkHorizontal - 5)
    ));

    // PICKUP ALGAE LOW button A
    new JoystickMultiAnalogButton(otherJoystick, 3, 0.3, 1).whileTrue(new ParallelCommandGroup(
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHoldAlgae),

      new SequentialCommandGroup(
        new WaitCommand(0.35),
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorGrabLow + Constants.Setpoints.elevatorPickupMoveUp)
      )
    ));



    // ---- MISC CONTROLS, LEFT BUMPER (5) ----

    // ELEVATOR DOWN button A
    new JoystickMultiButton(otherJoystick, 5, 1).whileTrue(new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero));
    // ARM HIGH button Y
    new JoystickMultiButton(otherJoystick, 5, 4).whileTrue(new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh));    
    
    // INTAKE POSITION button X
    new JoystickMultiButton(otherJoystick, 5, 3).onTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero),
        new TailArmMove(m_tailArm, Constants.Setpoints.tailArmPreintake)
      ),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleStore),
      new TailArmMove(m_tailArm, Constants.Setpoints.tailArmIntake)
    ));

    // INTAKE RUN button B
    new JoystickMultiButton(otherJoystick, 5, 2).whileTrue(new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayPickup, false));

    // ARM DOWN ALL THE WAY button START
    new JoystickButton(otherJoystick, 8).whileTrue(new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleStore));


    // ---- STINGRAY, RIGHT BUMPER (6) ----

    // TAIL ARM INTAKE button A
    // new JoystickMultiButton(otherJoystick, 6, 1).onTrue();

    // TAIL ARM SCORE POS button Y
    new JoystickMultiButton(otherJoystick, 6, 4).onTrue(new ParallelCommandGroup(
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armElevatorMoveAngle + Constants.MotorConfig.armTolerance),
      new TailArmMove(m_tailArm, Constants.Setpoints.tailArmUp)
    ));

    // CLIMB button
    new JoystickMultiButton(otherJoystick, 6, 1).whileFalse(new ParallelCommandGroup(
      new ClimbDirectMove(m_climb, true, 180),
      new ClimbDirectMove(m_climb, false, 160)
    ));


    // ---- OTHER ----

    // RESET ALGAE FLAP back button
    new JoystickButton(otherJoystick, 7).whileTrue(new SequentialCommandGroup(
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorAlgaeFlapMovePos + 2 * Constants.MotorConfig.elevatorTolerance),
      new AlgaeForkMove(m_algaeFork, m_elevator, Constants.Setpoints.algaeForkZero),
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero)
    ));

  }

  private void registerPathplannerCommands() {
    // Register commands used in Pathplanner autos

    // Auto moves
    NamedCommands.registerCommand("tagPositionLeft", new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralOffsetLeft, Constants.AprilTags.coralXTagOffset, "limelight-two", false, false));
    NamedCommands.registerCommand("tagPositionRight", new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralOffsetRight, Constants.AprilTags.coralXTagOffset, "limelight-two", false, false));
    
    // NamedCommands.registerCommand("tagPositionPickup", new DriveToAprilTagWPILib(m_robotDrive, Constants.AprilTags.coralCollectOffset, 0, "limelight-four", true));


    // Arm/elevator/intake motors
    NamedCommands.registerCommand("armHigh", new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh));
    NamedCommands.registerCommand("armHigh", new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh));
    NamedCommands.registerCommand("runIntakeMotor", new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayPickup, false));

    // Scoring 
    NamedCommands.registerCommand("preL4", new ParallelCommandGroup(
      new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorHigh),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
    ));

    NamedCommands.registerCommand("dunkScoreL4", new SequentialCommandGroup(
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHorizontal),
      new ParallelCommandGroup(
        new IntakeMove(m_intakeMotor, m_coralDetector, Constants.Setpoints.m_intakeMotorStopDelayDunk, true),
        new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleHigh)
      )
    ));

    NamedCommands.registerCommand("elevatorZero", new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero));

    NamedCommands.registerCommand("intakePosition", new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorMove(m_elevator, m_arm, Constants.Setpoints.elevatorZero),
        new TailArmMove(m_tailArm, Constants.Setpoints.tailArmPreintake)
      ),
      new ArmMove(m_arm, m_elevator, Constants.Setpoints.armAngleStore),
      new TailArmMove(m_tailArm, Constants.Setpoints.tailArmIntake)
    ));
  }

  private void setupPathplannerSelector() {
    // sets up the Smartdashboard dropdown selector to pick which autonomous to run
    
    // List all the autos from Pathplanner here:
    m_pathPlannerChooser.addOption("None", null);
    m_pathPlannerChooser.addOption("Just Move", "Just Move");
    m_pathPlannerChooser.addOption("Center Score 1", "Center Score 1");
    m_pathPlannerChooser.addOption("Right Score 2", "Right Score 2");
    m_pathPlannerChooser.addOption("Left Score 2", "Left Score 2");

    SmartDashboard.putData("Select Auto", m_pathPlannerChooser);

    System.out.println("select auto placed");
  }

  public Command getAutonomousCommand() {
    String selectedAuto = m_pathPlannerChooser.getSelected();

    if (selectedAuto == null) {
      return new InstantCommand(() -> {});
    }

    PathPlannerAuto auto = new PathPlannerAuto(selectedAuto);
    m_robotDrive.resetOdometry(auto.getStartingPose()); // Sets the odometry to the starting position of the robot in the autonomous
    return auto;
  }

  public void updateSmartDashboard() {
    m_robotDrive.updateSmartDashboard();
    SmartDashboard.putBoolean("LL2", Utility.aprilTagInView("limelight-two"));
    SmartDashboard.putBoolean("LL4", Utility.aprilTagInView("limelight-four"));
    SmartDashboard.putNumber("elevator", m_elevator.m_encoder.getPosition());
    SmartDashboard.putBoolean("Coral Detected", m_coralDetector.detected());
    SmartDashboard.putNumber("Arm Encoder", m_arm.m_encoder.getPosition());
    // SmartDashboard.putNumber("Algae Forks", m_algaeFork.m_encoder.getPosition());
    SmartDashboard.putNumber("Gyro", m_robotDrive.getRawAngle());
    SmartDashboard.putNumber("odometry rotation", m_robotDrive.getPose().getRotation().getDegrees());
    // m_climb.updateSmartDashboard();
  }
}
