// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

// Referencing: https://github.com/6528TerrorBytes/2024/blob/f8164e7a2b333dee86592a26a9f2aa825ace3897/src/main/java/frc/robot/commands/auton/AutonPaths.java
// and "FRC 0 to Autonomous" on YouTube
// see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
public final class WPILibPath {
  private static final TrajectoryConfig trajConfig = new TrajectoryConfig(
    Constants.AprilTags.aprilTagDriveConstraints.maxVelocity(),
    Constants.AprilTags.aprilTagDriveConstraints.maxAcceleration()
  ).setKinematics(Constants.DriveConstants.kDriveKinematics);

  private static final TrapezoidProfile.Constraints thetaConfig = new TrapezoidProfile.Constraints(
    Constants.AprilTags.aprilTagDriveConstraints.maxAngularVelocityRadPerSec(),
    Constants.AprilTags.aprilTagDriveConstraints.maxAngularAccelerationRadPerSecSq()
  );

  public static SwerveControllerCommand genSwerveCommand(Trajectory trajectory, DriveSubsystem robotDrive, boolean addReq) {
    // PID controllers used for following the trajectory (correcting errors)
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    // Angle correction PID Controller
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, thetaConfig);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Maybe these above controllers should be single constants used for all paths generated?

    System.out.println("Making wpilib path command 1");
    // Handles the swerve trajectory stuff
    return new SwerveControllerCommand(
      trajectory,
      robotDrive::getPose,
      DriveConstants.kDriveKinematics,
      xController, yController,
      thetaController,
      robotDrive::setModuleStates,
      (addReq ? robotDrive : null)
    );
  }

  // Quintic spline, for angles at all interior points
  public static Trajectory genTrajectory(List<Pose2d> points) {
    System.out.println("Generating trajectory 1");
    return TrajectoryGenerator.generateTrajectory(points, trajConfig);
  }

  // Cubic spline, angles at interior points automatically determined
  public static Trajectory genTrajectory(Pose2d starting, List<Translation2d> points, Pose2d ending) {
    return TrajectoryGenerator.generateTrajectory(starting, points, ending, trajConfig);
  }

  // Run once after robot initialization
  // This loads the trajectory generator into memory, eliminating the initial delay of generating the first trajectory
  public static Trajectory warmupTrajectoryGeneration() {
    return genTrajectory(List.of(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(1, 1, Rotation2d.fromDegrees(90))
    ));
  }
}
