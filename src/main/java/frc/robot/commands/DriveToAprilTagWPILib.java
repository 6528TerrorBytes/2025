// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

// Overrides isFinished and end, has a bug where the robot will sometimes randomly die upon finishing the path in teleop (???)
public class DriveToAprilTagWPILib extends DriveToAprilTag {
  Command m_path;

  /** Creates a new DriveToAprilTagAuto. */
  public DriveToAprilTagWPILib(DriveSubsystem driveSubsystem, Translation2d posOffset, double xOffset, String limelightName, boolean isPickupStation, boolean addReq) {
    super(driveSubsystem, posOffset, xOffset, limelightName, isPickupStation, addReq);
  }

  @Override
  public void initialize() {
    m_path = null;
    m_foundTag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_foundTag) {
      double aprilTagID = (m_isPickupStation ? stationTagInView(m_limelightName) : coralTagInView(m_limelightName));
      if (aprilTagID < 0) {
        System.out.println("not found");
        return; // exit if no apriltag found
      }

      System.out.println("found");

      m_foundTag = true;
      Pose2d robotPos = m_driveSubsystem.getPose(); // ** IF DOES NOT WORK, CHANGE THIS TO GET THE DIRECT ODOMETRY POSE? **

      Pose2d goalPos = calculateGoalPos(robotPos, m_posOffset, m_xOffset, m_limelightName, aprilTagID);

      // Add goal pose to robot pos
      goalPos = new Pose2d(goalPos.getX() + robotPos.getX(), goalPos.getY() + robotPos.getY(), goalPos.getRotation());

      // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html:

      // Create path from current robot position to the new position

      // Angle pointing towards goal from the starting position:
      
      // double angle = Math.atan((goalPos.getY() - robotPos.getY()) / (goalPos.getX() - robotPos.getX()));
      double angle;
      if ((goalPos.getX() - robotPos.getX()) == 0) {
        angle = Math.atan((goalPos.getY() - robotPos.getY()) / 0.0001);
        System.out.println("x dist is zero");
      } else {
        angle = Math.atan((goalPos.getY() - robotPos.getY()) / (goalPos.getX() - robotPos.getX()));
      }

      System.out.println("wpilib trajectory generating...");
      Trajectory traj = WPILibPath.genTrajectory(List.of(
        new Pose2d(robotPos.getTranslation(), Rotation2d.fromRadians(angle)), // starting pose with angle pointing towards goal
        // robotPos, // do this instead to start with the direction of movement?
        goalPos
      ));
      System.out.println("wpilib trajectory done!");

      System.out.println("wpilib path generating...");
      m_path = WPILibPath.genSwerveCommand(traj, m_driveSubsystem);
      System.out.println("wpilib path done!");
      m_path.schedule(); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended start1");
    try {
      System.out.println("ended start inner");

      m_path.end(false);
      m_path.cancel();
      m_driveSubsystem.setX();

      System.out.println("ended command not null");
    } catch (java.lang.NullPointerException e) {
      System.out.println("ended null");
    }

    System.out.println("ended path function successful");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    try {
      System.out.println("isFinished start");
      if (m_foundTag && m_path != null && m_path.isFinished()) {
        System.out.println("isFinished ended successful1");
        return true;
      }
      System.out.println("isFinished ended successful2");
    } catch (java.lang.NullPointerException e) {
      System.out.println("path not set up (isFinished)");
      return false;
    }

    return false;
  }
}
