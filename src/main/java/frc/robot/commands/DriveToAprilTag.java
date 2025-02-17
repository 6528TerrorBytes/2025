// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToAprilTag extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final boolean m_leftSide; // Left or right side of the coral to go to
  
  private boolean m_foundTag;
  private Command m_path;

  public DriveToAprilTag(DriveSubsystem driveSubsystem, boolean leftSide) {
    m_driveSubsystem = driveSubsystem;
    m_leftSide = leftSide;
    m_foundTag = false;

    addRequirements(driveSubsystem); // Prevents from driving while the path is active
  }

  // Checking for the AprilTag ID corresponding to the current team color
  public static boolean coralTagInView() {
    return (Utility.aprilTagInView() &&
      (
        (Utility.teamColorIsRed() && Utility.aprilTagIDIsInList(Constants.AprilTags.coralRedTags)) ||
        (Utility.aprilTagIDIsInList(Constants.AprilTags.coralBlueTags))
      )
    );
  }

  // Finds the field position of the robot facing the AprilTag, lined up to the coral.
  // MATHS DESMOS: https://www.desmos.com/calculator/uagr4pd9gv
  public static Pose2d findGoalPos(Pose2d robotPos, Pose2d aprilTagPos, boolean leftSide) {
    double robotRot = robotPos.getRotation().getRadians();
    double faceTagAngle = robotRot - aprilTagPos.getRotation().getRadians(); // robotRot - tagRot, finds angle to face the AprilTag
    
    // Calculate the AprilTag's position on the field
    Translation2d tagFieldPos = new Translation2d(
      // X = (tagX * cos(robotRot)) + (tagY * cos(robotRot - pi/2))
      (aprilTagPos.getX() * Math.cos(robotRot))  +  (aprilTagPos.getY() * Math.cos(robotRot - (Math.PI / 2))),
      (aprilTagPos.getX() * Math.sin(robotRot))  +  (aprilTagPos.getY() * Math.sin(robotRot - (Math.PI / 2)))
    );

    double offsetHoriz = Constants.AprilTags.coralOffset.getX();
    double offsetOut = Constants.AprilTags.coralOffset.getY();

    if (leftSide) {
      offsetHoriz *= -1; // Flip to other side of the AprilTag
    }

    // Add offsets to find the position of the robot on the field next to the AprilTag
    Translation2d finalGoalPos = new Translation2d(
      // X = tagFieldX + (offsetX * Math.cos(faceAngle - PI/2)) + (offsetY * Math.cos(faceAngle - PI/2))
      tagFieldPos.getX()  +  (offsetHoriz * Math.cos(faceTagAngle - (Math.PI / 2)))  +  (offsetOut * Math.cos(faceTagAngle - Math.PI)),
      tagFieldPos.getY()  +  (offsetHoriz * Math.sin(faceTagAngle - (Math.PI / 2)))  +  (offsetOut * Math.sin(faceTagAngle - Math.PI))
    );

    return new Pose2d(finalGoalPos, Rotation2d.fromRadians(faceTagAngle));
  }

  @Override
  public void execute() {
    if (!m_foundTag) {
      if (!coralTagInView()) { return; } // Exit if no AprilTag in view

      m_foundTag = true;

      Pose2d robotPos = m_driveSubsystem.getPose();
      Pose3d tagBotSpace = Utility.getTagPoseRelativeToBot();
      
      // Convert AprilTag Pose3d to Pose2d
      //                               TAG OUT DIST        TAG HORIZONTAL DIST
      Pose2d aprilTagPose = new Pose2d(tagBotSpace.getZ(), tagBotSpace.getX(), Rotation2d.fromRadians(tagBotSpace.getRotation().getY()));
    
      // Calculate goal pose
      Pose2d goalPos = findGoalPos(robotPos, aprilTagPose, m_leftSide);
  
      // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html:

      // Create path from current robot position to the new position

      // Angle pointing towards goal from the starting position:
      double angle = Math.atan((goalPos.getY() - robotPos.getY()) / (goalPos.getX() - robotPos.getX()));

      // ROTATIONS ARE PATH OF TRAVEL
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(robotPos.getTranslation(), Rotation2d.fromRadians(angle)), // starting pose with angle pointing towards goal
        goalPos
      );

      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        Constants.AprilTags.constraints, // really slow for testing purposes
        null, // May need to add a starting state for velocity & angle of the bot at the start of the pose
        new GoalEndState(0, goalPos.getRotation())
      );

      path.preventFlipping = true;
      m_path = AutoBuilder.followPath(path);
  
      // New based on https://pathplanner.dev/pplib-pathfinding.html#pathfind-to-pose
      // m_path = AutoBuilder.pathfindToPose(goalPos, Constants.AprilTags.constraints);
      // m_path.initialize();
    }

    // Basically just wrapping m_path in this external command now.
    m_path.execute();
  }

  @Override
  public void end(boolean interrupted) {
    if (m_foundTag) {
      m_path.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return m_foundTag && m_path.isFinished();
  }
}
