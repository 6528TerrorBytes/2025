// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.FollowPathCommand; // This is the path command thing

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToAprilTag extends Command {
  protected final DriveSubsystem m_driveSubsystem;
  protected final Translation2d m_posOffset; // Left or right side of the coral to go to
  protected final String m_limelightName;
  
  protected boolean m_isPickupStation;
  protected double m_xOffset;

  protected boolean m_foundTag;
  // private Command m_path;

  public DriveToAprilTag(DriveSubsystem driveSubsystem, Translation2d posOffset, double xOffset, String limelightName, boolean isPickupStation) {
    m_driveSubsystem = driveSubsystem;
    m_posOffset = posOffset;
    m_xOffset = xOffset;
    m_foundTag = false;
    m_limelightName = limelightName;
    m_isPickupStation = isPickupStation;

    addRequirements(driveSubsystem); // Prevents from driving while the path is active
  }

  // Checking for the AprilTag ID corresponding to the current team color
  public static double coralTagInView(String limelightName) {
    if ((Utility.aprilTagInView(limelightName) && (
        (Utility.teamColorIsRed() && Utility.aprilTagIDIsInList(limelightName, Constants.AprilTags.coralRedTags)) ||
        (Utility.aprilTagIDIsInList(limelightName, Constants.AprilTags.coralBlueTags)))))
      return Utility.getAprilTagID(limelightName);
    return -1;
  }

  public static double stationTagInView(String limelightName) {
    if ((Utility.aprilTagInView(limelightName) && (
        (Utility.teamColorIsRed() && Utility.aprilTagIDIsInList(limelightName, Constants.AprilTags.stationRedTags)) ||
        (Utility.aprilTagIDIsInList(limelightName, Constants.AprilTags.stationBlueTags)))))
      return Utility.getAprilTagID(limelightName);
    return -1;
  }

  // Finds the field position of the robot facing the AprilTag, lined up to the coral.
  // MATHS DESMOS: https://www.desmos.com/calculator/uagr4pd9gv
  public static Pose2d findGoalPos(Pose2d robotPos, Pose2d aprilTagPos, Translation2d posOffset, double xOffset, double aprilTagId) {
    double robotRot = robotPos.getRotation().getRadians();
    double faceTagAngle = robotRot - aprilTagPos.getRotation().getRadians(); // robotRot - tagRot, finds angle to face the AprilTag
    // double faceTagAngle = Math.toRadians(Constants.AprilTags.aprilTagFaceAngles.get(aprilTagId)); // RADIANS NOT DEGREES >:( direct gyro reading might not be accurate
    
    // Calculate the AprilTag's position on the field
    Translation2d tagFieldPos = new Translation2d(
      // X = (tagX * cos(robotRot)) + (tagY * cos(robotRot - pi/2))
      (aprilTagPos.getX() * Math.cos(robotRot))  +  (aprilTagPos.getY() * Math.cos(robotRot - (Math.PI / 2))),
      (aprilTagPos.getX() * Math.sin(robotRot))  +  (aprilTagPos.getY() * Math.sin(robotRot - (Math.PI / 2)))
    );

    double offsetHoriz = posOffset.getX();
    double offsetOut = posOffset.getY();

    // if (leftSide) {
    //   offsetHoriz *= -1; // Flip to other side of the AprilTag
    // }

    offsetHoriz += xOffset;

    // Add offsets to find the position of the robot on the field next to the AprilTag
    Translation2d finalGoalPos = new Translation2d(
      // X = tagFieldX + (offsetX * Math.cos(faceAngle - PI/2)) + (offsetY * Math.cos(faceAngle - PI/2))
      tagFieldPos.getX()  +  (offsetHoriz * Math.cos(faceTagAngle - (Math.PI / 2)))  +  (offsetOut * Math.cos(faceTagAngle - Math.PI)),
      tagFieldPos.getY()  +  (offsetHoriz * Math.sin(faceTagAngle - (Math.PI / 2)))  +  (offsetOut * Math.sin(faceTagAngle - Math.PI))
    );

    // System.out.println((finalGoalPos.getX() - tagFieldPos.getX()) + ", " + (finalGoalPos.getY() - tagFieldPos.getY()));

    // System.out.println(aprilTagPos.getX() + ", " + aprilTagPos.getY() + " | " + faceTagAngle + ", " + faceTagAngle);

    return new Pose2d(finalGoalPos, Rotation2d.fromRadians(faceTagAngle));
  }

  public static Pose2d calculateGoalPos(Pose2d robotPos, Translation2d posOffset, double xOffset, String limelightName, double aprilTagID) {
    Pose3d tagBotSpace = Utility.getTagPoseRelativeToBot(limelightName);
    
    // Convert AprilTag Pose3d to Pose2d
    //                               TAG OUT DIST        TAG HORIZONTAL DIST
    Pose2d aprilTagPose = new Pose2d(tagBotSpace.getZ(), tagBotSpace.getX(), Rotation2d.fromRadians(tagBotSpace.getRotation().getY()));
  
    // Calculate goal pose
    return findGoalPos(robotPos, aprilTagPose, posOffset, xOffset, aprilTagID);
  }

  @Override
  public void initialize() {
    System.out.println("initializing again");
    // m_path = null;
    m_foundTag = false;
  }

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
      Pose2d robotPos = m_driveSubsystem.getPose();

      Pose2d goalPos = calculateGoalPos(robotPos, m_posOffset, m_xOffset, m_limelightName, aprilTagID);

      // Add goal pose to robot pos
      goalPos = new Pose2d(goalPos.getX() + robotPos.getX(), goalPos.getY() + robotPos.getY(), goalPos.getRotation());

      // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html:

      // Create path from current robot position to the new position

      // Angle pointing towards goal from the starting position:
      double angle;
      if ((goalPos.getX() - robotPos.getX()) == 0) {
        angle = Math.atan((goalPos.getY() - robotPos.getY()) / 0.0001);
      } else {
        angle = Math.atan((goalPos.getY() - robotPos.getY()) / (goalPos.getX() - robotPos.getX()));
      }

      // ROTATIONS ARE PATH OF TRAVEL
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(robotPos.getTranslation(), Rotation2d.fromRadians(angle)), // starting pose with angle pointing towards goal
        goalPos
      );

      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        Constants.AprilTags.aprilTagDriveConstraints, // really slow for testing purposes
        null, // May need to add a starting state for velocity & angle of the bot at the start of the pose
        new GoalEndState(0, goalPos.getRotation())
      );

      path.preventFlipping = true;
      // m_path = AutoBuilder.followPath(path);
      
      // New based on https://pathplanner.dev/pplib-pathfinding.html#pathfind-to-pose
      // m_path = AutoBuilder.pathfindToPose(goalPos, Constants.AprilTags.constraints);

      // m_path.initialize();
      // m_path.schedule(); 

      AutoBuilder.followPath(path).schedule();
      
      // DriveSubsystem.m_controllersDisabled = true;
    } /*else {
      // Basically just wrapping m_path in this external command now lol
      // m_path.execute();
    }*/
  }

  // @Override
  // public void end(boolean interrupted) {
  //   System.out.println("ended start1");
  //   // try {
  //   //   System.out.println("ended start inner");
  //   //   m_path.end(false);
  //   //   m_path.cancel();
  //   //   System.out.println("ended command not null");
  //   // } catch (java.lang.NullPointerException e) {
  //   //   System.out.println("ended null");
  //   // }

  //   // System.out.println("ended path function successful");
    
  //   // DriveSubsystem.m_controllersDisabled = false;
  // }

  // @Override
  // public boolean isFinished() {
  //   if (!m_doIsFinished) return false;

  //   // try {
  //   //   System.out.println("isFinished start");
  //   //   if (m_foundTag && m_path != null && m_path.isFinished()) {
  //   //     System.out.println("isFinished ended successful1");
  //   //     return true;
  //   //   }
  //   //   System.out.println("isFinished ended successful2");
  //   // } catch (java.lang.NullPointerException e) {
  //   //   System.out.println("path not set up (isFinished)");
  //   //   return false;
  //   // }
    
  //   System.out.println("IsFinished done");

  //   System.out.println("g - x: " + m_goal.getX() + " | y: " + m_goal.getY() + " | rot: " + m_goal.getRotation().getDegrees());
  //   System.out.println("s - x: " + m_start.getX() + " | y: " + m_start.getY() + " | rot: " + m_start.getRotation().getDegrees());

  //   return false;
  // }
}
