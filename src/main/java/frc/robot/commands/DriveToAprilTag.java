// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.DriveSubsystem;

// Put command on a "whileTrue" button, otherwise driver will have no way to cancel the command
public class DriveToAprilTag extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final int m_side;

  // 0 is left side, 1 is right side
  public DriveToAprilTag(DriveSubsystem driveSubsystem, int side) {
    m_driveSubsystem = driveSubsystem;
    m_side = side;
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

  @Override
  public void initialize() {
    Pose2d currentRobotPose = m_driveSubsystem.getPose();
    
    // Find the goal position, relative to the AprilTag's position (which is in relation to the bot, not the field)
    Pose3d aprilTagBotSpace = Utility.getTagPoseRelativeToBot();
    double rotation = aprilTagBotSpace.getRotation().getY(); // Degrees, where positive means tag is rotating clockwise
    // Robot field rotation is positive = counterclockwise...

    // Find field-relative april tag rotation (bot rotation + apriltag rotation)
    Rotation2d aprilTagAngle = Rotation2d.fromDegrees(currentRobotPose.getRotation().getDegrees() + aprilTagBotSpace.getRotation().getDegrees());

    // Goal pose becomes coords on field where the AprilTag is
    Pose2d goalPose = Utility.addPosesAtAngle(currentRobotPose, aprilTagBotSpace, currentRobotPose.getRotation());

    // Add position goal offset to april tag location 
    // Pose2d offset = Constants.AprilTags.coralOffsets.get(m_side);
    // Pose2d finalGoal = Utility.addPosesAtAngle(currentRobotPose, offset, aprilTagAngle);
    
    // Create path
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      m_driveSubsystem.getPose()

    );

    // PathPlannerPath path = new PathPlannerPath(
    //   waypoints,
    //   new PathConstraints(0.2,  0.2, 0.2, 0.2), // really slow for testing purposes
    //   null,
    //   new GoalEndState(
    //     0,
    //     Rotation2d.fromDegrees(m_driveSubsystem.getAngleBlueSide()).plus(aprilTagBotSpace.getRotation()) // no idea if this will work :O
    //   )
    // );

    // path.preventFlipping = true;

    // Display trajectory via SmartDashboard?
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
