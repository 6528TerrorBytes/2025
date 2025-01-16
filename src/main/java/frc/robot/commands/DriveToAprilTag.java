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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.DriveSubsystem;

// Put command on a "whileTrue" button, otherwise driver will have no way to cancel the command
public class DriveToAprilTag extends Command {
  private final DriveSubsystem m_driveSubsystem;

  public DriveToAprilTag(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
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
    // Find the goal position, relative to the AprilTag's position (which is in relation to the bot, not the field)
    Pose2d aprilTagBotSpace = Utility.getTagPoseRelativeToBot();
    // Add offset from AprilTag pose to the destination position
    
    // TODO
    

    // Create path
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      m_driveSubsystem.getPose()

    );

    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      new PathConstraints(0.2,  0.2, 0.2, 0.2), // really slow for testing purposes
      null,
      new GoalEndState(
        0,
        Rotation2d.fromDegrees(m_driveSubsystem.getAngleBlueSide()).plus(aprilTagBotSpace.getRotation()) // no idea if this will work :O
      )
    );

    path.preventFlipping = true;
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
