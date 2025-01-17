// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers.PoseEstimate;

// The data you can get from the Limelight: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

public final class Utility {
  public static boolean teamColorIsRed() {
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red; 
  }

  public static double getMatchTime() {
    return Timer.getMatchTime();
  }
  
  public static boolean aprilTagInView() {
    return LimelightHelpers.getTV("limelight");
  }

  public static double getAprilTagID() {
    return LimelightHelpers.getFiducialID("limelight");
  }
  
  public static boolean aprilTagIDIsInList(double[] list) {
    double currentAprilTagID = getAprilTagID();

    for (double id : list)
      if (currentAprilTagID == id)
        return true;

    return false;
  }

  // Call this before using the robot field pose with MegaTag2
  public static void setRobotOrientation(double rotation) {
    LimelightHelpers.SetRobotOrientation("limelight", rotation, 0, 0, 0, 0, 0);
  }

  // The data you can get from the Limelight: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
  public static PoseEstimate getRobotFieldPose() {
    // Flip rotation measurement depending on team color?
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight"); 
  }

  public static Pose2d getTagPoseRelativeToBot() {
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight").toPose2d();
  }

  public static Pose2d addDistanceAtAngle(Pose2d pose, double dist, double theta) {
    Translation2d translate = new Translation2d(
      Math.cos(Math.toRadians(theta)) * dist,
      Math.sin(Math.toRadians(theta)) * dist
    );

    pose.getTranslation().plus(translate);
    return pose;
  }

  // Assuming x means the distance out, and y is the distance left/right from the direction
  public static Pose2d addPosesAtAngle(Pose2d pose1, Pose2d pose2, Rotation2d theta) {
    pose1 = addDistanceAtAngle(pose2, pose2.getX(), theta.getDegrees());
    pose1 = addDistanceAtAngle(pose2, pose2.getY(), theta.getDegrees() - 90); // Might be positive 90??

    return pose1;
  }
}
