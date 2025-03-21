// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers.PoseEstimate;

// The data you can get from the Limelight: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

public final class Utility {
  public static boolean teamColorIsRed() {
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red; 
  }
  
  // System clock time in seconds
  public static double getTime() {
    return Timer.getFPGATimestamp();
  }

  public static double getMatchTime() {
    return Timer.getMatchTime();
  }
  
  public static boolean aprilTagInView(String limelight) {
    return LimelightHelpers.getTV(limelight);
  }

  public static double getAprilTagID(String limelight) {
    return LimelightHelpers.getFiducialID(limelight);
  }
  
  public static boolean aprilTagIDIsInList(String limelight, double[] list) {
    double currentAprilTagID = getAprilTagID(limelight);

    for (double id : list)
      if (currentAprilTagID == id)
        return true;

    return false;
  }

  // Call this before using the robot field pose with MegaTag2
  public static void setRobotOrientation(String limelight, double rotation) {
    LimelightHelpers.SetRobotOrientation(limelight, rotation, 0, 0, 0, 0, 0);
  }

  // All of the different data you can get from the Limelight: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
  
  // Bottom left of field is (0, 0). X is long-side distance, Y is short-side distance: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
  public static PoseEstimate getRobotFieldPose(String limelight) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight); 
  }

  // Z means distance out, X positive is camera-out right, while negative means left. Y is height
  // Rotation Y is horizontal rotation. Positive Y means the right side of the tag is closest to the robot (tag rotating clockwise)
  public static Pose3d getTagPoseRelativeToBot(String limelight) {
    return LimelightHelpers.getTargetPose3d_RobotSpace(limelight);
  }

  public static void turnOnLimelightLED() {
    LimelightHelpers.setLEDMode_ForceOn("limelight-two");
  }

  public static void turnOffLimelightLED() {
    LimelightHelpers.setLEDMode_ForceOff("limelight-two");
  }
}
