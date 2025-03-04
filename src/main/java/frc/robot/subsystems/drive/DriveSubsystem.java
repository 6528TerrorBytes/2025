// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
// Gyro NAVX:
import com.studica.frc.AHRS; // EXAMPLE NAVX CODE: https://pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/ and https://www.chiefdelphi.com/t/navx-vendordeps/478142/3
import com.studica.frc.AHRS.NavXComType;
import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;
import com.pathplanner.lib.auto.AutoBuilder; // Pathplanner: https://pathplanner.dev/pplib-getting-started.html#install-pathplannerlib
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import frc.utils.SwerveUtils;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.DriveToAprilTag;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.subsystems.MAXSwerveModule;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset
  );

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset
  );

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset
  );

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset
  );

  // The gyro sensor
  private final AHRS m_navX = new AHRS(NavXComType.kMXP_SPI); // (May need to change this: NavXUpdateRate.k200Hz)
  private double m_rotationOffset; // for starting the navX at a different angle than it really is

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public static double speedMultiplier = 1;

  // Field object for Shuffleboard
  private final Field2d m_field = new Field2d();
  private final Field2d m_field2 = new Field2d();

  // Odometry class for tracking robot pose
  // https://first.wpi.edu/wpilib/allwpilib/docs/beta/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator.html
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(getAngleBlueSide()),
    getModulePositions(),
    new Pose2d(),

    // TUNING THESE STANDARD DEVIATIONS: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html#tuning-pose-estimators
    // Making these too low will result in jittery pose estimation (see Smartdashboard for pose estimation location),
    // Whereas too high will result in the pose not adjusting for vision data quick enough. Adjust these as needed
    VecBuilder.fill(0.1, 0.1, 99999999), // x, y, rotation for encoder pos (I think)
    VecBuilder.fill(0.1, 0.1, 99999999) // x, y, rotation for vision data
  );

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Displayed pathplanner path on the field perhaps
    // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    //   // Do whatever you want with the pose here
    //   m_field.getObject("target pose").setPose(pose);
    // });

    m_rotationOffset = 0;
    
    // Auton setup for PathPlanner, see https://pathplanner.dev/pplib-getting-started.html#install-pathplannerlib

    // might not work :)
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();

      // Default, if not setup within Pathplanner's settings already
      config = new RobotConfig(74.088, 6.883, null);
    }

    AutoBuilder.configure(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeeds,
      (speeds, forwards) -> driveRobotRelative(speeds),
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config,
      Utility::teamColorIsRed,
      this
    );
    
    SmartDashboard.putData("field", m_field);
    SmartDashboard.putData("tagfield", m_field2);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  // Get rotation in degrees
  public double getRawAngle() {
    return -m_navX.getAngle() * 1.0307507301151; // Scale factor found by rotating the bot 10 times and seeing how much the gyro is off of.
  }

  // The angle that is stored in the robot's odometry (because that has to always be relative to the blue side)
  public double getAngleBlueSide() {
    return getRawAngle() + m_rotationOffset;
  }

  public double getOdometryRotation() {
    return getPose().getRotation().getDegrees() + (Utility.teamColorIsRed() ? 180 : 0);
  }

  @Override
  public void periodic() {
    // System.out.println("Drive train periodic is running (this println is for Pathplanner Apriltag pose estimation testing)");

    // Update the odometry in the periodic block
    m_odometry.updateWithTime(
      Utility.getMatchTime(),
      Rotation2d.fromDegrees(getAngleBlueSide()),
      getModulePositions()
    );

    // System.out.println("rawblue: " + getAngleBlueSide() + " | odometry: " + getOdometryRotation() + " | offset: " + m_rotationOffset);

    // Use Limelights to adjust odometry to find more accurate field position
    // incorporateVisionPose("limelight-two");
    // incorporateVisionPose("limelight-four");
    
    // Update SmartDashboard field position
    m_field.setRobotPose(getPose());
  }

  public void incorporateVisionPose(String limelightName) {
    Utility.setRobotOrientation(limelightName, getOdometryRotation());

    // Add vision measurement to odometry calculation if an AprilTag is visible
    // Example: https://www.chiefdelphi.com/t/introducing-megatag2-by-limelight-vision/461243
    // Documetation: https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
    //               https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-wpilibs-pose-estimator
    if (Utility.aprilTagInView(limelightName)) {
      System.out.println("Incorporating " + limelightName);

      PoseEstimate poseEstimate = Utility.getRobotFieldPose(limelightName);
      if (poseEstimate == null) return;

      // Decrease the first and second numbers to trust limelight data more
      // TUNING STD DEVIATIONS: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html#tuning-pose-estimators
      // Making these too low will result in jittery pose estimation, too high results in not adjusting for vision data too quickly. Adjust as needed
      m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1,9999999));
      m_odometry.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);


      // show the apriltag calculated position
      double tagID = DriveToAprilTag.coralTagInView("limelight-two");
      if (tagID >= 0) {
        Pose2d apriltagpose = DriveToAprilTag.calculateGoalPos(getPose(), Constants.AprilTags.coralOffsetLeft, Constants.AprilTags.coralXTagOffset, "limelight-two", tagID);

        // Add to robot's current field position
        Pose2d apriltagPlusBotPos = new Pose2d(apriltagpose.getX() + getPose().getX(), apriltagpose.getY() + getPose().getY(), apriltagpose.getRotation());

        m_field2.setRobotPose(apriltagPlusBotPos);
      }
    }
  }

  public void updateSmartDashboard() {

  }

  /**
   * Sets the displayed trajectory for the SmartDashboard/Glass.
   * Does not make the bot follow the trajectory.
   */
  public void setFieldTrajectory(Trajectory trajectory) {
    m_field.getObject("traj").setTrajectory(trajectory);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_rotationOffset = pose.getRotation().getDegrees() - getRawAngle();

    if (m_rotationOffset < 8) m_rotationOffset = 0;

    // System.out.println("RESET: " + m_rotationOffset + " | " + pose.getRotation().getDegrees() + " | " + getRawAngle());

    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getAngleBlueSide()),
      getModulePositions(),
      pose
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean multSpeed) {
    SmartDashboard.putNumber("Gyro angle", getRawAngle());

    if (multSpeed) {
      // Speed multiplier
      xSpeed *= speedMultiplier;
      ySpeed *= speedMultiplier;
      rot *= speedMultiplier;
    }
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;

      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    // coding and stuff and things about the passage of time (Aedan was here :D)
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getOdometryRotation()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  
  // Get chassis speeds, robot relative
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    );
  }

  // Sets the swerve modules based on the given ChassisSpeeds
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void resetGyro() { // this function no longer works.....
    m_navX.reset();
    m_rotationOffset = (Utility.teamColorIsRed() ? 180 : 0);
    
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html#resetting-the-robot-pose
    // resetOdometry(getPose());

    Pose2d pose = getPose();

    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getAngleBlueSide()),
      getModulePositions(),
      new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(getAngleBlueSide()))
    );
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navX.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}