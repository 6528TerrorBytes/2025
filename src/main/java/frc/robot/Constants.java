package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Constants {
  // height to the arm joint: 33.375 inches
  // 2.15: 46 + 1/16
  // 6.2: 70.5

  // 5.95 inches per rotation

  public static final class Setpoints {
    // Elevator setpoints
    public static final double elevatorZero = 0.1;
    public static final double elevatorScoreLow = 0.5;
    public static final double elevatorScoreMiddle = 1.325;
    public static final double elevatorIntake = 2.338;
    public static final double elevatorGrabLow = 1.9;
    public static final double elevatorGrabHigh = 4.5;
    public static final double elevatorHigh = 6.2;

    public static final double elevatorAlgaeFlapMovePos = 1;

    // Arm config angles    
    public static final double armAngleAtRest = 42; // angle it sits at with no power
    public static final double armAngleHorizontal = 125; // angle when it's perfectly horizontal
    
    // Arm setpoint angles
    public static final double armAngleVerticalDown = armAngleHorizontal - 100;
    public static final double armAngleL2 = armAngleHorizontal + 30;
    public static final double armAngleL1Stage1 = armAngleHorizontal - 30;
    public static final double armAngleL1Stage2 = armAngleHorizontal;
    public static final double armAngleIntake = armAngleVerticalDown + 42;
    public static final double armAngleMiddleHigh = armAngleHorizontal + 55;
    public static final double armAngleHigh = armAngleHorizontal + 80;
    public static final double armAngleHoldAlgae = armAngleHorizontal - 25;

    public static final double armElevatorMoveAngle = 52; // Arm needs to be greater than this angle before the elevator is allowed to move

    // Algae fork setpoint angles
    public static final double algaeForkHorizontal = 119;
    public static final double algaeForkZero = algaeForkHorizontal - 95;

    // Tail arm setpoint angles
    public static final double tailArmHorizontal = 70;
    public static final double tailArmScore = tailArmHorizontal + 40;
    public static final double tailArmStartingAngle = tailArmHorizontal + 105;
    public static final double tailArmDown = tailArmHorizontal - 35;

    // Intake motor stop delays
    public static final double m_intakeMotorStopDelayPickup = 0;
    public static final double m_intakeMotorStopDelayDunk = 0.2;
  }

  public static final class AprilTags {
    public static final double[] coralBlueTags = { 17, 18, 19, 20, 21, 22 };
    public static final double[] coralRedTags =  { 6, 7, 8, 9, 10, 11 };

    public static final double[] stationBlueTags = { 12, 13 };
    public static final double[] stationRedTags =  { 1, 2 };

    // X is horizontal distance, Y is distance out from coral aprilTag -- half of bot width (bumpers included)
    public static final Translation2d coralOffsetRight = new Translation2d(0.147, 0.56); // 0.148
    public static final Translation2d coralOffsetLeft = new Translation2d(-0.147, 0.56);
    public static final Translation2d coralOffsetRightLow = new Translation2d(0.147, 0.62);
    public static final Translation2d coralOffsetLeftLow = new Translation2d(-0.147, 0.62);
    
    public static final Translation2d coralOffsetCentered = new Translation2d(-0.06, 0.46);

    public static final double coralXTagOffset = -0.04;

    public static final Translation2d coralCollectOffset = new Translation2d(0, 0.515); // 0.035

    public static final PathConstraints aprilTagDriveConstraints = new PathConstraints(0.4, 0.4, 0.4, 0.4);

    public static Map<Double, Double> aprilTagFaceAngles = new HashMap<Double, Double>();

    static {
      // blue side apriltags 
      aprilTagFaceAngles.put(Double.valueOf(17), Double.valueOf(60));
      aprilTagFaceAngles.put(Double.valueOf(18), Double.valueOf(0));
      aprilTagFaceAngles.put(Double.valueOf(19), Double.valueOf(-60));
      aprilTagFaceAngles.put(Double.valueOf(20), Double.valueOf(-120));
      aprilTagFaceAngles.put(Double.valueOf(21), Double.valueOf(180));
      aprilTagFaceAngles.put(Double.valueOf(22), Double.valueOf(120));

      // red side apriltags
      aprilTagFaceAngles.put(Double.valueOf(11), Double.valueOf(60));
      aprilTagFaceAngles.put(Double.valueOf(10), Double.valueOf(0));
      aprilTagFaceAngles.put(Double.valueOf(9), Double.valueOf(-60));
      aprilTagFaceAngles.put(Double.valueOf(8), Double.valueOf(-120));
      aprilTagFaceAngles.put(Double.valueOf(7), Double.valueOf(180));
      aprilTagFaceAngles.put(Double.valueOf(6), Double.valueOf(120));

      // coral intake apriltags
      aprilTagFaceAngles.put(Double.valueOf(13), Double.valueOf(125));
      aprilTagFaceAngles.put(Double.valueOf(12), Double.valueOf(-125));
      aprilTagFaceAngles.put(Double.valueOf(2), Double.valueOf(55));
      aprilTagFaceAngles.put(Double.valueOf(1), Double.valueOf(-55));
    }
  }

  public static final class MotorIDs {
    public static final int climbOuterID = 1;
    public static final int climbInnerID = 3;
    public static final int elevatorID = 4;
    public static final int armID = 5;
    public static final int intakeMotorID = 6;
    public static final int algaeForkID = 7;
    public static final int tailArmID = 8;
    public static final int tailIntakeID = 9;
  }

  public static final class DigitalInputs {
    public static final int coralDetector = 0;
  }

  public static final class BlinkinConfig {
    public static final int id = 0;

    // Color numbers PDF at the bottom of: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
    public static final double blue = 0.87;
    public static final double red = 0.61;
    public static final double green = 0.77;

    public static final double strobeRed = -0.11;
    public static final double strobeBlue = -0.09;
  }

  public static final class MotorConfig {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final double elevatorTolerance = 0.1;

    static {
      elevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(35)
        .inverted(false);
      elevatorConfig.alternateEncoder
        .positionConversionFactor(1) // 5.95 conversion inches
        .velocityConversionFactor(1) // 5.95 conversion inches
        .inverted(true);
    }
    
    // intake arm
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final double armTolerance = 8; // Might need to be greater to account for error

    static {
      armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(35)
        .inverted(false);
      armConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(false);
    }

    public static final SparkMaxConfig algaeForkConfig = new SparkMaxConfig();
    public static final double algaeForkTolerance = 5;

    static {
      algaeForkConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false);
      algaeForkConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(false);
    }

    public static final SparkMaxConfig tailArmConfig = new SparkMaxConfig();
    public static final double tailArmTolerance = 5;

    static {
      tailArmConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(true);
      tailArmConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(false);
    }

    public static final SparkMaxConfig outerClimbConfig = new SparkMaxConfig();

    static {
      outerClimbConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(35)
        .inverted(false);
      outerClimbConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(true);
      outerClimbConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.025, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);
        // .positionWrappingMinInput(0)
        // .positionWrappingMaxInput(360);
      outerClimbConfig.closedLoop.maxMotion
        .maxVelocity(360 * 300) // units per minute i think? of the actual motor, not the encoder?
        .maxAcceleration(360 * 500)
        .allowedClosedLoopError(10);
    }

    public static final SparkMaxConfig innerClimbConfig = new SparkMaxConfig();

    static {
      innerClimbConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(35)
        .inverted(true); // this is important or else it'll go the wrong direction away from any setpoints
      innerClimbConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(false);
      innerClimbConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.025, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);
        // .positionWrappingMinInput(0)
        // .positionWrappingMaxInput(360);
      innerClimbConfig.closedLoop.maxMotion
        .maxVelocity(360 * 300) // units per minute i think? of the actual motor, not the encoder?
        .maxAcceleration(360 * 500)
        .allowedClosedLoopError(10);
    }
  }

  // DRIVECONSTANTS & MODULECONSTANTS are copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 100; // radians per second
    public static final double kMagnitudeSlewRate = 100; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 100; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = 0.5715; // meters
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5715;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 10;
    public static final int kRearRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontRightTurningCanId = 20;
    public static final int kRearRightTurningCanId = 23;

    public static final boolean kGyroReversed = false;

    // Used for AprilTag positioning
    // Full width/length including bumpers in meters
    public static final double kFullWidth = 0.762;
    public static final double kFullLength = 0.762;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;
    // Module free speed is 6.77
    // Gear reduction 4.0

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.07601585;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 35; // amps
    public static final int kTurningMotorCurrentLimit = 15; // amps
  }

  public static final class SwerveConfig {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    // Setup all the constants, using those defined above in ModuleConstants. Used in MAXSwerveModule.java
    static {
      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
      drivingConfig.encoder
        .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor) // meters
        .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor); // meters per second
      drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
        .velocityFF(ModuleConstants.kDrivingFF)
        .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
      turningConfig.absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        .inverted(ModuleConstants.kTurningEncoderInverted)
        .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor) // radians
        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor); // radians per second
      turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
        .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    }
  }  
  
  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
