// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WPIElevator extends SubsystemBase {
  public final SparkMax m_motor; 
  public final RelativeEncoder m_encoder;

  private double m_goal;

  // Configuration
  private final TrapezoidProfile.Constraints m_trapezoidConfig = new TrapezoidProfile.Constraints(1, 1);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(1, 0, 0, m_trapezoidConfig);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0, 0, 0);
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
  // https://www.chiefdelphi.com/t/how-to-obtain-ks-for-feed-forward/446623


  /** Creates a new WPIElevator. */
  public WPIElevator() {
    m_motor = new SparkMax(Constants.MotorIDs.elevatorID, MotorType.kBrushless);
    m_encoder = m_motor.getAlternateEncoder();

    m_motor.configure(Constants.MotorConfig.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    m_motor.setVoltage(
      m_controller.calculate(m_encoder.getPosition()) +
      m_feedforward.calculate(m_encoder.getVelocity())
    );
  }

  public void setGoal(double goal) {
    m_goal = goal;
    m_controller.setGoal(goal);
  }

  public boolean atGoal() {
    double pos = m_encoder.getPosition();
    // Within plus/minus elevator tolerance of the goal position
    return (pos >= m_goal - Constants.MotorConfig.elevatorTolerance) && (pos <= m_goal + Constants.MotorConfig.elevatorTolerance);
  }

  public void disable() {
    m_motor.set(0);
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }
}
