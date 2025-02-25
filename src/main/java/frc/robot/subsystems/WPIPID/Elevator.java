// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WPIPID;

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

public class Elevator extends SubsystemBase {
  public final SparkMax m_motor; 
  public final RelativeEncoder m_encoder;

  private boolean m_disabled;
  private double m_goal;

  // Configuration
  private final TrapezoidProfile.Constraints m_trapezoidConfig = new TrapezoidProfile.Constraints(20, 16);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(16, 1.2, 0, m_trapezoidConfig);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.1, 0.6, 0); // make it so that when not moving, it holds its position
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
  // https://www.chiefdelphi.com/t/how-to-obtain-ks-for-feed-forward/446623

  public Elevator() {
    m_motor = new SparkMax(Constants.MotorIDs.elevatorID, MotorType.kBrushless);
    m_encoder = m_motor.getAlternateEncoder();
    m_disabled = true;

    m_motor.configure(Constants.MotorConfig.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (m_disabled) return;

    m_motor.setVoltage(
      m_controller.calculate(m_encoder.getPosition()) +
      m_feedforward.calculate(m_controller.getSetpoint().velocity)
    );
  }

  public void setGoal(double goal) {
    // Fixes desync between controller & position:
    m_controller.reset(m_encoder.getPosition()); // Fixes an issue where after a period of being disabled and the elevator moving, the position goes backwards for a bit when setting new goal.
    m_controller.setGoal(goal);
    m_goal = goal;
  }

  public boolean atGoal() {
    double pos = getPos();
    return (pos > m_goal - Constants.MotorConfig.elevatorTolerance) && (pos < m_goal + Constants.MotorConfig.elevatorTolerance);
  }
  
  public void enable() {
    m_disabled = false;
  }

  public void disable() {
    m_motor.set(0);
    m_disabled = true;
  }

  public double getPos() {
    return m_motor.get();
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }
}
