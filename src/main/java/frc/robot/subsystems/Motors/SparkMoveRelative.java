// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Uses relative encoder rather than absolute encoders
public class SparkMoveRelative extends SubsystemBase {
  public final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_controller;

  private double m_goal;
  private double m_tolerance;

  /** Creates a new SparkMove. */
  public SparkMoveRelative(int motorId, SparkMaxConfig config) {
    m_motor = new SparkMax(motorId, MotorType.kBrushless);
    m_encoder = m_motor.getAlternateEncoder();
    m_controller = m_motor.getClosedLoopController();

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Sets reference of motor higher/lower based on the constant offset (gravity)
    // Example: https://github.com/STMARobotics/frc-7028-2023/blob/5578980e795de4744cb163f76aa883e8ba2c35d5/src/main/java/frc/robot/subsystems/WristSubsystem.java
    // May not be necessary because the new position control is now based on position rather rather than velocity.
    // if (m_motor.get() != 0) { }
  }

  public void setGoal(double position) {
    System.out.println(position);
    System.out.println(m_encoder.getPosition());
    m_goal = position;
    m_controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public void setTolerance(double tolerance) {
    m_tolerance = tolerance;
  }

  public boolean atGoal() {
    double pos = m_encoder.getPosition();
    return (pos >= m_goal - m_tolerance) && (pos <= m_goal + m_tolerance);
  }

  public double getPos() {
    return m_encoder.getPosition();
  }

  public void disable() {
    m_motor.set(0);
  }
}
