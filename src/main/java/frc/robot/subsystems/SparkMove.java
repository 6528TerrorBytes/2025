// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMove extends SubsystemBase {
  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_controller;

  private double m_goal;
  private double m_tolerance;

  /** Creates a new SparkMove. */
  public SparkMove(int motorId, SparkMaxConfig config) {
    m_motor = new SparkMax(motorId, MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_controller = m_motor.getClosedLoopController();

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setGoal(double position) {
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

  public void disable() {
    m_motor.set(0);
  }
}
