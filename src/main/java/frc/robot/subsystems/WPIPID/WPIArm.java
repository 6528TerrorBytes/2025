// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WPIPID;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WPIArm extends SubsystemBase {
  private final SparkMax m_motor;
  public final AbsoluteEncoder m_encoder;

  private boolean m_disabled;

  private final TrapezoidProfile.Constraints m_trapezoidConfig;
  private final ProfiledPIDController m_controller;
  private final ArmFeedforward m_feedforward;

  public WPIArm(int motorID, SparkMaxConfig config, double tolerance, double maxVel, double maxAcc, double p, double i, double d, double s, double g, double v) {
    m_trapezoidConfig = new TrapezoidProfile.Constraints(maxVel, maxAcc);
    m_controller = new ProfiledPIDController(p, i, d, m_trapezoidConfig);
    m_feedforward = new ArmFeedforward(s, g, v);

    m_motor = new SparkMax(motorID, MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_disabled = true;

    m_controller.setTolerance(tolerance);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (m_disabled) return;
    
    m_motor.setVoltage(
      m_controller.calculate(m_encoder.getPosition()) +
      // This makes it so that zero is horizontal plus an angle to account for the off-center weight of the arm
      m_feedforward.calculate(Math.toRadians(getAngleFromHorizontal(m_encoder.getPosition())), m_controller.getSetpoint().velocity) // ENCODER POSITION needs to be 0 at HORIZONTAL
    );
  }

  /**
   * OVERRIDE THIS in child classes.
   * Must return an angle in degrees, where angle of 0 is when the arm is perfectly horizontal.
   * This is for the arm feedforward.
   **/ 
  public double getAngleFromHorizontal(double encoderPos) {
    System.out.println("ERROR: override getAngleFromHorizontal in child class");
    return 0;
  }

  public void setGoal(double goal) {
    m_controller.reset(m_encoder.getPosition()); // Fixes desync, very important
    m_controller.setGoal(goal);
  }

  public boolean atGoal() {
    return m_controller.atSetpoint();
  }

  public void enable() {
    m_disabled = false;
  }
  
  public void disable() {
    m_motor.set(0);
    m_disabled = true;
  }
}
