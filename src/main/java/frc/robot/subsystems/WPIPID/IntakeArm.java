// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WPIPID;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
  private final SparkMax m_motor;
  public final AbsoluteEncoder m_encoder;

  private double m_goal;
  private boolean m_disabled;

  // This doesn't work when maxAcceleration is greater than zero for some reason
  private final TrapezoidProfile.Constraints m_trapezoidConfig = new TrapezoidProfile.Constraints(1000, 1000);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(0.05, 0.01, 0, m_trapezoidConfig);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0.95, 0);

  public IntakeArm() {
    m_motor = new SparkMax(Constants.MotorIDs.armID, MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_disabled = true;

    m_controller.setTolerance(Constants.MotorConfig.armTolerance);
    m_motor.configure(Constants.MotorConfig.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (m_disabled) return;
    
    m_motor.setVoltage(
      m_controller.calculate(m_encoder.getPosition()) +
      // This makes it so that zero is horizontal plus an angle to account for the off-center weight of the arm
      m_feedforward.calculate(Math.toRadians(m_encoder.getPosition() - Constants.MotorConfig.armAngleHorizontal - (Constants.MotorConfig.armAngleAtRest - Constants.MotorConfig.armAngleVerticalDown)), m_controller.getSetpoint().velocity) // ENCODER POSITION needs to be 0 at HORIZONTAL
    );
  }

  public void setGoal(double goal) {
    m_goal = goal;
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
