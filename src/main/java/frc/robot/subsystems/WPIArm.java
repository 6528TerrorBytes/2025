// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WPIArm extends SubsystemBase {
  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;

  private double m_goal;

  private final TrapezoidProfile.Constraints m_trapezoidConfig = new TrapezoidProfile.Constraints(1, 1);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(1, 0, 0, m_trapezoidConfig);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0);

  /** Creates a new WPIArm. */
  public WPIArm() {
    m_motor = new SparkMax(5, MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();

    m_motor.configure(Constants.MotorConfig.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_controller.enableContinuousInput(0, 360); // Position wrapping (can cross over 360 to the other side)
  }

  @Override
  public void periodic() {
    m_motor.setVoltage(
      m_controller.calculate(m_encoder.getPosition()) +
      m_feedforward.calculate(Math.toRadians(m_encoder.getPosition()), m_encoder.getVelocity()) // ENCODER POSITION needs to be 0 at HORIZONTAL
    );
  }

  public void setGoal(double goal) {
    m_goal = goal;
    m_controller.setGoal(goal);
  }

  public boolean atGoal() {
    double pos = m_encoder.getPosition();
    return (pos >= m_goal - Constants.MotorConfig.armTolerance) && (pos <= m_goal + Constants.MotorConfig.armTolerance);
  }
  
  public void disable() {
    m_motor.set(0);
  }
}
