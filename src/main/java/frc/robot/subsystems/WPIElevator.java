// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
  private final SparkMax m_motor; 
  private final AbsoluteEncoder m_encoder;

  // Configuration
  private final TrapezoidProfile.Constraints m_trapezoidConfig = new TrapezoidProfile.Constraints(0, 0);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(0, 0, 0, m_trapezoidConfig);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0, 0, 0);


  /** Creates a new WPIElevator. */
  public WPIElevator(int id) {
    m_motor = new SparkMax(id, MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();

    m_motor.configure(Constants.MotorConfig.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    m_motor.setVoltage(
      m_controller.calculate(m_encoder.getPosition()) +
      m_feedforward.calculate(m_encoder.getVelocity())
    );
  }
}
