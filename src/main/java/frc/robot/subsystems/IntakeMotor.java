// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMotor extends SubsystemBase {
  private final SparkMax m_motor;

  /** Creates a new IntakeMotor. */
  public IntakeMotor() {
    m_motor = new SparkMax(Constants.MotorIDs.intakeMotorID, MotorType.kBrushless);
  }
  
  public void onForward() {
    m_motor.set(1);
  }

  public void onBackwards() {
    m_motor.set(-1);
  }

  public void disable() {
    m_motor.set(0);
  }
}
