// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralDetector extends SubsystemBase {
  private final DigitalInput m_coralBeamBreak;

  public CoralDetector() {
    m_coralBeamBreak = new DigitalInput(Constants.DigitalInputs.coralDetector);
  }

  /**
   * Returns true when the beam is broken and there's a piece
   */ 
  public boolean detected() {
    return !m_coralBeamBreak.get();
  }
}
