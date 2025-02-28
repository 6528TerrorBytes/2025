// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WPIPID.TailArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TailArmMove extends Command {
  private final TailArm m_tailArm;
  private final double m_setpoint;
  
  /** Creates a new TailArmMove. */
  public TailArmMove(TailArm tailArm, double setpoint) {
    m_tailArm = tailArm;
    m_setpoint = setpoint;
    
    addRequirements(tailArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tailArm.enable();
    m_tailArm.setGoal(m_setpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tailArm.atGoal();
  }
}
