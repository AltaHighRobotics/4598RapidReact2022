// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbingSub;

public class WinchRevCommand extends CommandBase {
  /** Creates a new WinchRevCommand. */
  private final ClimbingSub m_climbingSub;

  public WinchRevCommand(ClimbingSub climbingSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climbingSub = climbingSub;
    addRequirements(climbingSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbingSub.revWinch();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbingSub.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
