// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainNavigationSub;

public class TestAutoCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private DriveTrainNavigationSub m_nav;


  public TestAutoCommand(DriveTrainNavigationSub navSub) {
    m_nav = navSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nav.setPos(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_nav.driveTrainPosIntegration();
    m_nav.setDriveToWaypoint(5,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
