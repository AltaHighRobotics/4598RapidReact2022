// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainNavigationSub;

public class TestAutoCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private DriveTrainNavigationSub m_nav;
  private double oldPosData [];
  private double newPosData [];
  private double targetData [];
  private double prevHeading;


  public TestAutoCommand(DriveTrainNavigationSub navSub) {
    m_nav = navSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldPosData[0] = 0.0;
    oldPosData[1] = 0.0;
    oldPosData[2] = 0.0;
    oldPosData[3] = 0.0;

    prevHeading = 0.0;
    targetData[0] = 0.0;
    targetData[1] = 0.0;
    targetData[2] = 0.0;
    targetData[3] = 0.0;
    targetData[4] = prevHeading;    

    newPosData = m_nav.driveTrainPosIntegration(oldPosData);
    m_nav.setDriveToWaypoint(targetData);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newPosData = m_nav.driveTrainPosIntegration(oldPosData);
    oldPosData = newPosData;

    targetData[0] = 5;
    targetData[1] = 5;
    targetData[2] = newPosData[2];
    targetData[3] = newPosData[3];

    prevHeading = m_nav.setDriveToWaypoint(targetData);

    targetData[4] = prevHeading;
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
