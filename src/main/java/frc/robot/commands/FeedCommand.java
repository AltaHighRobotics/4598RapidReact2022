// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.StorageSub;

public class FeedCommand extends CommandBase {
  /** Creates a new ElevatorOnCommand. */
  private StorageSub m_StorageSub;
  public FeedCommand(StorageSub storageSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_StorageSub = storageSub;
    addRequirements(storageSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_StorageSub.feedOn();
    SmartDashboard.putString("Feeder Status:", "Feeding");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_StorageSub.feedOff();
    SmartDashboard.putString("Feeder Status:", "Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
