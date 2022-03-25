// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeedSub;

public class FeedCommand extends CommandBase {
  /** Creates a new ElevatorOnCommand. */
  private FeedSub m_feedSub;
  private int t;
  public FeedCommand(FeedSub feedSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feedSub = feedSub  ;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (t > 75)
    {
      m_feedSub.feedOn();
    }
    t++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feedSub.feedOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
