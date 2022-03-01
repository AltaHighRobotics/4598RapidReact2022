// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbingSub;

public class SecondaryClimbAutoCommand extends CommandBase {
  /** Creates a new SecondaryClimbAutoCommand. */
  private ClimbingSub m_climbingSub;
  private int stage;
  public int c;
  public int time;
  public boolean isItFinished;
  public SecondaryClimbAutoCommand(ClimbingSub climbingSub) {
    m_climbingSub = climbingSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int stage = 0;
    int c = 0;
    int time = 0;
    boolean isItFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage == 0)
    {
      boolean hasReachedSecondPosition = m_climbingSub.SetArmsWithClamp(Constants.MIN_ARM_POSITION);
      if (hasReachedSecondPosition)
      {
        stage = 1;
      }
    }
    else if (stage == 1)
    {
      boolean hasReachedThirdPosition = m_climbingSub.SetArmsWithClamp(Constants.ALMOST_MIN_POSITION);
      if (hasReachedThirdPosition)
      {
        stage = 2;
      }
    }
    else if (stage == 2)
    {
      m_climbingSub.SwingArms();
      time++;
      if (time == 2000)
      {
        time = 0;
        stage = 3;
      }
    }
    else if (stage == 3)
    {
      boolean hasReachedFourthPosition = m_climbingSub.SetArmsWithClamp(Constants.MAX_ARM_POSITION);
      if (hasReachedFourthPosition)
      {
        stage = 4;
      }
    }
    else if (stage == 4)
    {
      m_climbingSub.ReturnArms();
      time++;
      if (time == 2000)
      {
        time = 0;
        stage = 5;
      }
    }
    else if (stage == 5)
    {
      boolean hasReachedLastPosition = m_climbingSub.SetArmsWithClamp(Constants.MIN_ARM_POSITION);
      if (m_climbingSub.getRightCoderAngle() < Constants.HALF_ARM_POSITION)
      {
        m_climbingSub.ExtendArms();
      }
      if (hasReachedLastPosition)
      {
        stage = 1;
        c++;
        if (c == 2)
        {
          isItFinished = true;
        }
      }
      
    }
    if (c == 2)
    {
      isItFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbingSub.SetArmsWithClamp(Constants.MIN_ARM_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isItFinished;
  }
}
