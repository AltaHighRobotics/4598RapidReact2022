// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonInclude.Include;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class RapidReactAutoCommand extends CommandBase {
  private final DrivetrainSub m_drivetrain;
  private final IntakeSub m_intakeSub;
  private final ShooterSub m_shooterSub;
  private boolean includeBall1;
  private boolean includeBall2;
  private boolean includeBall3;
  private boolean includeBall4;
  private int stage;
  private double [] waypoint;

  public RapidReactAutoCommand(DrivetrainSub drivetrainSub, ShooterSub shooterSub, IntakeSub intakeSub, Boolean ball1, Boolean ball2, Boolean ball3, Boolean ball4) {
    m_drivetrain = drivetrainSub;
    m_shooterSub = shooterSub;
    m_intakeSub = intakeSub;
    includeBall1 = ball1;
    includeBall2 = ball2;
    includeBall3 = ball3;
    includeBall4 = ball4;

    waypoint = new double[2];
    stage = 1;
    
    addRequirements(shooterSub, intakeSub, drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    waypoint[0] = 1;
    waypoint[1] = 1;
    m_drivetrain.setPos(0, 0);
    m_drivetrain.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSub.IntakeExtend();
    m_intakeSub.IntakeOn();
    m_drivetrain.drivetrainPositionIntegration();
    m_drivetrain.setDriveToWaypoint(waypoint[0],waypoint[1]);

    switch (stage)
    {
      case 1:
        if (!includeBall1)
        {
          stage = 2;
        }
        else
        {
          waypoint = Constants.WAYPOINT_BALL_1;
          if (m_drivetrain.hasReachedWaypoint())
          {
            if(m_shooterSub.autoShootBall())
            {
              stage = 2;
              System.out.println("1");
            }
          }
        }
        break;

        case 2:
        if (!includeBall2)
        {
          stage = 3;
        }
        else
        {
          waypoint = Constants.WAYPOINT_BALL_2;
          if (m_drivetrain.hasReachedWaypoint())
          {
            if(m_shooterSub.autoShootBall())
            {
              stage = 3;
            }
          }
        }
        break;

        case 3:
        if (!includeBall3)
        {
          stage = 4;
        }
        else
        {
          waypoint = Constants.WAYPOINT_BALL_3;
          if (m_drivetrain.hasReachedWaypoint())
          {
            if(m_shooterSub.autoShootBall())
            {
              stage = 4;
            }
          }
        }
        break;

        case 4:
        if (!includeBall4)
        {
          stage = 5;
        }
        else
        {
          waypoint = Constants.WAYPOINT_BALL_4;
          if (m_drivetrain.hasReachedWaypoint())
          {
            if(m_shooterSub.autoShootBall())
            {
              stage = 5;
            }
          }
        }
        break;

      case 5:
        SmartDashboard.putString("Auto Process", "IDLE");
        break;
      
      default:
        SmartDashboard.putString("Auto Process", "ERROR BAD LOGIC");
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
