// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonInclude.Include;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;

public class RapidReactAutoCommand extends CommandBase {
  private final DrivetrainSub m_drivetrain;
  private boolean includeBall1;
  private boolean includeBall2;
  private boolean includeBall3;
  private boolean includeBall4;
  private int stage;
  private double [] waypoint;

  public RapidReactAutoCommand(DrivetrainSub drivetrainSub, Boolean ball1, Boolean ball2, Boolean ball3, Boolean ball4) {
    m_drivetrain = drivetrainSub;
    includeBall1 = ball1;
    includeBall2 = ball2;
    includeBall3 = ball3;
    includeBall4 = ball4;

    waypoint = new double[2];
    stage = 1;
    
    addRequirements(drivetrainSub);
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
    m_drivetrain.drivetrainPositionIntegration();
    m_drivetrain.setDriveToWaypoint(waypoint[0],waypoint[1]);
    //System.out.println(!includeBall1);

    switch (stage)
    {
      case 1:
        if (!includeBall1)
        {
          System.out.println("doesnt work");
          stage = 2;
        }
        else
        {
          waypoint = Constants.WAYPOINT_BALL_1;
          if (m_drivetrain.hasReachedWaypoint())
          {
            stage = 2;
          }
        }
        break;

      case 2:
        if(!includeBall2)
        {
          stage = 3;
        }
        //ball 2 code
        break;

      case 3:
        if(!includeBall3)
        {
          stage = 4;
        }
        //ball 3 code
        break;

      case 4:
        if(!includeBall4)
        {
          stage = 5;
        }
        //ball 4 code
        break;

      case 5:
        SmartDashboard.putString("Auto Process", "IDLE");
        break;
      
      default:
        SmartDashboard.putString("Auto Process", "ERROR BAD LOGIC");
        break;
    }
    //System.out.println(waypoint[0]+ " " + waypoint[1]);
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
