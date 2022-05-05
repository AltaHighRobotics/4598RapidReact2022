
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
import frc.robot.subsystems.ShootingSub;
import frc.robot.utilities.vector;

public class RapidReactAutoCommand extends CommandBase {
  private final DrivetrainSub m_drivetrain;
  private final IntakeSub m_intakeSub;
  private final ShootingSub m_shooterSub;
  private boolean includeBall1;
  private boolean includeBall2;
  private boolean includeBall3;
  private boolean includeBall4;
  private boolean hasgoneshoot;
  private boolean runningoutofnames;
  private boolean isDriving;
  private int stage;
  private int c;
  private int cButlikeAgain;
  private int cargoCount;
  private Integer origPos;
  private vector currentPos;

  public RapidReactAutoCommand(DrivetrainSub drivetrainSub, ShootingSub shootingSub, IntakeSub intakeSub, Integer InitialPosition, Boolean ball1, Boolean ball2, Boolean ball3, Boolean ball4) {
    m_drivetrain = drivetrainSub;
    m_shooterSub = shootingSub;
    m_intakeSub = intakeSub;
    isDriving = false;
    hasgoneshoot = false;
    runningoutofnames = false;
    includeBall1 = ball1;
    includeBall2 = ball2;
    includeBall3 = ball3;
    includeBall4 = ball4;
    origPos = InitialPosition;
    cargoCount = 1;
    c = 0;
    cButlikeAgain = 0;
    currentPos = new vector(0, 0);
    stage = 0;
    
    addRequirements(shootingSub, intakeSub, drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    switch (origPos)
    {
      case 1:
        m_drivetrain.setPos(Constants.START_ONE_OFFSET);
        break;

      case 2:
        m_drivetrain.setPos(Constants.START_TWO_OFFSET);
      
      case 3:
        m_drivetrain.setPos(Constants.START_THREE_OFFSET);
    }
    // m_drivetrain.setPos(0, 0);
    currentPos = m_drivetrain.getPos();
    m_drivetrain.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSub.IntakeOn();
    m_intakeSub.IntakeExtend();
    m_drivetrain.drivetrainPositionIntegration();
    currentPos = m_drivetrain.getPos();
    
    if(cargoCount > 1)
    {
        robotShootCargo();
    }
    else
    {
        robotDriveToPoint();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
    m_intakeSub.IntakeRetract();
    m_intakeSub.IntakeOff();
    m_shooterSub.stopAimingMotors();
    m_shooterSub.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void robotShootCargo()
  {
    m_drivetrain.aimRobot(Constants.GOAL_VECTOR);
    m_shooterSub.autoShoot();
    c++;
    if (c > 50)
    {
        cargoCount = 0;
        c = 0;
    }
    
  }

  public void robotDriveToPoint()
  {
    switch (stage)
    {
        case 0:
        m_drivetrain.driveForwardTo(Constants.WAYPOINT_BALL_1);

        if(m_drivetrain.isAtPoint(Constants.WAYPOINT_BALL_1))
        {
            cargoCount++;
            stage++;
        }
    }
  }
}
