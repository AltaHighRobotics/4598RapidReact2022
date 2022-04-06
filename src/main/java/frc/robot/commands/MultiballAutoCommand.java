// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ShootingSub;


public class MultiballAutoCommand extends CommandBase 
{
  /** Creates a new DriveCommand. */
  private final DrivetrainSub m_drivetrain;
  private final ShootingSub m_shootSub;
  private int c;
  public MultiballAutoCommand(DrivetrainSub drivetrainSub, ShootingSub shootSub) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    c = 0;
    m_drivetrain = drivetrainSub;
    m_shootSub = shootSub;
    addRequirements(m_drivetrain, m_shootSub);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shootSub.enableLimeLight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (c < 80)
    {
      m_drivetrain.setMotorAuto();
    }
    else
    {
      m_drivetrain.stopMotors();
    }
    c++;
    if (c > 80)
    {
      m_shootSub.autoShoot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
    m_shootSub.disableLimeLight();
    m_shootSub.stopAimingMotors();
    m_shootSub.stopShooterMotors();
    m_shootSub.feedOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}