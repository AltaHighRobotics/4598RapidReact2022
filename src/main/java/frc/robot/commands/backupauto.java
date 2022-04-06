// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;


public class backupauto extends CommandBase 
{
  /** Creates a new DriveCommand. */
  private final DrivetrainSub m_drivetrain;
  private int c;
  public backupauto(DrivetrainSub drivetrainSub) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    c = 0;
    m_drivetrain = drivetrainSub;
    addRequirements(m_drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (c < 70)
    {
      m_drivetrain.setMotorAuto();
    }
    else
    {
      m_drivetrain.stopMotors();
    }
    c++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
