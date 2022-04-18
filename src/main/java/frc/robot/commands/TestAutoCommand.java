// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub;

public class TestAutoCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private final DrivetrainSub m_drivetrain;

  public TestAutoCommand(DrivetrainSub drivetrainSub) {
    m_drivetrain = drivetrainSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setPos(0, 0);
    m_drivetrain.resetYaw();
    System.out.println("IM NOT CRAZY");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drivetrainPositionIntegration();
    m_drivetrain.setDriveToWaypoint(25,25,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.hasReachedWaypoint();
  }
}
