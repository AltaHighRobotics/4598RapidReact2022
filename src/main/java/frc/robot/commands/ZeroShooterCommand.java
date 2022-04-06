// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSub;

public class ZeroShooterCommand extends CommandBase {
  
  private final ShootingSub m_shootingSub;
  
  /** Creates a new ElveationAngleCommand. */
  public ZeroShooterCommand(ShootingSub shootingSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shootingSub = shootingSub;
    addRequirements(shootingSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shootingSub.moveAzimuthMotorToAngle(0);
    m_shootingSub.moveElevationMotorToAngle(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootingSub.stopAimingMotors();
    m_shootingSub.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
