// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub;

public class ConstantShootCommand extends CommandBase {
  /** Creates a new ConstantShootCommand. */
  private ShooterSub m_ShooterSub;
  private double leftShooterIntegral;
  private double rightShooterIntegral;


  public ConstantShootCommand(ShooterSub shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSub = shooterSub;
    addRequirements(m_ShooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftShooterIntegral = 0;
    rightShooterIntegral = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Insert Test Velocity here
    leftShooterIntegral = m_ShooterSub.setShooterMotorVelocity(5000, leftShooterIntegral, Constants.LEFT_SHOOTER_MOTOR);
    rightShooterIntegral = m_ShooterSub.setShooterMotorVelocity(5000, rightShooterIntegral, Constants.RIGHT_SHOOTER_MOTOR);
    SmartDashboard.putString("Shooter Status:", "Shooting");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSub.setShooterMotorsPower(0);
    SmartDashboard.putString("Shooter Status:", "Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
