// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AimingSub;

public class AimCommand extends CommandBase {
  
  private AimingSub m_aimingSub;
  private PS4Controller m_Ps4Controller;
  private double leftXAxis;
  
  /** Creates a new ElveationAngleCommand. */
  public AimCommand(AimingSub aimingSub, PS4Controller ps4Controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_aimingSub = aimingSub;
    m_Ps4Controller = ps4Controller;
    addRequirements(m_aimingSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_aimingSub.enableLimeLight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_aimingSub.moveAzimuthMotorToLimeLight();
    m_aimingSub.MoveElevationMotorToLimeLight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_aimingSub.disableLimeLight();
    m_aimingSub.stopAimingMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
