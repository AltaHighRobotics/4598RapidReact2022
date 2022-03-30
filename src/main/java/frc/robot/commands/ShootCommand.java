// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSub;

public class ShootCommand extends CommandBase {
  
  private final ShootingSub m_shootingSub;
  private final PS4Controller m_Ps4Controller;
  private double leftXAxis;
  private double leftYAxis;
  private double manualTargetAzimuth;
  private double manualTargetElevation;
  
  
  /** Creates a new ElveationAngleCommand. */
  public ShootCommand(ShootingSub shootingSub, PS4Controller ps4Controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shootingSub = shootingSub;
    m_Ps4Controller = ps4Controller;
    addRequirements(shootingSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shootingSub.enableLimeLight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftXAxis = m_Ps4Controller.getRawAxis(Constants.PS4_LEFT_STICK_X_AXIS);
    if(Math.abs(leftXAxis) < Constants.SHOOTER_CONTROL_OVERRIDE_THRESHOLD) {
      leftXAxis = 0;
    }
    leftYAxis = m_Ps4Controller.getRawAxis(Constants.PS4_LEFT_STICK_Y_AXIS);
    if(Math.abs(leftYAxis) < Constants.SHOOTER_CONTROL_OVERRIDE_THRESHOLD) {
      leftYAxis = 0;
    }

    if(m_shootingSub.getLimeLightElevation() == 0 && (leftXAxis != 0 || leftYAxis != 0) ) {
      manualTargetAzimuth = manualTargetAzimuth + leftXAxis * Constants.SHOOTER_CONTROL_SPEED;
      manualTargetElevation = manualTargetElevation + leftYAxis * Constants.SHOOTER_CONTROL_SPEED;
      m_shootingSub.moveAzimuthMotorToAngle(manualTargetAzimuth);
      m_shootingSub.moveElevationMotorToAngle(manualTargetElevation);
    } else {
      m_shootingSub.autoShoot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootingSub.disableLimeLight();
    m_shootingSub.stopAimingMotors();
    m_shootingSub.stopShooterMotors();
    m_shootingSub.feedOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
