// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.ColorSub;
import frc.robot.subsystems.FeedSub;
import frc.robot.subsystems.LimeLightSub;

public class AimCommand extends CommandBase {
  
  private final AimingSub m_aimingSub;
  private final PS4Controller m_Ps4Controller;
  private final LimeLightSub m_limeLightSub;
  private final FeedSub m_feedSub;
  private final ColorSub m_colorSub;
  private double leftXAxis;
  private double leftYAxis;
  private double targetShooterVelocity = 5000;
  private double targetShooterElevation = 45;
  private boolean shouldScore = false;
  private int count = 0;
  
  
  /** Creates a new ElveationAngleCommand. */
  public AimCommand(AimingSub aimingSub, PS4Controller ps4Controller, LimeLightSub limeLightSub, FeedSub feedSub, ColorSub colorSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_aimingSub = aimingSub;
    m_Ps4Controller = ps4Controller;
    m_limeLightSub = limeLightSub;
    m_feedSub = feedSub;
    m_colorSub = colorSub;
    addRequirements(m_aimingSub, m_limeLightSub, m_feedSub, m_colorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limeLightSub.enableLimeLight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftXAxis = m_Ps4Controller.getRawAxis(Constants.PS4_LEFT_STICK_X_AXIS);
    leftYAxis = m_Ps4Controller.getRawAxis(Constants.PS4_LEFT_STICK_Y_AXIS);
    if(Math.abs(leftYAxis) > 0.2) {
      targetShooterElevation = targetShooterElevation + leftYAxis*0.25;
    }
    if(Math.abs(leftXAxis) > 0.2) {
      targetShooterVelocity = targetShooterVelocity + leftXAxis*5;
    }
    m_colorSub.getColor();
    if(m_colorSub.ballDetected()) {
      if(m_colorSub.matchColorToAlliance(m_colorSub.getAlliance())) {
        shouldScore = true;
      } else {
        shouldScore = false;
      }
      count = 0;
    } else {
      count = count + 1;
      if(count > 100) {
        shouldScore = true;
      }
    }
    SmartDashboard.putBoolean("Should Shoot?", shouldScore);
    double limeLightYaw = m_limeLightSub.getLimeLightYaw();
    double limeLightElevation = m_limeLightSub.getLimeLightElevation();
    if(limeLightElevation != 0) {
      m_aimingSub.lerpShooter(limeLightElevation);
    } else {
      m_aimingSub.stopShooterMotors();
      m_aimingSub.moveElevationMotorToAngle(0);
    }
    if(shouldScore) {
      m_aimingSub.moveAzimuthMotorToLimeLight(limeLightYaw+Constants.LIMELIGHT_YAW_OFFSET);
      
    } else {
      m_aimingSub.moveAzimuthMotorToAngle(Constants.AZIMUTH_BARF_ANGLE);
      //m_aimingSub.setShooterMotorsVelocity(Constants.SHOOTER_BARF_SPEED);
    }
    if(m_aimingSub.getIsAimReady()) {
      m_feedSub.feedOn();
    } else {
      m_feedSub.feedOff();
    }
    // m_aimingSub.moveAzimuthMotorToLimeLight(limeLightYaw+Constants.LIMELIGHT_YAW_OFFSET);
    // m_aimingSub.moveElevationMotorToAngle(Constants.ELEVATION_BARF_ANGLE);
    // m_aimingSub.setShooterMotorsVelocity(10000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLightSub.disableLimeLight();
    m_aimingSub.stopAimingMotors();
    m_aimingSub.stopShooterMotors();
    m_feedSub.feedOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
