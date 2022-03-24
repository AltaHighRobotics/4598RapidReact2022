// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AzimuthSub;
import frc.robot.subsystems.LimeLightSub;
import limelightvision.limelight.frc.ControlMode.LedMode;

public class AzimuthCommand extends CommandBase {
  
  private AzimuthSub m_azimuthSub;
  private LimeLightSub m_limeLightSub;
  private PS4Controller m_Ps4Controller;
  private final int targetPipeline = 0;
  private final int lookPipeline = 1;
  private double leftXAxis;
  private double targetAzimuth;
  
  /** Creates a new ElveationAngleCommand. */
  public AzimuthCommand(AzimuthSub azimuthSub, PS4Controller ps4Controller, LimeLightSub limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_azimuthSub = azimuthSub;
    m_Ps4Controller = ps4Controller;
    m_limeLightSub = limelight;
    addRequirements(m_azimuthSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAzimuth = 0;
    m_limeLightSub.getLimeLight().setPipeline(targetPipeline);
    m_limeLightSub.getLimeLight().setLEDMode(LedMode.kforceOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // leftXAxis = m_Ps4Controller.getRawAxis(Constants.PS4_LEFT_STICK_X_AXIS);
    // if(leftXAxis > -0.2 && leftXAxis < 0.2){
    //   leftXAxis = 0;
    // }
    // targetAzimuth = targetAzimuth + leftXAxis * 0.5;
    targetAzimuth = targetAzimuth + m_limeLightSub.limeLight.getdegRotationToTarget()/20;
    // SmartDashboard.putNumber("Limelight Angle", m_limeLightSub.limeLight.getdegRotationToTarget());
    SmartDashboard.putNumber("Target Angle", targetAzimuth);
    m_azimuthSub.moveAzimuthMotorToAngle(targetAzimuth);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLightSub.getLimeLight().setLEDMode(LedMode.kforceOff);
    m_limeLightSub.getLimeLight().setPipeline(lookPipeline);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
