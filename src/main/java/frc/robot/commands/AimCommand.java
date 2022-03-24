// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.LimeLightSub;

public class AimCommand extends CommandBase {
  
  private final AimingSub m_aimingSub;
  private final PS4Controller m_Ps4Controller;
  private final LimeLightSub m_limeLightSub;
  private double leftXAxis;
  private double leftYAxis;
  
  /** Creates a new ElveationAngleCommand. */
  public AimCommand(AimingSub aimingSub, PS4Controller ps4Controller, LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_aimingSub = aimingSub;
    m_Ps4Controller = ps4Controller;
    m_limeLightSub = limeLightSub;
    addRequirements(m_aimingSub, m_limeLightSub);
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
    double limeLightYaw = m_limeLightSub.getLimeLightYaw();
    double limeLightElevation = m_limeLightSub.getLimeLightElevation();
    m_aimingSub.moveAzimuthMotorToLimeLight(limeLightYaw);
    m_aimingSub.moveElevationMotorToLimeLight(limeLightElevation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLightSub.disableLimeLight();
    m_aimingSub.stopAimingMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
