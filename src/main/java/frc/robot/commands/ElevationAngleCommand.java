// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevationAngleSub;

public class ElevationAngleCommand extends CommandBase {
  
  private ElevationAngleSub m_ElevationAngleSub;
  private PS4Controller m_Ps4Controller;
  private double leftYAxis;
  private double targetElevation;
  
  /** Creates a new ElveationAngleCommand. */
  public ElevationAngleCommand(ElevationAngleSub elevationAngleSub, PS4Controller ps4Controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElevationAngleSub = elevationAngleSub;
    m_Ps4Controller = ps4Controller;
    addRequirements(m_ElevationAngleSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetElevation = Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftYAxis = -m_Ps4Controller.getRawAxis(Constants.PS4_LEFT_STICK_Y_AXIS);
    if(leftYAxis > -0.2 && leftYAxis < 0.2){
      leftYAxis = 0;
    }
    targetElevation = targetElevation + leftYAxis * Constants.ELEVATION_SPEED;
    m_ElevationAngleSub.MoveElevationMotorToAngle(targetElevation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
