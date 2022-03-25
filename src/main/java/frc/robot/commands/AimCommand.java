// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double targetShooterVelocity = 5000;
  private double targetShooterElevation = 45;
  
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
    if(Math.abs(leftYAxis) > 0.2) {
      targetShooterElevation = targetShooterElevation + leftYAxis*0.25;
    }
    if(Math.abs(leftXAxis) > 0.2) {
      targetShooterVelocity = targetShooterVelocity + leftXAxis*5;
    }
    double A2 = m_limeLightSub.getLimeLightElevation();
    double angleToGoalDegrees = Constants.LIMELIGHT_ELEVATION_ANGLE+A2;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    double D = (Constants.H2-Constants.H1)/(Math.tan(angleToGoalRadians));
    SmartDashboard.putNumber("Distance", D);
    int i; 
    for(i=0;i<Constants.SHOOTER_DATA.length && D > Constants.SHOOTER_DATA[i][0]; i++){
      //Searching for the right i
    }    
    if(i>=Constants.SHOOTER_DATA.length){
      //don't shoot
      return;
    }
    if(i==0){
      //shoot at min power, too close
      return;
    }

    double bigD = Constants.SHOOTER_DATA[i][0];
    double smallD = Constants.SHOOTER_DATA[i-1][0];
    
    double bigPower = Constants.SHOOTER_DATA[i][1];
    double smallerPower = Constants.SHOOTER_DATA[i-1][1];

    double bigAnlge = Constants.SHOOTER_DATA[i][2];
    double smallerAngle = Constants.SHOOTER_DATA[i-1][2];

    double distanceSteps;
    double deltaPower;
    double deltaAngle;

    double powerModifier;
    double angleModifier;
    double finalPower;
    double finalAngle;


    distanceSteps = bigD-smallD;
    deltaPower = bigPower-smallerPower;
    deltaAngle = bigAnlge-smallerAngle;

    powerModifier=deltaPower*((D-smallD)/distanceSteps);
    angleModifier=deltaAngle*((D-smallD)/distanceSteps);

    finalPower = smallerPower + powerModifier;
    finalAngle = smallerAngle + angleModifier;

    double limeLightYaw = m_limeLightSub.getLimeLightYaw();
    double limeLightElevation = m_limeLightSub.getLimeLightElevation();
    m_aimingSub.moveAzimuthMotorToLimeLight(limeLightYaw+Constants.LIMELIGHT_YAW_OFFSET);
    m_aimingSub.moveElevationMotorToAngle(finalAngle);//limeLightElevation);
    m_aimingSub.setShooterMotorsVelocity(finalPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLightSub.disableLimeLight();
    m_aimingSub.stopAimingMotors();
    m_aimingSub.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
