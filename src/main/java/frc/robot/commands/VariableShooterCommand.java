// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.Constants;

public class VariableShooterCommand extends CommandBase {
  /** Creates a new VariableShooterCommand. */

  private LimeLightSub m_limeLightSub;
  private double angleToGoalDegrees;
  private double angleToGoalRadians;
  private double A2;
  private double D;
  private boolean isFinished;
  private ShooterSub m_ShooterSub;

  public VariableShooterCommand(LimeLightSub limeLightSub, ShooterSub shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limeLightSub = limeLightSub;
    m_ShooterSub = shooterSub;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    A2 = m_limeLightSub.limeLight.getdegVerticalToTarget();
    angleToGoalDegrees = Constants.A1+A2;
    angleToGoalRadians = angleToGoalDegrees*Constants.RADIAN_CONVERSION;
    D = (Constants.H2-Constants.H1)/(Math.tan(angleToGoalRadians));

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

    double distanceSteps;
    double deltaPower;

    double powerModifier;
    double finalPower;


    distanceSteps = bigD-smallD;
    deltaPower = bigPower-smallerPower;

    powerModifier=deltaPower*((D-smallD)/distanceSteps);

    finalPower = smallerPower + powerModifier;    

    m_ShooterSub.setShooterMotors(finalPower);

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
