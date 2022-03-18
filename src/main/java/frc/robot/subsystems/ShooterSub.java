// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */

  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
 

  public ShooterSub() {

    leftShooterMotor = new TalonFX(Constants.LEFT_SHOOTER_MOTOR);
    rightShooterMotor = new TalonFX(Constants.RIGHT_SHOOTER_MOTOR);

    leftShooterMotor.setInverted(true);
    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();

    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);

    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(true);

    rightShooterMotor.setSensorPhase(false);
    leftShooterMotor.setSensorPhase(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double setShooterMotorsVelocity(double TargetShooterVelocity, double Integral){

    leftShooterMotor.follow(rightShooterMotor);

    double currentShooterVelocity = rightShooterMotor.getSelectedSensorVelocity();

    double VelocityError = TargetShooterVelocity - currentShooterVelocity;

    Integral = Math.max(Math.min(Integral+VelocityError*Constants.SHOOTER_INTERGRAL_GAIN,Constants.MAX_ARM_INTEGRAL),-Constants.MAX_ARM_INTEGRAL);

    double power = VelocityError*Constants.SHOOTER_PORPORTIONAL_GAIN;

    double finalPower = Math.max(power + Constants.POWER_OFFSET+Integral,0);
    
    System.out.println(VelocityError);

    rightShooterMotor.set(ControlMode.PercentOutput, finalPower);

    SmartDashboard.putNumber("Shooter Integral", Integral);
    SmartDashboard.putNumber("Final Power", finalPower);
    SmartDashboard.putNumber("Velocity Error", VelocityError);

    return Integral;

  }

  public void setShooterMotorsPower(double Speed){
    leftShooterMotor.set(ControlMode.PercentOutput, Speed);
    rightShooterMotor.set(ControlMode.PercentOutput, Speed);
  }

}
