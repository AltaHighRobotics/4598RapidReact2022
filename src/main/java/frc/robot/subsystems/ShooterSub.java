// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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

  public double setShooterMotorVelocity(double targetShooterVelocity, double integral, int motorID){

    double currentShooterVelocity = 0;

    if(motorID == Constants.LEFT_SHOOTER_MOTOR) {
      currentShooterVelocity = leftShooterMotor.getSelectedSensorVelocity();
    } else {
      currentShooterVelocity = rightShooterMotor.getSelectedSensorVelocity();
    }

    double velocityError = targetShooterVelocity - currentShooterVelocity;

    integral = Math.max(Math.min(integral + velocityError * Constants.SHOOTER_INTERGRAL_GAIN, Constants.MAX_ARM_INTEGRAL), -Constants.MAX_ARM_INTEGRAL);

    double power = velocityError*Constants.SHOOTER_PORPORTIONAL_GAIN;

    double finalPower = Math.max(power + integral + Constants.POWER_OFFSET, 0);

    if(motorID == Constants.LEFT_SHOOTER_MOTOR) {
      leftShooterMotor.set(ControlMode.PercentOutput, finalPower);
    } else {
      rightShooterMotor.set(ControlMode.PercentOutput, finalPower);
    }

    SmartDashboard.putNumber("Shooter Integral", integral);
    SmartDashboard.putNumber("Final Power", finalPower);
    SmartDashboard.putNumber("Velocity Error", velocityError);

    return integral;

  }

  public void setShooterMotorsPower(double Speed){
    leftShooterMotor.set(ControlMode.PercentOutput, Speed);
    rightShooterMotor.set(ControlMode.PercentOutput, Speed);
  }

}
