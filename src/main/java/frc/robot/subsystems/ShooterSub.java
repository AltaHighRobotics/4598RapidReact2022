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
  private double shooterPowers [];
  private double shooterErrors [];
  private double shooterLeftIntegral;
  private double shooterRightIntegral;
 

  public ShooterSub() {

    leftShooterMotor = new TalonFX(Constants.LEFT_SHOOTER_MOTOR);
    rightShooterMotor = new TalonFX(Constants.RIGHT_SHOOTER_MOTOR);

    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();

    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);

    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(true);

    rightShooterMotor.setSensorPhase(false);
    leftShooterMotor.setSensorPhase(false);

    shooterPowers = new double [2];
    shooterErrors = new double [2];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Proportional Integral Controller used to set both of the shooter motors speed
   *  Adjusts both motors power to match a target velocity
   *  @param targetShooterVelocity A double representing the target velocity that the shooter motors should attempt to reach
   */
  public void setShooterMotorsVelocity(double targetShooterVelocity){
    // Gets the velocity of each of the two shooter motors
    double shooterLeftVelocity = leftShooterMotor.getSelectedSensorVelocity();
    double shooterRightVelocity = rightShooterMotor.getSelectedSensorVelocity();

    // Finds the difference between the target velocity and the current velocity for each motor
    double shooterLeftVelocityError = targetShooterVelocity - shooterLeftVelocity;
    double shooterRightVelocityError = targetShooterVelocity - shooterRightVelocity;

    // Adds the current error to the sum of all past errors, which allows the controller to find the exact power level needed
    shooterLeftIntegral = Math.max(Math.min(shooterLeftIntegral + shooterLeftVelocityError * Constants.SHOOTER_INTERGRAL_GAIN, Constants.MAX_ARM_INTEGRAL), -Constants.MAX_ARM_INTEGRAL);
    shooterRightIntegral = Math.max(Math.min(shooterRightIntegral + shooterRightVelocityError * Constants.SHOOTER_INTERGRAL_GAIN, Constants.MAX_ARM_INTEGRAL), -Constants.MAX_ARM_INTEGRAL);

    // Adds the current error * a constant gain to the output power, which improves response time for changes in target velocity
    double shooterLeftPower = shooterLeftIntegral + shooterLeftVelocityError * Constants.SHOOTER_PORPORTIONAL_GAIN;
    double shooterRightPower = shooterRightIntegral + shooterRightVelocityError * Constants.SHOOTER_PORPORTIONAL_GAIN;

    // Clamps the power output above the power offset value, which ensures the motors don't apply brakes, as that would cause instability and vibrations
    shooterLeftPower = Math.max(shooterLeftPower, Constants.SHOOTER_POWER_OFFSET);
    shooterRightPower = Math.max(shooterRightPower, Constants.SHOOTER_POWER_OFFSET);

    // Sets the motors to the computed power levels
    leftShooterMotor.set(ControlMode.PercentOutput, shooterLeftPower);
    rightShooterMotor.set(ControlMode.PercentOutput, shooterRightPower);

    // Displays useful values in Smart Dashboard
    shooterPowers[0] = shooterLeftPower;
    shooterPowers[1] = shooterRightPower;
    shooterErrors[0] = shooterLeftVelocityError;
    shooterErrors[1] = shooterRightVelocityError;

    SmartDashboard.putNumberArray("Shooter Power:", shooterPowers);
    SmartDashboard.putNumberArray("Shooter Velocity Errors:", shooterErrors);
  }

  public void setShooterMotorsPower(double Speed){
    leftShooterMotor.set(ControlMode.PercentOutput, Speed);
    rightShooterMotor.set(ControlMode.PercentOutput, Speed);
  }

}
