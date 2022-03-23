// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.ConfigurablePID;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */

  private final WPI_TalonFX leftShooterMotor;
  private final WPI_TalonFX rightShooterMotor;
  private final ConfigurablePID leftShooterPID;
  private final ConfigurablePID rightShooterPID;
  private final SupplyCurrentLimitConfiguration shooterCurrentLimit;

  public ShooterSub() {

    shooterCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT, 0, 0.1);

    leftShooterPID = new ConfigurablePID(
      Constants.SHOOTER_PORPORTIONAL_GAIN,
      Constants.SHOOTER_INTERGRAL_GAIN,
      Constants.SHOOTER_DERIVITIVE_GAIN,
      Constants.SHOOTER_MAX_PROPORTIONAL,
      Constants.SHOOTER_MAX_INTEGRAL,
      Constants.SHOOTER_MAX_DERIVITIVE,
      Constants.SHOOTER_POWER_OFFSET,
      1,
      1
    );

    rightShooterPID = new ConfigurablePID(
      Constants.SHOOTER_PORPORTIONAL_GAIN,
      Constants.SHOOTER_INTERGRAL_GAIN,
      Constants.SHOOTER_DERIVITIVE_GAIN,
      Constants.SHOOTER_MAX_PROPORTIONAL,
      Constants.SHOOTER_MAX_INTEGRAL,
      Constants.SHOOTER_MAX_DERIVITIVE,
      Constants.SHOOTER_POWER_OFFSET,
      1,
      1
    );

    leftShooterMotor = new WPI_TalonFX(Constants.LEFT_SHOOTER_MOTOR);
    rightShooterMotor = new WPI_TalonFX(Constants.RIGHT_SHOOTER_MOTOR);

    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();

    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);

    leftShooterMotor.setSensorPhase(false);
    rightShooterMotor.setSensorPhase(false);

    leftShooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightShooterMotor.setInverted(TalonFXInvertType.Clockwise);

    leftShooterMotor.configOpenloopRamp(Constants.SHOOTER_POWER_RAMP_TIME, 0);
    rightShooterMotor.configOpenloopRamp(Constants.SHOOTER_POWER_RAMP_TIME, 0);

    leftShooterMotor.configSupplyCurrentLimit(shooterCurrentLimit);
    rightShooterMotor.configSupplyCurrentLimit(shooterCurrentLimit);
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

    // Runs the controllers
    double shooterLeftPower = leftShooterPID.runPID(targetShooterVelocity, shooterLeftVelocity);
    double shooterRightPower = rightShooterPID.runPID(targetShooterVelocity, shooterRightVelocity);

    // Sets the motors to the computed power levels
    leftShooterMotor.set(ControlMode.PercentOutput, shooterLeftPower);
    rightShooterMotor.set(ControlMode.PercentOutput, shooterRightPower);

    // Displays useful values in Smart Dashboard
    SmartDashboard.putNumber("Shooter Left Power:", shooterLeftPower);
    SmartDashboard.putNumber("Shooter Right Power:", shooterRightPower);
    SmartDashboard.putString("Shooter Status:", "Shooting");
  }

  public void setShooterMotorsPower(double Speed){
    leftShooterMotor.set(ControlMode.PercentOutput, Speed);
    rightShooterMotor.set(ControlMode.PercentOutput, Speed);
  }

  public void stopShooterMotors(){
    leftShooterMotor.set(ControlMode.PercentOutput, 0);
    rightShooterMotor.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putString("Shooter Status:", "Stopped");
  }

}
