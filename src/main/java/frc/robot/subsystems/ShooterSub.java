// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */

  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFX azimuthMotor;
  private TalonSRX elevationAngleMotor;
  private double shooterPowers [];
  private double shooterErrors [];
  private double shooterLeftIntegral;
  private double shooterRightIntegral;
 

  public ShooterSub() {

    leftShooterMotor = new TalonFX(Constants.LEFT_SHOOTER_MOTOR);
    rightShooterMotor = new TalonFX(Constants.RIGHT_SHOOTER_MOTOR);
    azimuthMotor = new TalonFX(Constants.AZIMUTH_MOTOR);
    elevationAngleMotor = new TalonSRX(Constants.ELEVATION_ANGLE_MOTOR);

    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();
    azimuthMotor.configFactoryDefault();
    elevationAngleMotor.configFactoryDefault();

    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    azimuthMotor.setNeutralMode(NeutralMode.Brake);
    elevationAngleMotor.setNeutralMode(NeutralMode.Brake);

    leftShooterMotor.setSensorPhase(false);
    rightShooterMotor.setSensorPhase(false);
    azimuthMotor.setSensorPhase(false);
    elevationAngleMotor.setSensorPhase(false);

    leftShooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightShooterMotor.setInverted(TalonFXInvertType.Clockwise);
    azimuthMotor.setInverted(TalonFXInvertType.CounterClockwise);
    elevationAngleMotor.setInverted(false);

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

  /** Proportional Controller used to set the elevation angle for the shooter
   *  Adjusts the motor power to go to a target angle, in degrees.
   *  @param targetElevationAngle A double representing the target angle that the elevation angle motor should attempt to reach
   */
  public void MoveElevationMotorToAngle(double targetElevationAngle){
    // Converts the target from degrees to encoder units
    targetElevationAngle = targetElevationAngle/360 * 4096;

    // Gets the current rotation of the elevation motor, relative to the rotation upon robot power-up (1 rotation = 4096 units)
    double currentElevationAngle = elevationAngleMotor.getSelectedSensorPosition();

    // Gets the difference between the target angle and the current angle
    double elevationAngleError = targetElevationAngle - currentElevationAngle;

    // Sets the motor power to the error in elevation * a constant gain
    double elevationMotorPower = elevationAngleError * Constants.ELEVATION_ANGLE_PROPORTIONAL_GAIN;
    elevationAngleMotor.set(ControlMode.PercentOutput, elevationMotorPower);
  }

}
