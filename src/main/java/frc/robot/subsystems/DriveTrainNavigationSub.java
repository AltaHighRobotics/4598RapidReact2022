// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConfigurablePID;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrainNavigationSub extends SubsystemBase {
  /** Creates a new DriveTrainPIDSub. */
  private TalonFX rightMotorFront;
  private TalonFX rightMotorBack;
  private TalonFX leftMotorFront;
  private TalonFX leftMotorBack;

  private ConfigurablePID drivetrainHeadingPID;
  private ConfigurablePID drivetrainSpeedPID;
  private SupplyCurrentLimitConfiguration drivetrainCurrentLimit;

  private AHRS navX;

  private double rightMotorVelocity;
  private double leftMotorVelocity;

  private double currentRightMotorPosition;
  private double currentLeftMotorPosition;

  private double drivePower;
  private double steeringPower;

  private double currentHeading;
  private double previousHeading;
  private double headingError;
  private double headingRate;

  private double previousRightMotorPosition;
  private double previousLeftMotorPosition;
  private double distanceTraveledLeft;
  private double distanceTraveledRight;
  private double distanceTraveled;
  private double distanceError;
  private double robotX;
  private double robotY;

  private double targetX;
  private double targetY;
  private double targetHeading;

  public DriveTrainNavigationSub() {

    this.robotY = 0;
    this.robotX = 0;
    this.drivetrainCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.DRIVETRAIN_CURRENT_LIMIT, 0, 0.1);

    this.drivetrainHeadingPID = new ConfigurablePID(
      Constants.DRIVETRAIN_HEADING_PROPORTIONAL_GAIN,
      Constants.DRIVETRAIN_HEADING_INTEGRAL_GAIN,
      Constants.DRIVETRAIN_HEADING_DERIVITIVE_GAIN,
      Constants.DRIVETRAIN_HEADING_MAX_PROPORTIONAL,
      Constants.DRIVETRAIN_HEADING_MAX_INTEGRAL,
      Constants.DRIVETRAIN_HEADING_MAX_DERIVITIVE,
      -Constants.DRIVETRAIN_HEADING_MAX_POWER,
      Constants.DRIVETRAIN_HEADING_MAX_POWER,
      Constants.DRIVETRAIN_HEADING_SPEED
    );

    this.drivetrainSpeedPID = new ConfigurablePID(
      Constants.DRIVETRAIN_SPEED_PROPORTIONAL_GAIN,
      Constants.DRIVETRAIN_SPEED_INTEGRAL_GAIN,
      Constants.DRIVETRAIN_SPEED_DERIVITIVE_GAIN,
      Constants.DRIVETRAIN_SPEED_MAX_PROPORTIONAL,
      Constants.DRIVETRAIN_SPEED_MAX_INTEGRAL,
      Constants.DRIVETRAIN_SPEED_MAX_DERIVITIVE,
      -Constants.DRIVETRAIN_SPEED_MAX_POWER,
      Constants.DRIVETRAIN_SPEED_MAX_POWER,
      Constants.DRIVETRAIN_SPEED_SPEED
    );

    this.navX = new AHRS(SPI.Port.kMXP);

    this.rightMotorFront = new TalonFX(Constants.RIGHT_DRIVE_MOTOR_FRONT);
    this.rightMotorBack = new TalonFX(Constants.RIGHT_DRIVE_MOTOR_BACK);
    this.leftMotorFront = new TalonFX(Constants.LEFT_DRIVE_MOTOR_FRONT);
    this.leftMotorBack = new TalonFX(Constants.LEFT_DRIVE_MOTOR_BACK);

    this.rightMotorFront.configFactoryDefault();
    this.rightMotorBack.configFactoryDefault();
    this.leftMotorFront.configFactoryDefault();
    this.rightMotorFront.configFactoryDefault();

    this.rightMotorFront.setSensorPhase(false);
    this.rightMotorBack.setSensorPhase(false);
    this.leftMotorFront.setSensorPhase(true);
    this.leftMotorBack.setSensorPhase(true);

    this.rightMotorFront.setInverted(TalonFXInvertType.Clockwise);
    this.rightMotorBack.setInverted(TalonFXInvertType.Clockwise);
    this.leftMotorFront.setInverted(TalonFXInvertType.CounterClockwise);
    this.leftMotorBack.setInverted(TalonFXInvertType.CounterClockwise);

    this.rightMotorFront.setNeutralMode(NeutralMode.Brake);
    this.rightMotorBack.setNeutralMode(NeutralMode.Brake);
    this.leftMotorFront.setNeutralMode(NeutralMode.Brake);
    this.leftMotorBack.setNeutralMode(NeutralMode.Brake);

    this.rightMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
    this.rightMotorBack.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
    this.leftMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
    this.rightMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);

    this.rightMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);
    this.rightMotorBack.configSupplyCurrentLimit(drivetrainCurrentLimit);
    this.leftMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);
    this.rightMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);

    this.rightMotorBack.follow(this.rightMotorFront);
    this.leftMotorBack.follow(this.leftMotorFront);

  }

  public void driveTrainPosIntegration()
  {
    this.currentLeftMotorPosition = this.leftMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS;
    this.currentRightMotorPosition = this.rightMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS;

    this.currentHeading = (double) this.navX.getYaw();

    SmartDashboard.putNumber("Robot Heading:", this.currentHeading);

    this.currentHeading = Math.toRadians(this.currentHeading);

    this.distanceTraveledLeft = Constants.DRIVETRAIN_ROTATION_DISTANCE_RATIO * Constants.DRIVETRAIN_GEAR_RATIO * (this.currentLeftMotorPosition - this.previousLeftMotorPosition);
    this.distanceTraveledRight = Constants.DRIVETRAIN_ROTATION_DISTANCE_RATIO * Constants.DRIVETRAIN_GEAR_RATIO * (this.currentRightMotorPosition - this.previousRightMotorPosition);
    this.distanceTraveled = (this.distanceTraveledLeft + this.distanceTraveledRight)/2;

    this.previousLeftMotorPosition = this.currentLeftMotorPosition;
    this.previousRightMotorPosition = this.currentRightMotorPosition;

    this.robotX = this.robotX + (Math.cos(this.currentHeading) * this.distanceTraveled);
    this.robotY = this.robotY + (Math.sin(this.currentHeading) * -this.distanceTraveled);

    SmartDashboard.putNumber("Robot X:", this.robotX);
    SmartDashboard.putNumber("Robot Y:", this.robotY);

  }

  public void setDriveToWaypoint(double waypointX, double waypointY)
  {
    this.targetX = waypointX;
    this.targetY = waypointY;

    this.targetHeading = Math.toDegrees(Math.atan2(-(this.targetY - this.robotY), this.targetX - this.robotX));

    this.currentHeading = (double) this.navX.getYaw();

    this.headingRate = this.currentHeading - this.previousHeading;
    this.headingError = this.targetHeading - this.currentHeading;
    this.previousHeading = this.currentHeading;

    this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading, this.headingRate);

    SmartDashboard.putNumber("Heading Error:", this.headingError);

    if(Math.abs(this.headingError) < Constants.MAX_DRIVE_HEADING_ERROR)
    {
      this.distanceError = Math.sqrt(Math.pow(this.targetY - this.robotY, 2) + Math.pow(this.targetX - this.robotX, 2));
      this.drivePower = this.drivetrainSpeedPID.runPID(0, -this.distanceError);

      SmartDashboard.putNumber("Distance to Waypoint:", this.distanceError);
    } else {
      this.drivePower = 0;
    }

    SmartDashboard.putNumber("Auto Throttle:", this.drivePower);
    SmartDashboard.putNumber("Auto Steering:", this.steeringPower);

    //this.setMotorPower(this.drivePower, this.steeringPower);
    this.leftMotorFront.set(ControlMode.PercentOutput, -this.steeringPower);
    //this.leftMotorBack.set(ControlMode.PercentOutput, this.steeringPower);
    this.rightMotorFront.set(ControlMode.PercentOutput, this.steeringPower);
    //this.rightMotorBack.set(ControlMode.PercentOutput, this.steeringPower);
  }

  public boolean hasReachedWaypoint()
  {
    return Math.abs(this.distanceError) < Constants.MAX_WAYPOINT_ERROR;
  }

  public void setPos(double x, double y)
  {
    this.robotX = x; 
    this.robotY = y;
  }

  private void setMotorPower(double throttle, double rotation) {
    this.leftMotorFront.set(ControlMode.PercentOutput, throttle + rotation);
    this.leftMotorBack.set(ControlMode.PercentOutput, throttle + rotation);
    this.rightMotorFront.set(ControlMode.PercentOutput, throttle - rotation);
    this.rightMotorBack.set(ControlMode.PercentOutput, throttle - rotation);
  }

  public void stop() {
    setMotorPower(0, 0);
  }

  @Override
  public void periodic() {}
}
