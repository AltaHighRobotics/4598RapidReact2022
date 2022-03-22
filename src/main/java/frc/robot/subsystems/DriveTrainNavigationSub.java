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

  public AHRS navX;

  private double rightMotorVelocity;
  private double leftMotorVelocity;

  private double currentRightMotorPosition;
  private double currentLeftMotorPosition;

  private double leftDrivePower;
  private double rightDrivePower;
  private double speedPower;
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
    drivetrainCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.DRIVETRAIN_CURRENT_LIMIT, 0, 0.1);

    drivetrainHeadingPID = new ConfigurablePID(
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

    drivetrainSpeedPID = new ConfigurablePID(
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

    navX = new AHRS(SPI.Port.kMXP);

    rightMotorFront = new TalonFX(Constants.RIGHT_DRIVE_MOTOR_FRONT);
    rightMotorBack = new TalonFX(Constants.RIGHT_DRIVE_MOTOR_BACK);
    leftMotorFront = new TalonFX(Constants.LEFT_DRIVE_MOTOR_FRONT);
    leftMotorBack = new TalonFX(Constants.LEFT_DRIVE_MOTOR_BACK);

    rightMotorFront.configFactoryDefault();
    rightMotorBack.configFactoryDefault();
    leftMotorFront.configFactoryDefault();
    rightMotorFront.configFactoryDefault();

    rightMotorFront.setSensorPhase(false);
    rightMotorBack.setSensorPhase(false);
    leftMotorFront.setSensorPhase(true);
    leftMotorBack.setSensorPhase(true);

    rightMotorFront.setInverted(TalonFXInvertType.Clockwise);
    rightMotorBack.setInverted(TalonFXInvertType.Clockwise);
    leftMotorFront.setInverted(TalonFXInvertType.CounterClockwise);
    leftMotorBack.setInverted(TalonFXInvertType.CounterClockwise);

    rightMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
    rightMotorBack.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
    leftMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);
    rightMotorFront.configOpenloopRamp(Constants.DRIVETRAIN_POWER_RAMP_TIME, 0);

    rightMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);
    rightMotorBack.configSupplyCurrentLimit(drivetrainCurrentLimit);
    leftMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);
    rightMotorFront.configSupplyCurrentLimit(drivetrainCurrentLimit);

  }

  public void driveTrainPosIntegration()
  {
    this.currentLeftMotorPosition = leftMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS;
    this.currentRightMotorPosition = rightMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS;

    this.currentHeading = (double) navX.getYaw();

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

    this.currentHeading = (double) navX.getYaw();

    this.headingRate = this.currentHeading - this.previousHeading;
    this.headingError = this.targetHeading - this.currentHeading;
    this.previousHeading = this.currentHeading;

    this.steeringPower = -drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading, this.headingRate);

    SmartDashboard.putNumber("Heading Error:", this.headingError);

    this.leftDrivePower = -this.steeringPower;
    this.rightDrivePower = this.steeringPower;

    if(Math.abs(this.headingError) < Constants.MAX_DRIVE_HEADING_ERROR)
    {
      this.distanceError = Math.sqrt(Math.pow(this.targetY - this.robotY, 2) + Math.pow(this.targetX - this.robotX, 2));
      this.speedPower = drivetrainSpeedPID.runPID(0, -this.distanceError);

      SmartDashboard.putNumber("Distance to Waypoint:", this.distanceError);

      this.leftDrivePower = this.leftDrivePower + this.speedPower;
      this.rightDrivePower = this.rightDrivePower + this.speedPower;
    }

    SmartDashboard.putNumber("Left Power:", leftDrivePower);
    SmartDashboard.putNumber("Right Power:", rightDrivePower);

    leftMotorFront.set(ControlMode.PercentOutput, leftDrivePower);
    leftMotorBack.set(ControlMode.PercentOutput, leftDrivePower);
    rightMotorFront.set(ControlMode.PercentOutput, rightDrivePower);
    rightMotorBack.set(ControlMode.PercentOutput, rightDrivePower);
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

  @Override
  public void periodic() {}
}
