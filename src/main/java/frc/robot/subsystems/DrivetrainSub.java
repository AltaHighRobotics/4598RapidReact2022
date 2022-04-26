// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.ConfigurablePID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSub extends SubsystemBase 
{
  private final WPI_TalonFX rightMotorFront;
  private final WPI_TalonFX rightMotorBack;
  private final WPI_TalonFX leftMotorFront;
  private final WPI_TalonFX leftMotorBack;
  private final SupplyCurrentLimitConfiguration drivetrainCurrentLimit;

  private final ConfigurablePID drivetrainHeadingPID;
  private final ConfigurablePID drivetrainSpeedPID;

  private final AHRS navX;

  //private double rightMotorVelocity;
  //private double leftMotorVelocity;

  private double [] pos = {-9999, -9999};

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
  
  /** Creates a new DriveTrainSub. */
  public DrivetrainSub() {
    this.robotY = 0;
    this.robotX = 0;
    this.targetX = 0;
    this.targetY = 0;
    this.targetHeading = 0;

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

    this.drivetrainCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.DRIVETRAIN_CURRENT_LIMIT, 0, 0);

    this.rightMotorFront = new WPI_TalonFX(Constants.RIGHT_DRIVE_MOTOR_FRONT);
    this.rightMotorBack = new WPI_TalonFX(Constants.RIGHT_DRIVE_MOTOR_BACK);
    this.leftMotorFront = new WPI_TalonFX(Constants.LEFT_DRIVE_MOTOR_FRONT);
    this.leftMotorBack = new WPI_TalonFX(Constants.LEFT_DRIVE_MOTOR_BACK);

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
    
    this.rightMotorBack.setStatusFramePeriod(1, 255);
    this.rightMotorBack.setStatusFramePeriod(2, 255);
    this.leftMotorBack.setStatusFramePeriod(1, 255);
    this.leftMotorBack.setStatusFramePeriod(2, 255);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArcadeDrive(final double forward, final double turn) {
    this.rightMotorFront.set(ControlMode.PercentOutput, forward * Constants.DRIVE_MAX_SPEED, DemandType.ArbitraryFeedForward, -turn*(Constants.DRIVE_MAX_SPEED * 0.666));
    this.leftMotorFront.set(ControlMode.PercentOutput, forward * Constants.DRIVE_MAX_SPEED, DemandType.ArbitraryFeedForward, turn*(Constants.DRIVE_MAX_SPEED * 0.666));
  }

  public void drivetrainPositionIntegration() {
    this.currentLeftMotorPosition = this.leftMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS;
    this.currentRightMotorPosition = this.rightMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS;

    this.currentHeading = (double) this.navX.getYaw();

    //SmartDashboard.putNumber("Robot Heading:", this.currentHeading);

    this.currentHeading = Math.toRadians(this.currentHeading);

    this.distanceTraveledLeft = Constants.DRIVETRAIN_ROTATION_DISTANCE_RATIO * Constants.DRIVETRAIN_GEAR_RATIO * (this.currentLeftMotorPosition - this.previousLeftMotorPosition);
    this.distanceTraveledRight = Constants.DRIVETRAIN_ROTATION_DISTANCE_RATIO * Constants.DRIVETRAIN_GEAR_RATIO * (this.currentRightMotorPosition - this.previousRightMotorPosition);
    this.distanceTraveled = (this.distanceTraveledLeft + this.distanceTraveledRight)/2;

    this.previousLeftMotorPosition = this.currentLeftMotorPosition;
    this.previousRightMotorPosition = this.currentRightMotorPosition;

    this.robotX = this.robotX + (Math.cos(this.currentHeading) * this.distanceTraveled);
    this.robotY = this.robotY + (Math.sin(this.currentHeading) * this.distanceTraveled);

    //SmartDashboard.putNumber("Robot X:", this.robotX);
    //SmartDashboard.putNumber("Robot Y:", this.robotY);

  }

  public void driveForwardTo(final double waypointX, final double waypointY)
  {

  }

  public void driveBackwardsTo(final double waypointX, final double waypointY)
  {
    
  }

  public double [] getPos()
  {
    pos[0] = robotX;
    pos[1] = robotY;
    return pos;
  }
  public boolean setDriveToWaypoint(final double waypointX, final double waypointY, final boolean driveBackwards) {
    this.targetX = waypointX;
    this.targetY = waypointY;
    if(!driveBackwards) {
      this.targetHeading = Math.toDegrees(Math.atan2(this.targetY - this.robotY, this.targetX - this.robotX));
    } else {
      this.targetHeading = Math.toDegrees(Math.atan2(-(this.targetY - this.robotY), -(this.targetX - this.robotX)));
    }
    this.currentHeading = (double) this.navX.getYaw();
    //SmartDashboard.putNumber("Target Heading", this.targetHeading);
    this.headingRate = this.currentHeading - this.previousHeading;
    this.headingError = this.targetHeading - this.currentHeading;
    this.previousHeading = this.currentHeading;
    //SmartDashboard.putNumber("Heading Error:", this.headingError);

    this.distanceError = Math.sqrt(Math.pow(this.targetY - this.robotY, 2) + Math.pow(this.targetX - this.robotX, 2));
    //SmartDashboard.putNumber("Distance to Waypoint:", this.distanceError);

    this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading, this.headingRate);

    if(Math.abs(this.headingError) < Constants.MAX_DRIVE_HEADING_ERROR) {
      this.drivePower = this.drivetrainSpeedPID.runPID(0, -this.distanceError);
      if(driveBackwards) {
        this.drivePower = -this.drivePower;
      }
    } else {
      this.drivePower = 0;
    }
    //SmartDashboard.putNumber("Target X", this.targetX);
    //SmartDashboard.putNumber("Target Y", this.targetY);
    //SmartDashboard.putNumber("Auto Throttle:", this.drivePower);
    //SmartDashboard.putNumber("Auto Steering:", this.steeringPower);
    if(hasReachedWaypoint()) {
      stopMotors();
    } else {
      this.setArcadeDrive(this.drivePower, this.steeringPower);
    }
    return hasReachedWaypoint();
  }

  public boolean pointAtWaypoint(final double waypointX, final double waypointY) {
    this.targetX = waypointX;
    this.targetY = waypointY;
    this.targetHeading = Math.toDegrees(Math.atan2(this.targetY - this.robotY, this.targetX - this.robotX));

    this.currentHeading = (double) this.navX.getYaw();
    //SmartDashboard.putNumber("Target Heading", this.targetHeading);
    this.headingRate = this.currentHeading - this.previousHeading;
    this.headingError = this.targetHeading - this.currentHeading;
    this.previousHeading = this.currentHeading;
    //SmartDashboard.putNumber("Heading Error:", this.headingError);

    this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading, this.headingRate);
    this.setArcadeDrive(0, this.steeringPower);
    return Math.abs(this.currentHeading - this.targetHeading) < Constants.MAX_DRIVE_HEADING_ERROR;
  }

  public void pointAtAngle(double targetAngle) {
    this.targetHeading = targetAngle;

    this.currentHeading = (double) this.navX.getYaw();
    //SmartDashboard.putNumber("Target Heading", this.targetHeading);
    this.headingRate = this.currentHeading - this.previousHeading;
    this.headingError = this.targetHeading - this.currentHeading;
    this.previousHeading = this.currentHeading;
    //SmartDashboard.putNumber("Heading Error:", this.headingError);

    this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading, this.headingRate);
    this.setArcadeDrive(0, this.steeringPower);
  }

  public boolean hasReachedWaypoint() {
    //SmartDashboard.putNumber("DISTANCE ERROR", distanceError);
    return Math.abs(this.distanceError) < Constants.MAX_WAYPOINT_ERROR;
  }

  public void stopMotors() {
    this.leftMotorFront.neutralOutput();
    this.rightMotorFront.neutralOutput();
  }

  public void setMotorAuto()
  {
    leftMotorFront.set(ControlMode.PercentOutput, -0.3);    
    rightMotorFront.set(ControlMode.PercentOutput, -0.3);  
  }

  // public void stopMotorAuto()
  // {
  //   leftMotorFront.set(ControlMode.PercentOutput, 0);  
  //   leftMotorBack.set(ControlMode.PercentOutput, 0);  
  //   rightMotorFront.set(ControlMode.PercentOutput, 0);  
  //   rightMotorBack.set(ControlMode.PercentOutput, 0);  
  //}

  public void setPos(double x, double y) {
    this.robotX = x; 
    this.robotY = y;
  }

  public void resetYaw() {
    this.navX.zeroYaw();
  }
}