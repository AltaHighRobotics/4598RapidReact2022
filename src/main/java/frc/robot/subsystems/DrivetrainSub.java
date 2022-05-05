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
import frc.robot.utilities.ConfigurablePID;
import frc.robot.utilities.vector;
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

  private double drivePower;
  private double steeringPower;

  private double currentHeading;
  private double previousHeading;
  private double headingError;
  private double headingRate;

  private vector currentMotorPositions;
  private vector previousMotorPositions;
  private vector motorVelocities;
  private vector velocity;
  private vector position;
  private vector positionError;

  private vector target;
  private double targetHeading;
  private double targetSpeed;
  
  /** Creates a new DriveTrainSub. */
  public DrivetrainSub() {
    this.drivePower = 0;
    this.steeringPower = 0;
    this.targetHeading = 0;
    this.position = new vector(0, 0);
    this.velocity = new vector(0, 0);
    this.target = new vector(0, 0);
    this.positionError = new vector(0, 0);
    this.currentMotorPositions = new vector(0, 0);
    this.previousMotorPositions = new vector(0, 0);
    this.motorVelocities = new vector(0, 0);
    this.targetSpeed = 0;

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

  public void setArcadeDrive(final double forward, final double turn)
  {
    this.rightMotorFront.set(ControlMode.PercentOutput, forward * Constants.DRIVE_MAX_SPEED, DemandType.ArbitraryFeedForward, -turn*(Constants.DRIVE_MAX_SPEED * 0.666));
    this.leftMotorFront.set(ControlMode.PercentOutput, forward * Constants.DRIVE_MAX_SPEED, DemandType.ArbitraryFeedForward, turn*(Constants.DRIVE_MAX_SPEED * 0.666));
  }

  public void drivetrainPositionIntegration()
  {
    this.drivePower = 0;
    this.steeringPower = 0;
    this.currentMotorPositions.set(
      this.leftMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS,
      this.rightMotorFront.getSelectedSensorPosition() / Constants.ENCODER_ROTATION_UNITS
    );
    this.motorVelocities = this.currentMotorPositions.getSubtraction(this.previousMotorPositions);
    this.previousMotorPositions.copy(this.currentMotorPositions);
    this.motorVelocities.multiply(Constants.DRIVETRAIN_ROTATION_DISTANCE_RATIO * Constants.DRIVETRAIN_GEAR_RATIO);
    this.motorVelocities.average();

    this.currentHeading = (double) this.navX.getYaw();

    this.currentHeading = Math.toRadians(this.currentHeading);

    this.velocity.set((Math.cos(this.currentHeading) * this.motorVelocities.average), (Math.sin(this.currentHeading) * this.motorVelocities.average));
    this.position.add(this.velocity);

    SmartDashboard.putNumber("Robot X:", this.position.x);
    SmartDashboard.putNumber("Robot Y:", this.position.y);

  }

  public void driveForwardTo(final vector waypoint)
  {
    this.setWaypoint(waypoint);
    this.setTargetHeadingToWaypoint();
    this.setSteeringPowerToTargetHeading(false);
    this.setDriveToWaypoint();
    if(this.hasReachedWaypoint()) {
      this.stopMotors();
    } else {
      this.setArcadeDrive(this.drivePower, this.steeringPower);
    }
  }

  public void driveBackwardTo(final vector waypoint)
  {
    this.setWaypoint(waypoint);
    this.setTargetHeadingToWaypoint();
    this.setSteeringPowerToTargetHeading(true);
    this.setDriveToWaypoint();
    if(this.hasReachedWaypoint()) {
      this.stopMotors();
    } else {
      this.setArcadeDrive(this.drivePower, this.steeringPower);
    }
  }

  public vector getPos()
  {
    return this.position.clone();
  }

  public void setWaypoint(final vector waypoint)
  {
    this.target.copy(waypoint);
  }

  private void setTargetHeadingToWaypoint()
  {
    this.positionError = this.target.getSubtraction(this.position);
    this.targetHeading = Math.toDegrees(Math.atan2(this.positionError.y, this.positionError.x));
  }

  private void setSteeringPowerToTargetHeading(boolean backwards)
  {
    this.currentHeading = (double) this.navX.getYaw();
    this.headingRate = this.currentHeading - this.previousHeading;
    this.headingError = this.targetHeading - this.currentHeading;
    this.previousHeading = this.currentHeading;
    if (backwards) {
      this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading, this.currentHeading, this.headingRate);
    } else {
      this.steeringPower = this.drivetrainHeadingPID.runVelocityPID(this.targetHeading+180, this.currentHeading, this.headingRate);
    }
  }

  private void setDriveToWaypoint()
  {
    this.targetSpeed = Math.cos(Math.toRadians(this.headingError)) * this.positionError.magnitude();
    this.drivePower = this.drivetrainSpeedPID.runPID(targetSpeed, this.motorVelocities.average);
  }

  public boolean aimRobot(final vector waypoint)
  {
    this.setWaypoint(waypoint);
    this.setTargetHeadingToWaypoint();
    this.setSteeringPowerToTargetHeading(false);
    this.setArcadeDrive(this.drivePower, this.steeringPower);
    return Math.abs(this.currentHeading - this.targetHeading) < Constants.MAX_DRIVE_HEADING_ERROR;
  }

  public boolean aimRobot(final double targetAngle)
  {
    this.targetHeading = targetAngle;
    this.setSteeringPowerToTargetHeading(false);
    this.setArcadeDrive(this.drivePower, this.steeringPower);
    return Math.abs(this.currentHeading - this.targetHeading) < Constants.MAX_DRIVE_HEADING_ERROR;
  }

  public boolean hasReachedWaypoint()
  {
    return Math.abs(this.positionError.magnitude()) < Constants.MAX_WAYPOINT_ERROR;
  }

  public boolean isAtPoint(vector point)
  {
    return Math.abs(this.position.getSubtraction(point).magnitude()) < Constants.MAX_WAYPOINT_ERROR;
  }

  public void stopMotors()
  {
    this.leftMotorFront.neutralOutput();
    this.rightMotorFront.neutralOutput();
  }

  public void setMotorAuto()
  {
    leftMotorFront.set(ControlMode.PercentOutput, -0.3);    
    rightMotorFront.set(ControlMode.PercentOutput, -0.3);  
  }

  public void setPos(vector newPosition)
  {
    this.position.copy(newPosition);
  }

  public void resetYaw()
  {
    this.navX.zeroYaw();
  }
}