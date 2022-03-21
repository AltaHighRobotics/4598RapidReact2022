// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConfigurablePID;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

public class DriveTrainNavigationSub extends SubsystemBase {
  /** Creates a new DriveTrainPIDSub. */
  public TalonFX rightMotorFront;
  public TalonFX rightMotorBack;
  public TalonFX leftMotorFront;
  public TalonFX leftMotorBack;

  private ConfigurablePID drivetrainHeadingPID;
  private ConfigurablePID drivetrainSpeedPID;
  private SupplyCurrentLimitConfiguration drivetrainCurrentLimit;

  public AHRS navX;

  public double rightMotorVelocity;
  public double leftMotorVelocity;

  public double rightMotorPos;
  public double leftMotorPos;

  public double rightMotorIntegral;
  public double leftMotorIntegral;

  public double compHeading;
  public double yaw;

  public double previousRight;
  public double previousLeft;
  public double robotX;
  public double robotY;

  public double targetX;
  public double targetY;
  public double previousHeading;

  public double robotNavData [];

  public DriveTrainNavigationSub() {

    drivetrainCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.DRIVETRAIN_CURRENT_LIMIT, 0, 0.1);

    drivetrainHeadingPID = new ConfigurablePID(
      Constants.DRIVETRAIN_HEADING_PROPORTIONAL_GAIN,
      Constants.DRIVETRAIN_HEADING_INTEGRAL_GAIN,
      Constants.DRIVETRAIN_HEADING_DERIVITIVE_GAIN,
      Constants.DRIVETRAIN_HEADING_MAX_PROPORTIONAL,
      Constants.DRIVETRAIN_HEADING_MAX_INTEGRAL,
      Constants.DRIVETRAIN_HEADING_MAX_DERIVITIVE
    );

    drivetrainSpeedPID = new ConfigurablePID(
      Constants.DRIVETRAIN_SPEED_PROPORTIONAL_GAIN,
      Constants.DRIVETRAIN_SPEED_INTEGRAL_GAIN,
      Constants.DRIVETRAIN_SPEED_DERIVITIVE_GAIN,
      Constants.DRIVETRAIN_SPEED_MAX_PROPORTIONAL,
      Constants.DRIVETRAIN_SPEED_MAX_INTEGRAL,
      Constants.DRIVETRAIN_SPEED_MAX_DERIVITIVE
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

    rightMotorBack.follow(rightMotorFront);
    leftMotorBack.follow(leftMotorFront);

    robotNavData = new double [4];
  }

  public double[] driveTrainPosIntegration(double [] oldData)
  {
    oldData[0] = previousRight;
    oldData[1] = previousLeft;
    oldData[2] = robotX;
    oldData[3] = robotY;      

    rightMotorPos = rightMotorFront.getSelectedSensorPosition();
    leftMotorPos = leftMotorFront.getSelectedSensorPosition();

    rightMotorVelocity = rightMotorFront.getSelectedSensorVelocity();
    leftMotorVelocity = leftMotorFront.getSelectedSensorVelocity();

    compHeading = (double) navX.getCompassHeading();

    SmartDashboard.putNumber("Robot Heading:", compHeading);

    compHeading = Math.toRadians(compHeading);

    yaw = (double) navX.getYaw();

    double distanceTravLeft = leftMotorPos - previousLeft;
    double distanceTravRight = rightMotorPos - previousRight;
    double distanceTravel = (distanceTravLeft + distanceTravRight)/2;

    robotX = robotX + (Math.cos(compHeading) * distanceTravel);
    robotY = robotY + (Math.sin(compHeading) * distanceTravel);

    SmartDashboard.putNumber("Robot X:", robotX);
    SmartDashboard.putNumber("Robot Y:", robotY);

    robotNavData[0] = rightMotorPos;
    robotNavData[1] = leftMotorPos;
    robotNavData[2] = robotX;
    robotNavData[3] = robotY;

    return robotNavData;
  }

  public double setDriveToWaypoint(double [] newData)
  {
    newData[0] = targetX;
    newData[1] = targetY;
    newData[2] = robotX;
    newData[3] = robotY; 
    newData[4] = previousHeading;  

    double targetHeading = Math.atan2(targetY - robotY, targetX - robotX);

    compHeading = (double) navX.getCompassHeading();
    compHeading = Math.toRadians(compHeading);

    double headingRate = compHeading - previousHeading;
    double headingError = targetHeading - compHeading;

    double steeringPower = drivetrainHeadingPID.runPID(headingError, headingRate);

    SmartDashboard.putNumber("Heading Error:", headingError);

    double leftDrivePower = -steeringPower;
    double rightDrivePower = steeringPower;

    if(headingError < Constants.MAX_DRIVE_HEADING_ERROR)
    {
      double distanceError = Math.sqrt(Math.pow((targetY - robotY), 2)) + (Math.pow((targetX - robotX), 2));
      double speedPower = drivetrainSpeedPID.runPID(0, distanceError);

      SmartDashboard.putNumber("Distance to Waypoint:", distanceError);

      leftDrivePower = leftDrivePower + speedPower;
      rightDrivePower = rightDrivePower + speedPower;
    }

    leftMotorFront.set(ControlMode.PercentOutput, leftDrivePower);
    leftMotorBack.set(ControlMode.PercentOutput, leftDrivePower);
    rightMotorFront.set(ControlMode.PercentOutput, rightDrivePower);
    rightMotorBack.set(ControlMode.PercentOutput, rightDrivePower);

    return compHeading;
  }

  public boolean hasReachedWaypoint(double dist)
  {
    if(dist > dist - Constants.MAX_WAYPOINT_ERROR &&
    dist < dist + Constants.MAX_WAYPOINT_ERROR)
    {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {}
}
