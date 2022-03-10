// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainPIDSub extends SubsystemBase {
  /** Creates a new DriveTrainPIDSub. */
  public TalonFX rightMotorFront;
  public TalonFX rightMotorBack;
  public TalonFX leftMotorFront;
  public TalonFX leftMotorBack;

  public double rightMotorVelocity;
  public double leftMotorVelocity;

  public double rightMotorPos;
  public double leftMotorPos;

  public double rightMotorIntegral;
  public double leftMotorIntegral;

  public DriveTrainPIDSub() {
    rightMotorFront = new TalonFX(Constants.RIGHT_MOTOR_FRONT);
    rightMotorBack = new TalonFX(Constants.RIGHT_MOTOR_BACK);
    leftMotorFront = new TalonFX(Constants.LEFT_MOTOR_FRONT);
    leftMotorBack = new TalonFX(Constants.LEFT_MOTOR_BACK);

    leftMotorFront.setNeutralMode(NeutralMode.Brake);
    rightMotorFront.setNeutralMode(NeutralMode.Brake);
    leftMotorBack.setNeutralMode(NeutralMode.Brake);
    rightMotorBack.setNeutralMode(NeutralMode.Brake);

    leftMotorFront.configFactoryDefault();
    rightMotorFront.configFactoryDefault();
    leftMotorBack.configFactoryDefault();
    rightMotorBack.configFactoryDefault();

    leftMotorFront.setInverted(false);
    rightMotorFront.setInverted(false);
    leftMotorBack.setInverted(false);
    rightMotorBack.setInverted(false);

    leftMotorFront.setSensorPhase(false);
    rightMotorFront.setSensorPhase(false);
    leftMotorBack.setSensorPhase(false);
    rightMotorBack.setSensorPhase(false);
  }

  public void DriveTrainPID(double targetPosition)
  {
    rightMotorPos = rightMotorFront.getSelectedSensorPosition();
    leftMotorPos = leftMotorFront.getSelectedSensorPosition();

    rightMotorVelocity = rightMotorFront.getSelectedSensorVelocity();
    leftMotorVelocity = leftMotorFront.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {}
}
