// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.*;

import org.opencv.core.Mat;

import java.lang.Math;

public class ClimbingSub extends SubsystemBase {
  /** Creates a new ClimbingPistonSub. */
  private Solenoid leftSwingSolenoid;
  private Solenoid rightSwingSolenoid;
  private TalonFX leftLiftMotor;
  private TalonFX rightLiftMotor;
  private double rightLiftMotorVelocity;
  private double leftLiftMotorVelocity;
  private double rightLiftMotorPosition;
  private double leftLiftMotorPosition;

  public ClimbingSub() {
      // leftSwingSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.LEFT_SWING_SOLENOID);
      // rightSwingSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.RIGHT_SWING_SOLENOID);
      leftLiftMotor = new TalonFX(Constants.LEFT_LIFT_MOTOR);
      rightLiftMotor = new TalonFX(Constants.RIGHT_LIFT_MOTOR);
      leftLiftMotor.setNeutralMode(NeutralMode.Brake);
      rightLiftMotor.setNeutralMode(NeutralMode.Brake);
      leftLiftMotor.configFactoryDefault();
      rightLiftMotor.configFactoryDefault();
      leftLiftMotor.setInverted(false);
      rightLiftMotor.setInverted(false);
      leftLiftMotor.setSensorPhase(false);
      rightLiftMotor.setSensorPhase(false);
    }

  public void ExtendArms(){
    leftLiftMotor.set(ControlMode.PercentOutput, -Constants.LIFT_ARM_SPEED);
    rightLiftMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
  }

  public void RetractArms(){
    leftLiftMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    rightLiftMotor.set(ControlMode.PercentOutput, -Constants.LIFT_ARM_SPEED);
  }

  public void ArmsStationary(){
    leftLiftMotor.set(ControlMode.PercentOutput, 0);
    rightLiftMotor.set(ControlMode.PercentOutput, 0);
  }
  
  public void SwingArms(){
    rightSwingSolenoid.set(true);
    leftSwingSolenoid.set(true);
  }

  public void ReturnArms(){
    rightSwingSolenoid.set(false);
    leftSwingSolenoid.set(false);
  }

  public boolean SetArmsWithClamp(double targetPosition)
  {
    rightLiftMotorPosition = rightLiftMotor.getSelectedSensorPosition();
    leftLiftMotorPosition = leftLiftMotor.getSelectedSensorPosition();
    rightLiftMotorVelocity = rightLiftMotor.getSelectedSensorVelocity();
    leftLiftMotorVelocity = leftLiftMotor.getSelectedSensorVelocity();
    
    double averageArmPosition = (leftLiftMotorPosition + rightLiftMotorPosition)/2;
    double armMin = Math.min(targetPosition, averageArmPosition + Constants.MAX_ARM_ERROR);
    double actualTarget = Math.max(armMin, averageArmPosition - Constants.MAX_ARM_ERROR);

    double leftArmError = -actualTarget + leftLiftMotorPosition;
    double rightArmError = -actualTarget + rightLiftMotorPosition;

    double leftArmVelocityError = leftArmError - leftLiftMotorVelocity;
    double rightArmVelocityError = rightArmError - rightLiftMotorVelocity;

    double leftArmPower = leftArmVelocityError * Constants.ARM_PROPORTIONAL_GAIN;
    double rightArmPower = rightArmVelocityError * Constants.ARM_PROPORTIONAL_GAIN;

    // leftLiftMotor.set(ControlMode.PercentOutput, leftArmPower);
    // rightLiftMotor.set(ControlMode.PercentOutput, rightArmPower);

    System.out.println("LV: " + leftArmVelocityError);
    System.out.println("RV: " + rightArmVelocityError);

    if (leftArmError + rightArmError < Constants.MAX_ARM_ERROR)
    {
      return true;
    }
    return false;
  }

  public double getLeftCoderAngle()
  {
    return leftLiftMotor.getSelectedSensorPosition();
  }

  public double getRightCoderAngle()
  {
    return rightLiftMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    
  } 
}
