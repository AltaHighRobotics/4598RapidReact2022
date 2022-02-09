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

public class ClimbingSub extends SubsystemBase {
  /** Creates a new ClimbingPistonSub. */
  private Solenoid leftSwingSolenoid;
  private Solenoid rightSwingSolenoid;
  private TalonFX leftLiftMotor;
  private TalonFX rightLiftMotor;

  public ClimbingSub() {
      leftSwingSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.LEFT_SWING_SOLENOID);
      rightSwingSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.RIGHT_SWING_SOLENOID);
      leftLiftMotor = new TalonFX(Constants.LEFT_LIFT_MOTOR);
      rightLiftMotor = new TalonFX(Constants.RIGHT_LIFT_MOTOR);
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

  @Override
  public void periodic() {
    
  } 
}
