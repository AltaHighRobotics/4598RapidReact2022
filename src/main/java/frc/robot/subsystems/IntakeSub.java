// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */
  private final Solenoid intakeSolenoid;
  private final WPI_VictorSPX intakeMotor;

  public IntakeSub() {
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID);
    intakeMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  public void IntakeExtend(){
    intakeSolenoid.set(true);
    //SmartDashboard.putString("Intake Piston Status:", "Deployed");
  }

  public void IntakeRetract(){
    intakeSolenoid.set(false);
    //SmartDashboard.putString("Intake Piston Status:", "Retracted");
  }

  public void IntakeOn(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
    //SmartDashboard.putString("Intake Motor Status:", "Running");
  }

  public void IntakeReverse(){
    intakeMotor.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
    //SmartDashboard.putString("Intake Motor Status:", "Reverse");
  }

  public void IntakeOff(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
    //SmartDashboard.putString("Intake Motor Status:", "Stopped");
  }
}
