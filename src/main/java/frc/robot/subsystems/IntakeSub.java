// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */
  private Solenoid leftIntakeSolenoid;
  private Solenoid rightIntakeSolenoid;
  private TalonFX intakeMotor;

  public IntakeSub() {
    leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.LEFT_INTAKE_SOLENOID);
    rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.RIGHT_INTAKE_SOLENOID);
    intakeMotor = new TalonFX(Constants.LEFT_INTAKE_MOTOR);
  }

  public void IntakeExtend(){
    leftIntakeSolenoid.set(true);
    rightIntakeSolenoid.set(true);
  }

  public void IntakeRetract(){
    leftIntakeSolenoid.set(false);
    rightIntakeSolenoid.set(false);
  }

  public void IntakeOn(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }

  public void IntakeOff(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
