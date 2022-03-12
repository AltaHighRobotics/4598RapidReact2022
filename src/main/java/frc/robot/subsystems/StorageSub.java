// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StorageSub extends SubsystemBase {
  /** Creates a new StorageSub. */

  private Victor windowMotor;
  private Victor storageMotor;
  private VictorSPX elevatorMotor;
  private Victor turrentMotor;

  public StorageSub() {
    windowMotor = new Victor(Constants.WINDOW_MOTOR);
    storageMotor = new Victor(Constants.STORAGE_MOTOR);
    elevatorMotor = new VictorSPX(Constants.ELEVATOR_MOTOR);
    turrentMotor = new Victor(Constants.TURRENT_MOTOR);

    elevatorMotor.setInverted(true);
  }

  public void elevatorOn(){
    elevatorMotor.set(ControlMode.PercentOutput, Constants.ELEVATOR_POWER);
    System.out.println(Constants.ELEVATOR_POWER);
  }

  public void elevatorOff(){
    elevatorMotor.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
