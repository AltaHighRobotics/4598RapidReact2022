// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StorageSub extends SubsystemBase {
  /** Creates a new StorageSub. */

  private TalonSRX elevationAngleMotor;
  private VictorSPX storageMotor;
  private VictorSPX feedMotor;
  private VictorSPX azimuthMotor;

  public StorageSub() {
    elevationAngleMotor = new TalonSRX(Constants.ELEVATION_ANGLE_MOTOR);
    elevationAngleMotor.configFactoryDefault();

    storageMotor = new VictorSPX(Constants.STORAGE_MOTOR);
    storageMotor.configFactoryDefault();

    feedMotor = new VictorSPX(Constants.FEED_MOTOR);
    feedMotor.configFactoryDefault();
    feedMotor.setInverted(true);

    azimuthMotor = new VictorSPX(Constants.AZIMUTH_MOTOR);
    azimuthMotor.configFactoryDefault();
  }

  public void feedOn(){
    feedMotor.set(ControlMode.PercentOutput, Constants.FEED_POWER);
  }

  public void feedOff(){
    feedMotor.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
