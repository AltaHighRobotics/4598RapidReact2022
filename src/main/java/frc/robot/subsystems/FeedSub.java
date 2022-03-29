// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeedSub extends SubsystemBase {
  /** Creates a new StorageSub. */

  private final WPI_VictorSPX feedMotor;

  public FeedSub() {
    feedMotor = new WPI_VictorSPX(Constants.FEED_MOTOR);
    feedMotor.configFactoryDefault();
    feedMotor.setInverted(true);

  }

  public void feedOn(){
    feedMotor.set(ControlMode.PercentOutput, Constants.FEED_POWER);
    SmartDashboard.putString("Feeder Status:", "Feeding");
  }

  public void feedOff(){
    feedMotor.set(ControlMode.PercentOutput, -0.1);
    SmartDashboard.putString("Feeder Status:", "Stopped");
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
