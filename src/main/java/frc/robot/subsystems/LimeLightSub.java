// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelightvision.limelight.frc.LimeLight;

public class LimeLightSub extends SubsystemBase {
  /** Creates a new LimeLightSub. */

  public LimeLight limeLight;
  public double RPM;

  public LimeLightSub() {

    limeLight = new LimeLight();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public LimeLight getLimeLight(){
  return limeLight;
}

}
