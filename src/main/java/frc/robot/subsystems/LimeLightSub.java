// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelightvision.limelight.frc.LimeLight;
import limelightvision.limelight.frc.ControlMode.LedMode;

public class LimeLightSub extends SubsystemBase {
  /** Creates a new LimeLightSub. */

  public final LimeLight limeLight;
  public double RPM;
  private final int targetPipeline = 0;

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

public double getLimeLightYaw(){
  return limeLight.getdegRotationToTarget();
}

public double getLimeLightElevation(){
  return limeLight.getdegVerticalToTarget();
}

public void enableLimeLight() {
  limeLight.setPipeline(targetPipeline);
  limeLight.setLEDMode(LedMode.kforceOn);
}

public void disableLimeLight() {
  limeLight.setPipeline(targetPipeline);
  limeLight.setLEDMode(LedMode.kforceOff);
}

}
