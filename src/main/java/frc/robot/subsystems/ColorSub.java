// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class ColorSub extends SubsystemBase {
  /** Creates a new ColorSub. */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private Color detectedColor;
  public ColorSub() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getColor() {
    detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("RED:", detectedColor.red);
    SmartDashboard.putNumber("GREEN:", detectedColor.green);
    SmartDashboard.putNumber("BLUE:", detectedColor.blue);
  }

  public boolean matchColorToAlliance(String alliance) {
    detectedColor = m_colorSensor.getColor();
    if(alliance == "Red Alliance") {
      if(matchColorToColor(Constants.RED_ALLIANCE_COLOR, detectedColor)) {
        SmartDashboard.putString("Color Match:", "Color Matches Red Alliance");
        return true;
      } else {
        SmartDashboard.putString("Color Match:", "Color Does Not Match Red Alliance");
        return false;
      }
    } else if(alliance == "Blue Alliance") {
      if(matchColorToColor(Constants.BLUE_ALLIANCE_COLOR, detectedColor)) {
        SmartDashboard.putString("Color Match:", "Color Matches Blue Alliance");
        return true;
      } else {
        SmartDashboard.putString("Color Match:", "Color Does Not Match Blue Alliance");
        return false;
      }
    } else {
      System.out.println("matchColorToAlliance() was used without a valid alliance. This will always return false!");
      return false;
    }
  }

  public boolean matchColorToColor(Color colorA, Color colorB) {
    boolean rMatch = matchColorChannel(colorA.red, colorB.red);
    boolean bMatch = matchColorChannel(colorA.blue, colorB.blue);
    boolean gMatch = matchColorChannel(colorA.green, colorB.green);

    return rMatch && bMatch && gMatch;
  }

  private boolean matchColorChannel(double a, double b) {
    return (a < b + Constants.COLOR_MATCH_THRESHOLD && a > b - Constants.COLOR_MATCH_THRESHOLD);
  }
}