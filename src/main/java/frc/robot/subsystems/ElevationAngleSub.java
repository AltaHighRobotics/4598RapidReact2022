package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevationAngleSub extends SubsystemBase{
  private TalonSRX elevationAngleMotor;
  
  public ElevationAngleSub() {
    elevationAngleMotor = new TalonSRX(Constants.ELEVATION_ANGLE_MOTOR);

    elevationAngleMotor.configFactoryDefault();

    elevationAngleMotor.setNeutralMode(NeutralMode.Brake);

    elevationAngleMotor.setSensorPhase(false);

    elevationAngleMotor.setInverted(true);
  }
    /** Proportional Controller used to set the elevation angle for the shooter
   *  Adjusts the motor power to go to a target angle, in degrees.
   *  @param targetElevationAngle A double representing the target angle that the elevation angle motor should attempt to reach
   */
  public void MoveElevationMotorToAngle(double targetElevationAngle){
    SmartDashboard.putNumber("Target Elevation Angle:", targetElevationAngle);
    targetElevationAngle = Math.min(Math.max(targetElevationAngle, Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT),Constants.SHOOTER_ELEVATION_ANGLE_UPPER_LIMIT);
    // Converts the target from degrees to encoder units

    // Gets the current rotation of the elevation motor, relative to the rotation upon robot power-up (1 rotation = 4096 units)
    double currentElevationAngle = (elevationAngleMotor.getSelectedSensorPosition()*Constants.ELEVATION_ANGLE_GEAR_RATIO)/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT;
    SmartDashboard.putNumber("Raw Encoder Angle Degrees",(elevationAngleMotor.getSelectedSensorPosition())/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT);
    SmartDashboard.putNumber("Current Elevation Angle", currentElevationAngle);
    // Gets the difference between the target angle and the current angle
    double elevationAngleError = targetElevationAngle - currentElevationAngle;

    // Sets the motor power to the error in elevation * a constant gain
    double elevationMotorPower = elevationAngleError * Constants.ELEVATION_ANGLE_PROPORTIONAL_GAIN;
    SmartDashboard.putNumber("Elevation Angle Motor Power", elevationMotorPower);
    elevationAngleMotor.set(ControlMode.PercentOutput, elevationMotorPower);
  }
}
