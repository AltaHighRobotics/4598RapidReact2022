package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConfigurablePID;
import frc.robot.Constants;

public class ElevationAngleSub extends SubsystemBase{
  private TalonSRX elevationAngleMotor;
  private ConfigurablePID elevationAnglePID;
  
  public ElevationAngleSub() {

    elevationAnglePID = new ConfigurablePID(
      Constants.ELEVATION_ANGLE_PROPORTIONAL_GAIN,
      Constants.ELEVATION_ANGLE_INTEGRAL_GAIN,
      Constants.ELEVATION_ANGLE_DERIVITIVE_GAIN,
      Constants.ELEVATION_ANGLE_MAX_PROPORTIONAL,
      Constants.ELEVATION_ANGLE_MAX_INTEGRAL,
      Constants.ELEVATION_ANGLE_MAX_DERIVITIVE,
      -1,
      1,
      1
    );

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
  public void MoveElevationMotorToAngle(double targetElevationAngle) {

    SmartDashboard.putNumber("Target Elevation Angle:", targetElevationAngle);

    targetElevationAngle = MathUtil.clamp(targetElevationAngle, Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT, Constants.SHOOTER_ELEVATION_ANGLE_UPPER_LIMIT);
    double currentElevationAngle = (elevationAngleMotor.getSelectedSensorPosition()*Constants.ELEVATION_ANGLE_GEAR_RATIO)/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT;

    //SmartDashboard.putNumber("Raw Encoder Angle Degrees",(elevationAngleMotor.getSelectedSensorPosition())/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT);
    SmartDashboard.putNumber("Current Elevation Angle:", currentElevationAngle);

    double elevationMotorPower = elevationAnglePID.runPID(targetElevationAngle, currentElevationAngle);

    SmartDashboard.putNumber("Elevation Angle Motor Power:", elevationMotorPower);

    elevationAngleMotor.set(ControlMode.PercentOutput, elevationMotorPower);
  }
}
