package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.ConfigurablePID;
import edu.wpi.first.wpilibj.SPI;

public class AzimuthSub extends SubsystemBase {

  private final WPI_TalonFX azimuthMotor;
  private final ConfigurablePID azimuthPID;
  private final SupplyCurrentLimitConfiguration azimuthCurrentLimit;
  private final AHRS navX;

  public AzimuthSub() {

    azimuthCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.AZIMUTH_CURRENT_LIMIT, 0, 0.1);

    azimuthPID = new ConfigurablePID(
      Constants.AZIMUTH_PROPORTIONAL_GAIN,
      Constants.AZIMUTH_INTEGRAL_GAIN,
      Constants.AZIMUTH_DERIVITIVE_GAIN,
      Constants.AZIMUTH_MAX_PROPORTIONAL,
      Constants.AZIMUTH_MAX_INTEGRAL,
      Constants.AZIMUTH_MAX_DERIVITIVE,
      -Constants.AZIMUTH_MAX_POWER,
      Constants.AZIMUTH_MAX_POWER,
      Constants.AZIMUTH_SPEED
    );

    navX = new AHRS(SPI.Port.kMXP);

    azimuthMotor = new WPI_TalonFX(Constants.AZIMUTH_MOTOR);
    azimuthMotor.configFactoryDefault();
    azimuthMotor.setNeutralMode(NeutralMode.Brake);
    azimuthMotor.setSensorPhase(true);
    azimuthMotor.setInverted(TalonFXInvertType.CounterClockwise);
    azimuthMotor.configOpenloopRamp(Constants.AZIMUTH_POWER_RAMP_TIME, 0);
    azimuthMotor.configSupplyCurrentLimit(azimuthCurrentLimit);
  }

  public void moveAzimuthMotorToAngle(double azimuthTargetAngle) {
    azimuthTargetAngle = MathUtil.clamp(azimuthTargetAngle-navX.getYaw(), Constants.AZIMUTH_LOWER_LIMIT, Constants.AZIMUTH_UPPER_LIMIT);
    double azimuthEncoderPosition = (azimuthMotor.getSelectedSensorPosition() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
    double azimuthEncoderVelocity = (azimuthMotor.getSelectedSensorVelocity() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;

    double azimuthMotorPower = azimuthPID.runVelocityPID(azimuthTargetAngle, azimuthEncoderPosition, azimuthEncoderVelocity);
    SmartDashboard.putNumber("Target Azimuth", azimuthTargetAngle);
    SmartDashboard.putNumber("Azimuth Angle", azimuthEncoderPosition);
    SmartDashboard.putNumber("Azimuth Power", azimuthMotorPower);
    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }

  public double getNavYaw() {
    return navX.getYaw();
  }

  public double getAzimuth() {
    return (azimuthMotor.getSelectedSensorPosition() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
  }

}
