package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.ConfigurablePID;

public class AzimuthSub extends SubsystemBase {

  private final WPI_TalonFX azimuthMotor;
  private final ConfigurablePID azimuthPID;
  private final SupplyCurrentLimitConfiguration azimuthCurrentLimit;

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

    azimuthMotor = new WPI_TalonFX(Constants.AZIMUTH_MOTOR);
    azimuthMotor.configFactoryDefault();
    azimuthMotor.setNeutralMode(NeutralMode.Brake);
    azimuthMotor.setSensorPhase(false);
    azimuthMotor.setInverted(TalonFXInvertType.CounterClockwise);
    azimuthMotor.configOpenloopRamp(Constants.AZIMUTH_POWER_RAMP_TIME, 0);
    azimuthMotor.configSupplyCurrentLimit(azimuthCurrentLimit);
  }

  public void moveAzimuthMotorToAngle(double azimuthTargetAngle) {
    azimuthTargetAngle = MathUtil.clamp(azimuthTargetAngle, Constants.AZIMUTH_LOWER_LIMIT, Constants.AZIMUTH_UPPER_LIMIT);
    double azimuthEncoderPosition = azimuthMotor.getSelectedSensorPosition() / 4096 * 360;
    double azimuthEncoderVelocity = azimuthMotor.getSelectedSensorVelocity();

    double azimuthMotorPower = azimuthPID.runVelocityPID(azimuthTargetAngle, azimuthEncoderPosition, azimuthEncoderVelocity);
    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }
    
}
