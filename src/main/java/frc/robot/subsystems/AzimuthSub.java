package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AzimuthSub extends SubsystemBase {

  private TalonFX azimuthMotor;
  private ConfigurablePID azimuthPID;

  public AzimuthSub() {
    azimuthPID = new ConfigurablePID(
      Constants.AZIMUTH_PROPORTIONAL_GAIN,
      Constants.AZIMUTH_INTEGRAL_GAIN,
      Constants.AZIMUTH_DERIVITIVE_GAIN,
      Constants.AZIMUTH_MAX_PROPORTIONAL,
      Constants.AZIMUTH_MAX_INTEGRAL,
      Constants.AZIMUTH_MAX_DERIVITIVE
    );
    
    azimuthMotor = new TalonFX(Constants.AZIMUTH_MOTOR);
    azimuthMotor.configFactoryDefault();
    azimuthMotor.setNeutralMode(NeutralMode.Brake);
    azimuthMotor.setSensorPhase(false);
    azimuthMotor.setInverted(TalonFXInvertType.CounterClockwise);
  }

  public void moveAzimuthMotorToAngle(double azimuthTargetAngle) {
    azimuthTargetAngle = Math.min(Math.max(azimuthTargetAngle,Constants.AZIMUTH_LOWER_LIMIT),Constants.AZIMUTH_UPPER_LIMIT);
    double azimuthEncoderPosition = azimuthMotor.getSelectedSensorPosition() / 4096 * 360;
    double azimuthEncoderVelocity = azimuthMotor.getSelectedSensorVelocity();

    double azimuthPositionError = azimuthTargetAngle - azimuthEncoderPosition;

    double azimuthMotorPower = azimuthPID.runPID(azimuthPositionError, azimuthEncoderVelocity);
    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }
    
}
