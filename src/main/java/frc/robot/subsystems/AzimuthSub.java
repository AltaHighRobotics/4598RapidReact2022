package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AzimuthSub extends SubsystemBase {

  private TalonFX azimuthMotor;

  public AzimuthSub() {
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
    double azimuthVelocityError = azimuthPositionError - azimuthEncoderVelocity;

    double azimuthMotorPower = azimuthVelocityError * Constants.AZIMUTH_PROPORTIONAL_GAIN;
    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }
    
}
