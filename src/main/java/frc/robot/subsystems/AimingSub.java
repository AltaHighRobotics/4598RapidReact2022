package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.ConfigurablePID;
import edu.wpi.first.wpilibj.SPI;
public class AimingSub extends SubsystemBase {

  private final WPI_TalonFX azimuthMotor;
  private final ConfigurablePID azimuthPID;
  private final SupplyCurrentLimitConfiguration azimuthCurrentLimit;
  private final AHRS navX;
  private double absoluteAzimuthToTarget;
  private double azimuthEncoderPosition;
  private double azimuthEncoderVelocity;
  private double absoluteNavYaw;
  private double relativeLimeLightYaw;
  private double azimuthMotorPower;
  private double azimuthTargetAngle;
  private double elevationTargetAngle;
  private double elevationEncoderPosition;
  private double elevationMotorPower;
  private final WPI_TalonSRX elevationAngleMotor;
  private final ConfigurablePID elevationAnglePID;
  private final WPI_TalonFX leftShooterMotor;
  private final WPI_TalonFX rightShooterMotor;
  private final ConfigurablePID leftShooterPID;
  private final ConfigurablePID rightShooterPID;
  private final SupplyCurrentLimitConfiguration shooterCurrentLimit;

  public AimingSub() {

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

    elevationAngleMotor = new WPI_TalonSRX(Constants.ELEVATION_ANGLE_MOTOR);

    elevationAngleMotor.configFactoryDefault();

    elevationAngleMotor.setNeutralMode(NeutralMode.Brake);

    elevationAngleMotor.setSensorPhase(false);

    elevationAngleMotor.setInverted(true);

    shooterCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT, 0, 0.1);

    leftShooterPID = new ConfigurablePID(
      Constants.SHOOTER_PORPORTIONAL_GAIN,
      Constants.SHOOTER_INTERGRAL_GAIN,
      Constants.SHOOTER_DERIVITIVE_GAIN,
      Constants.SHOOTER_MAX_PROPORTIONAL,
      Constants.SHOOTER_MAX_INTEGRAL,
      Constants.SHOOTER_MAX_DERIVITIVE,
      Constants.SHOOTER_POWER_OFFSET,
      1,
      1
    );

    rightShooterPID = new ConfigurablePID(
      Constants.SHOOTER_PORPORTIONAL_GAIN,
      Constants.SHOOTER_INTERGRAL_GAIN,
      Constants.SHOOTER_DERIVITIVE_GAIN,
      Constants.SHOOTER_MAX_PROPORTIONAL,
      Constants.SHOOTER_MAX_INTEGRAL,
      Constants.SHOOTER_MAX_DERIVITIVE,
      Constants.SHOOTER_POWER_OFFSET,
      1,
      1
    );

    leftShooterMotor = new WPI_TalonFX(Constants.LEFT_SHOOTER_MOTOR);
    rightShooterMotor = new WPI_TalonFX(Constants.RIGHT_SHOOTER_MOTOR);

    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();

    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);

    leftShooterMotor.setSensorPhase(false);
    rightShooterMotor.setSensorPhase(false);

    leftShooterMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightShooterMotor.setInverted(TalonFXInvertType.Clockwise);

    leftShooterMotor.configOpenloopRamp(Constants.SHOOTER_POWER_RAMP_TIME, 0);
    rightShooterMotor.configOpenloopRamp(Constants.SHOOTER_POWER_RAMP_TIME, 0);

    leftShooterMotor.configSupplyCurrentLimit(shooterCurrentLimit);
    rightShooterMotor.configSupplyCurrentLimit(shooterCurrentLimit);
  }

  /** PID Controller used to set the azimuth angle for the shooter
   *  Adjusts the motor power to aim with the limeLight.
   *  @param limeLightYaw the rotation angle to the target, as measured by the limelight
   */
  public void moveAzimuthMotorToLimeLight(double limeLightYaw) {
    azimuthEncoderPosition = (azimuthMotor.getSelectedSensorPosition() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
    azimuthEncoderVelocity = (azimuthMotor.getSelectedSensorVelocity() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
    absoluteNavYaw = navX.getYaw();
    relativeLimeLightYaw = limeLightYaw;
    if(relativeLimeLightYaw != 0) {
      absoluteAzimuthToTarget = relativeLimeLightYaw + azimuthEncoderPosition + absoluteNavYaw;
    }
    azimuthTargetAngle = MathUtil.clamp((absoluteAzimuthToTarget-absoluteNavYaw)%360, Constants.AZIMUTH_LOWER_LIMIT, Constants.AZIMUTH_UPPER_LIMIT);
    
    azimuthMotorPower = azimuthPID.runVelocityPID(azimuthTargetAngle, azimuthEncoderPosition, azimuthEncoderVelocity);
    SmartDashboard.putNumber("Target Azimuth", azimuthTargetAngle);
    SmartDashboard.putNumber("Azimuth Angle", azimuthEncoderPosition);
    SmartDashboard.putNumber("Azimuth Power", azimuthMotorPower);
    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }

  /** PI Controller used to set the elevation angle for the shooter
   *  Adjusts the motor power to aim with the limeLight.
   *  @param limeLightElevation the vertical angle to the target, as measured by the limelight
   */
  public void moveElevationMotorToLimeLight(double limeLightElevation) {
    if(limeLightElevation != 0) {
      elevationTargetAngle = limeLightElevation + Constants.LIMELIGHT_ELEVATION_ANGLE;
    }
    else {
      elevationTargetAngle = 0;
    }
    SmartDashboard.putNumber("Target Elevation Angle:", elevationTargetAngle);

    elevationTargetAngle = MathUtil.clamp(elevationTargetAngle, Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT, Constants.SHOOTER_ELEVATION_ANGLE_UPPER_LIMIT);
    elevationEncoderPosition = (elevationAngleMotor.getSelectedSensorPosition()*Constants.ELEVATION_ANGLE_GEAR_RATIO)/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT;

    //SmartDashboard.putNumber("Raw Encoder Angle Degrees",(elevationAngleMotor.getSelectedSensorPosition())/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT);
    SmartDashboard.putNumber("Current Elevation Angle:", elevationEncoderPosition);

    elevationMotorPower = elevationAnglePID.runPID(elevationTargetAngle, elevationEncoderPosition);

    SmartDashboard.putNumber("Elevation Angle Motor Power:", elevationMotorPower);

    elevationAngleMotor.set(ControlMode.PercentOutput, elevationMotorPower);
  }

  /** PID Controller used to set the azimuth angle for the shooter
   *  Adjusts the motor power to go to a target angle, in degrees.
   *  @param targetAngle A double representing the target angle that the azimuth angle motor should attempt to reach
   */
  public void moveAzimuthMotorToAngle(double targetAngle) {
    targetAngle = MathUtil.clamp(azimuthTargetAngle, Constants.AZIMUTH_LOWER_LIMIT, Constants.AZIMUTH_UPPER_LIMIT);
    azimuthEncoderPosition = (azimuthMotor.getSelectedSensorPosition() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
    azimuthEncoderVelocity = (azimuthMotor.getSelectedSensorVelocity() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;

    azimuthMotorPower = azimuthPID.runVelocityPID(azimuthTargetAngle, azimuthEncoderPosition, azimuthEncoderVelocity);

    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }

  /** PI Controller used to set the elevation angle for the shooter
   *  Adjusts the motor power to go to a target angle, in degrees.
   *  @param targetElevationAngle A double representing the target angle that the elevation angle motor should attempt to reach
   */
  public void moveElevationMotorToAngle(double targetElevationAngle) {

    SmartDashboard.putNumber("Target Elevation Angle:", targetElevationAngle);

    targetElevationAngle = MathUtil.clamp(targetElevationAngle, Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT, Constants.SHOOTER_ELEVATION_ANGLE_UPPER_LIMIT);
    double currentElevationAngle = (elevationAngleMotor.getSelectedSensorPosition()*Constants.ELEVATION_ANGLE_GEAR_RATIO)/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT;

    //SmartDashboard.putNumber("Raw Encoder Angle Degrees",(elevationAngleMotor.getSelectedSensorPosition())/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT);
    SmartDashboard.putNumber("Current Elevation Angle:", currentElevationAngle);

    double elevationMotorPower = elevationAnglePID.runPID(targetElevationAngle, currentElevationAngle);

    SmartDashboard.putNumber("Elevation Angle Motor Power:", elevationMotorPower);

    elevationAngleMotor.set(ControlMode.PercentOutput, elevationMotorPower);
  }

  public void stopAimingMotors() {
    elevationAngleMotor.neutralOutput();
    azimuthMotor.neutralOutput();
  }

  public boolean getIsAimReady() {
    return relativeLimeLightYaw != 0 && Math.abs(relativeLimeLightYaw) < Constants.AZIMUTH_MAX_ERROR;
  }

  public double getNavYaw() {
    return navX.getYaw();
  }

  public double getAzimuth() {
    return (azimuthMotor.getSelectedSensorPosition() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
  }

  /** Proportional Integral Controller used to set both of the shooter motors speed
   *  Adjusts both motors power to match a target velocity
   *  @param targetShooterVelocity A double representing the target velocity that the shooter motors should attempt to reach
   */
  public void setShooterMotorsVelocity(double targetShooterVelocity){

    // Gets the velocity of each of the two shooter motors
    double shooterLeftVelocity = leftShooterMotor.getSelectedSensorVelocity();
    double shooterRightVelocity = rightShooterMotor.getSelectedSensorVelocity();

    // Runs the controllers
    double shooterLeftPower = leftShooterPID.runPID(targetShooterVelocity, shooterLeftVelocity);
    double shooterRightPower = rightShooterPID.runPID(targetShooterVelocity, shooterRightVelocity);

    // Sets the motors to the computed power levels
    leftShooterMotor.set(ControlMode.PercentOutput, shooterLeftPower);
    rightShooterMotor.set(ControlMode.PercentOutput, shooterRightPower);

    // Displays useful values in Smart Dashboard
    SmartDashboard.putNumber("Shooter Left Power:", shooterLeftPower);
    SmartDashboard.putNumber("Shooter Right Power:", shooterRightPower);
    SmartDashboard.putString("Shooter Status:", "Shooting");
    SmartDashboard.putNumber("Target Speed", targetShooterVelocity);
  }

  public void setShooterMotorsPower(double Speed){
    leftShooterMotor.set(ControlMode.PercentOutput, Speed);
    rightShooterMotor.set(ControlMode.PercentOutput, Speed);
  }

  public void stopShooterMotors(){
    leftShooterMotor.set(ControlMode.PercentOutput, 0);
    rightShooterMotor.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putString("Shooter Status:", "Stopped");
  }
}
