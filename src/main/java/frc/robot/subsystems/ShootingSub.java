package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.ConfigurablePID;
import limelightvision.limelight.frc.LimeLight;
import limelightvision.limelight.frc.ControlMode.LedMode;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
public class ShootingSub extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private Color detectedColor;

  SendableChooser<String> m_allianceChooser = new SendableChooser<>();
  private final LimeLight limeLight;
  private final int targetPipeline = 0;
  private final WPI_TalonFX azimuthMotor;
  private final ConfigurablePID azimuthPID;
  private final SupplyCurrentLimitConfiguration azimuthCurrentLimit;
  private final AHRS navX;
  private final WPI_TalonSRX elevationAngleMotor;
  private final WPI_VictorSPX feedMotor;
  private final ConfigurablePID elevationAnglePID;
  private final WPI_TalonFX leftShooterMotor;
  private final WPI_TalonFX rightShooterMotor;
  private final ConfigurablePID shooterPID;
  private final SupplyCurrentLimitConfiguration shooterCurrentLimit;
  private final SupplyCurrentLimitConfiguration elevationAngleCurrentLimit;
  private double absoluteAzimuthToTarget;
  private double azimuthEncoderPosition;
  private double azimuthEncoderVelocity;
  private double absoluteNavYaw;
  private double relativeLimeLightYaw;
  private double azimuthMotorPower;
  private double azimuthTargetAngle;
  private double clampedAzimuthTargetAngle;
  private double clampedElevationTargetAngle;
  private double elevationEncoderPosition;
  private double elevationMotorPower;
  private double angleToGoalDegrees;
  private double angleToGoalRadians;
  private double distanceToGoal;
  private double shooterVelocity;
  private double shooterPower;
  private boolean azimuthReady = false;
  private boolean elevationReady = false;
  private boolean shooterReady = false;
  private int shotProbability = 0;
  private int autoMissTimer = 0;
  private boolean shouldMiss = false;
  

  public ShootingSub() {

    m_allianceChooser.setDefaultOption("Red Alliance", "Red Alliance");
    m_allianceChooser.addOption("Blue Alliance", "Blue Alliance");

    // Displays the alliance selector on the dashboard
    SmartDashboard.putData(m_allianceChooser);

    limeLight = new LimeLight();

    feedMotor = new WPI_VictorSPX(Constants.FEED_MOTOR);
    feedMotor.configFactoryDefault();
    feedMotor.setInverted(true);

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
      -Constants.ELEVATION_ANGLE_MAX_POWER,
      Constants.ELEVATION_ANGLE_MAX_POWER,
      0.5
    );

    elevationAngleCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.ELEVATION_ANGLE_CURRENT_LIMIT, 0, 0.1);

    elevationAngleMotor = new WPI_TalonSRX(Constants.ELEVATION_ANGLE_MOTOR);

    elevationAngleMotor.configFactoryDefault();

    elevationAngleMotor.setNeutralMode(NeutralMode.Brake);

    elevationAngleMotor.setSensorPhase(false);

    elevationAngleMotor.setInverted(false);

    elevationAngleMotor.configSupplyCurrentLimit(elevationAngleCurrentLimit);

    elevationAngleMotor.configOpenloopRamp(Constants.ELEVATION_ANGLE_RAMP_TIME);

    shooterCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT, 0, 0.1);

    shooterPID = new ConfigurablePID(
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

    leftShooterMotor.setInverted(TalonFXInvertType.Clockwise);
    rightShooterMotor.setInverted(TalonFXInvertType.CounterClockwise);

    leftShooterMotor.configOpenloopRamp(Constants.SHOOTER_POWER_RAMP_TIME, 0);
    rightShooterMotor.configOpenloopRamp(Constants.SHOOTER_POWER_RAMP_TIME, 0);

    leftShooterMotor.configSupplyCurrentLimit(shooterCurrentLimit);
    rightShooterMotor.configSupplyCurrentLimit(shooterCurrentLimit);
  }

  public void getColor() {
    detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("RED:", detectedColor.red);
    SmartDashboard.putNumber("GREEN:", detectedColor.green);
    SmartDashboard.putNumber("BLUE:", detectedColor.blue);
  }

  public boolean ballDetected() {
    return (matchColorToAlliance("Red Alliance") || matchColorToAlliance("Blue Alliance"));
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

  public String getAlliance() {
    return m_allianceChooser.getSelected();
  }

  public LimeLight getLimeLight() {
    return limeLight;
  }
  
  public double getLimeLightYaw() {
    return limeLight.getdegRotationToTarget();
  }
  
  public double getLimeLightElevation() {
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

  /** PID Controller used to set the azimuth angle for the shooter
   *  Adjusts the motor power to aim with the limeLight.
   *  @param limeLightYaw the rotation angle to the target, as measured by the limelight
   */
  public void moveAzimuthMotorToLimeLight(double offset) {
    azimuthEncoderPosition = (azimuthMotor.getSelectedSensorPosition() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
    azimuthEncoderVelocity = (azimuthMotor.getSelectedSensorVelocity() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
    absoluteNavYaw = navX.getYaw();
    relativeLimeLightYaw = getLimeLightYaw() + offset;
    if(relativeLimeLightYaw != 0) {
      absoluteAzimuthToTarget = relativeLimeLightYaw + azimuthEncoderPosition + absoluteNavYaw;
    }
    azimuthTargetAngle = (absoluteAzimuthToTarget-absoluteNavYaw)%360;
    clampedAzimuthTargetAngle = MathUtil.clamp(azimuthTargetAngle, Constants.AZIMUTH_LOWER_LIMIT, Constants.AZIMUTH_UPPER_LIMIT);
    azimuthMotorPower = azimuthPID.runVelocityPID(clampedAzimuthTargetAngle, azimuthEncoderPosition, azimuthEncoderVelocity);
    azimuthReady = azimuthTargetAngle == clampedAzimuthTargetAngle && Math.abs(azimuthPID.getError()) < Constants.AZIMUTH_MAX_ERROR;
    SmartDashboard.putNumber("Target Azimuth", azimuthTargetAngle);
    SmartDashboard.putNumber("Azimuth Angle", azimuthEncoderPosition);
    SmartDashboard.putNumber("Azimuth Power", azimuthMotorPower);
    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }

  /** PID Controller used to set the azimuth angle for the shooter
   *  Adjusts the motor power to go to a target angle, in degrees.
   *  @param targetAngle A double representing the target angle that the azimuth angle motor should attempt to reach
   */
  public void moveAzimuthMotorToAngle(double targetAngle) {
    clampedAzimuthTargetAngle = MathUtil.clamp(targetAngle, Constants.AZIMUTH_LOWER_LIMIT, Constants.AZIMUTH_UPPER_LIMIT);
    azimuthEncoderPosition = (azimuthMotor.getSelectedSensorPosition() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;
    azimuthEncoderVelocity = (azimuthMotor.getSelectedSensorVelocity() / 4096 * 360) * Constants.AZIMUTH_GEAR_RATIO;

    azimuthMotorPower = azimuthPID.runVelocityPID(clampedAzimuthTargetAngle, azimuthEncoderPosition, azimuthEncoderVelocity);
    azimuthReady = targetAngle == clampedAzimuthTargetAngle && Math.abs(azimuthPID.getError()) < Constants.AZIMUTH_MAX_ERROR;
    azimuthMotor.set(ControlMode.PercentOutput, azimuthMotorPower);
  }

  /** PI Controller used to set the elevation angle for the shooter
   *  Adjusts the motor power to go to a target angle, in degrees.
   *  @param targetElevationAngle A double representing the target angle that the elevation angle motor should attempt to reach
   */
  public void moveElevationMotorToAngle(double targetElevationAngle) {

    SmartDashboard.putNumber("Target Elevation Angle:", targetElevationAngle);

    clampedElevationTargetAngle = MathUtil.clamp(targetElevationAngle, Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT, Constants.SHOOTER_ELEVATION_ANGLE_UPPER_LIMIT);
    elevationEncoderPosition = ((elevationAngleMotor.getSelectedSensorPosition()*Constants.ELEVATION_ANGLE_GEAR_RATIO)/4096 * 360) + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT;

    //SmartDashboard.putNumber("Raw Encoder Angle Degrees",(elevationAngleMotor.getSelectedSensorPosition())/4096 * 360 + Constants.SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT);
    SmartDashboard.putNumber("Current Elevation Angle:", elevationEncoderPosition);

    elevationMotorPower = elevationAnglePID.runPID(clampedElevationTargetAngle, elevationEncoderPosition);
    elevationReady = clampedElevationTargetAngle == targetElevationAngle && Math.abs(elevationAnglePID.getError()) < Constants.ELEVATION_MAX_ERROR;

    SmartDashboard.putNumber("Elevation Angle Motor Power:", elevationMotorPower);

    elevationAngleMotor.set(ControlMode.PercentOutput, elevationMotorPower);
  }

  public void stopAimingMotors() {
    elevationAngleMotor.neutralOutput();
    azimuthMotor.neutralOutput();
  }

  public void stopElevationMotor() {
    elevationAngleMotor.neutralOutput();
  }

  public boolean getIsAimReady() {
    return shooterReady && azimuthReady && elevationReady;
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
  public void setShooterMotorsVelocity(double targetShooterVelocity) {

    // Gets the velocity of the shooter
    shooterVelocity = leftShooterMotor.getSelectedSensorVelocity();

    // Runs the controller
    shooterPower = shooterPID.runPID(targetShooterVelocity, shooterVelocity);

    shooterReady = Math.abs(shooterPID.getError()) < Constants.SHOOTER_MAX_ERROR;

    // Sets the motors to the computed power level
    leftShooterMotor.set(ControlMode.PercentOutput, shooterPower);
    rightShooterMotor.set(ControlMode.PercentOutput, shooterPower);

    // Displays useful values in Smart Dashboard
    SmartDashboard.putNumber("Shooter Power:", shooterPower);
    SmartDashboard.putString("Shooter Status:", "Shooting");
    SmartDashboard.putNumber("Shooter Target Speed", targetShooterVelocity);
    SmartDashboard.putNumber("Shooter Speed", shooterVelocity);
  }

  public void setShooterMotorsPower(double Speed) {
    leftShooterMotor.set(ControlMode.PercentOutput, Speed);
    rightShooterMotor.set(ControlMode.PercentOutput, Speed);
  }

  public void stopShooterMotors() {
    leftShooterMotor.neutralOutput();
    rightShooterMotor.neutralOutput();
    SmartDashboard.putString("Shooter Status:", "Stopped");
  }

  public void lerpShooter(double limeLightElevationAngle) {
    angleToGoalDegrees = limeLightElevationAngle + Constants.LIMELIGHT_ELEVATION_ANGLE;
    angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    distanceToGoal = (Constants.GOAL_HEIGHT-Constants.LIMELIGHT_HEIGHT)/(Math.tan(angleToGoalRadians));
    SmartDashboard.putNumber("Distance", distanceToGoal);
    int i; 
    for(i=0;i<Constants.SHOOTER_DATA.length && distanceToGoal > Constants.SHOOTER_DATA[i][0]; i++){
      //Searching for the right i
    }    
    if(i>=Constants.SHOOTER_DATA.length){
      //don't shoot
      return;
    }
    if(i==0){
      //shoot at min power, too close
      return;
    }

    double longDistance = Constants.SHOOTER_DATA[i][0];
    double shortDistance = Constants.SHOOTER_DATA[i-1][0];
    
    double highPower = Constants.SHOOTER_DATA[i][1];
    double lowPower = Constants.SHOOTER_DATA[i-1][1];

    double largeAnlge = Constants.SHOOTER_DATA[i][2];
    double smallAngle = Constants.SHOOTER_DATA[i-1][2];

    double distanceSteps = longDistance - shortDistance;
    double lerpFactor = (distanceToGoal - shortDistance)/distanceSteps;

    double finalPower = MathUtil.interpolate(lowPower, highPower, lerpFactor);
    double finalAngle = MathUtil.interpolate(smallAngle, largeAnlge, lerpFactor);

    setShooterMotorsVelocity(finalPower);
    moveElevationMotorToAngle(finalAngle);

  }

  public boolean autoShoot() {
    if(ballDetected()) {
      shouldMiss = !matchColorToAlliance(getAlliance());
      autoMissTimer = 0;
    } else {
      autoMissTimer = autoMissTimer + 1;
      if(autoMissTimer > 150) {
        shouldMiss = false;
      }
    }
    if(shouldMiss) {
      moveAzimuthMotorToLimeLight(Constants.AZIMUTH_BARF_ANGLE);
    } else {
      moveAzimuthMotorToLimeLight(0);
    }
    double limeLightElevationAngle = getLimeLightElevation();
    if(limeLightElevationAngle != 0) {
      lerpShooter(limeLightElevationAngle);
    } else {
      stopShooterMotors();
      stopElevationMotor();
    }
    if(getIsAimReady()) {
      shotProbability = shotProbability + 1;
      if(shotProbability > Constants.SHOT_PROBABILITY_THRESHOLD) {
        feedOff();
        return true;
      }
      feedOn();
    } else {
      shotProbability = 0;
      feedReverse();
    }
    return false;

  }

  public void feedOn(){
    feedMotor.set(ControlMode.PercentOutput, Constants.FEED_POWER);
    SmartDashboard.putString("Feeder Status:", "Feeding");
  }

  public void feedOff(){
    feedMotor.neutralOutput();
    SmartDashboard.putString("Feeder Status:", "Stopped");
  }

  public void feedReverse(){
    feedMotor.set(ControlMode.PercentOutput, -0.15);
    SmartDashboard.putString("Feeder Status:", "Reverse");
  }

}
