/** Subsystem for the climbing mechanism for the 2022 FRC competition Rapid React
 *  @author Cracker
 *  @author Icarus Innovated
 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.ConfigurablePID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;

public class ClimbingSub extends SubsystemBase {
  private final Solenoid armSwingSolenoid;
  private final WPI_TalonFX leftArmMotor;
  private final WPI_TalonFX rightArmMotor;
  private final WPI_TalonFX armWinchMotor;

  private final ConfigurablePID leftArmPID;
  private final ConfigurablePID rightArmPID;
  private final ConfigurablePID armWinchPID;
  private final SupplyCurrentLimitConfiguration armCurrentLimit;

  private double rightArmMotorVelocity;
  private double leftArmMotorVelocity;
  private double armWinchVelocity;

  private double rightArmMotorPosition;
  private double leftArmMotorPosition;
  private double armWinchPosition;

  private double currentArmTarget;
  private double currentArmSpeed;
  private double currentWinchTarget;
  private int currentStage;
  private boolean hasRun;

  public ClimbingSub() {

    armSwingSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.ARM_SWING_SOLENOID);
    leftArmMotor = new WPI_TalonFX(Constants.LEFT_ARM_MOTOR);
    rightArmMotor = new WPI_TalonFX(Constants.RIGHT_ARM_MOTOR);
    armWinchMotor = new WPI_TalonFX(Constants.ARM_WINCH_MOTOR); //kBrushless for 21-1650 ONLY

    armCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.ARM_CURRENT_LIMIT, 0, 0.1);
    
    leftArmPID = new ConfigurablePID(
      Constants.ARM_PROPORTIONAL_GAIN,
      Constants.ARM_INTEGRAL_GAIN,
      Constants.ARM_DERIVITIVE_GAIN,
      Constants.MAX_ARM_PROPORTIONAL,
      Constants.MAX_ARM_INTEGRAL,
      Constants.MAX_ARM_DERIVITIVE,
      -Constants.ARM_MAX_POWER,
      Constants.ARM_MAX_POWER,
      Constants.ARM_SLOW_SPEED
    );

    rightArmPID = new ConfigurablePID(
      Constants.ARM_PROPORTIONAL_GAIN,
      Constants.ARM_INTEGRAL_GAIN,
      Constants.ARM_DERIVITIVE_GAIN,
      Constants.MAX_ARM_PROPORTIONAL,
      Constants.MAX_ARM_INTEGRAL,
      Constants.MAX_ARM_DERIVITIVE,
      -Constants.ARM_MAX_POWER,
      Constants.ARM_MAX_POWER,
      Constants.ARM_SLOW_SPEED
    );

    armWinchPID = new ConfigurablePID(
      Constants.ARM_WINCH_PROPORTIONAL_GAIN,
      Constants.ARM_WINCH_INTEGRAL_GAIN,
      Constants.ARM_WINCH_DERIVITIVE_GAIN,
      Constants.MAX_ARM_WINCH_PROPORTIONAL,
      Constants.MAX_ARM_WINCH_INTEGRAL,
      Constants.MAX_ARM_WINCH_DERIVITIVE,
      -Constants.ARM_WINCH_MAX_POWER,
      Constants.ARM_WINCH_MAX_POWER,
      Constants.ARM_WINCH_SPEED
    );

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();
    armWinchMotor.configFactoryDefault();

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);
    armWinchMotor.setNeutralMode(NeutralMode.Brake);

    leftArmMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightArmMotor.setInverted(TalonFXInvertType.Clockwise);
    armWinchMotor.setInverted(TalonFXInvertType.Clockwise);

    leftArmMotor.setSensorPhase(true);
    armWinchMotor.setSensorPhase(true);
    rightArmMotor.setSensorPhase(false);

    leftArmMotor.configOpenloopRamp(Constants.ARM_POWER_RAMP_TIME, 0);
    rightArmMotor.configOpenloopRamp(Constants.ARM_POWER_RAMP_TIME, 0);
    armWinchMotor.configOpenloopRamp(Constants.ARM_POWER_RAMP_TIME, 0);

    leftArmMotor.configSupplyCurrentLimit(armCurrentLimit);
    rightArmMotor.configSupplyCurrentLimit(armCurrentLimit);
    armWinchMotor.configSupplyCurrentLimit(armCurrentLimit); //60 is upper limit for brushless model

    currentArmTarget = Constants.FIRST_HOOK_POSITION;
    currentArmSpeed = Constants.ARM_FAST_SPEED;
    currentStage = 0;
    currentWinchTarget = 0;
    hasRun = false;
  }

  /** Extends the climbing arms
   *  WARNING! This command has no automatic stop programmed into it and will break the robot if not used correctly
   *  @Deprecated
   */
  public void ExtendArms(){
    leftArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    rightArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    System.out.println("ExtendArms() is a deprecated function!");
  }

  /** Retracts the climbing arms
   *  WARNING! This command has no automatic stop programmed into it and will break the robot if not used correctly
   *  @Deprecated
   */
  public void RetractArms(){
    leftArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    rightArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    System.out.println("RetractArms() is a deprecated function!");
  }

  /** Sets the arms to not move
   * 
   */
  public void ArmsStationary() {
    leftArmMotor.neutralOutput();
    rightArmMotor.neutralOutput();
    armWinchMotor.neutralOutput();
  }
  
  /** Toggles the solonoids on the arms to on
   * 
   */
  public void SwingArms(){
    armSwingSolenoid.set(true);
  }

  /** Toggles the solonoids on the arms to off
   * 
   */
  public void ReturnArms(){
    armSwingSolenoid.set(false);
  }

  /** Turns on the winch motor
   * 
   */
  public void startWinch() {
    SmartDashboard.putNumber("WINCH POS", armWinchMotor.getSelectedSensorPosition());
    armWinchMotor.set(Constants.WINCH_SPEED);
  }

  /** Reverses the winch motor
   * 
   */
  public void revWinch() {
    SmartDashboard.putNumber("WINCH POS", armWinchMotor.getSelectedSensorPosition());
    armWinchMotor.set(-Constants.WINCH_SPEED);
  }

  /** Turns off the winch motor
   * 
   */
  public void stopWinch() {
    armWinchMotor.stopMotor();
  }

  /** Proportional Integral Controller used for the arms
   *  Scales the motors power to reach the target position as fast as possible
   *  @param targetPosition A double representing the target position tracked by the encoder that the controller will attempt to reach
   */
  public void SetArmsWithClamp(double targetPosition, double targetVelocity)
  {
    // Gets the position and velocity of both arm motors
    rightArmMotorPosition = rightArmMotor.getSelectedSensorPosition();
    leftArmMotorPosition = leftArmMotor.getSelectedSensorPosition();
    rightArmMotorVelocity = rightArmMotor.getSelectedSensorVelocity();
    leftArmMotorVelocity = leftArmMotor.getSelectedSensorVelocity();
    
    /** Gets the average position between the two arms, and then clamps the target for each arm to be within an acceptable range of it. 
     * This ensures the arms do not get out of sync, as that would tip the robot
     */
    double averageArmPosition = (leftArmMotorPosition + rightArmMotorPosition)/2;
    double actualTarget = MathUtil.clamp(targetPosition, averageArmPosition - Constants.MAX_ARM_ERROR, averageArmPosition + Constants.MAX_ARM_ERROR);

    leftArmPID.setSpeed(targetVelocity);
    rightArmPID.setSpeed(targetVelocity);

    double leftArmPower = leftArmPID.runVelocityPID(actualTarget, leftArmMotorPosition, leftArmMotorVelocity);
    double rightArmPower = rightArmPID.runVelocityPID(actualTarget, rightArmMotorPosition, rightArmMotorVelocity);

    //SmartDashboard.putNumber("Left Arm Power:", leftArmPower);
    //SmartDashboard.putNumber("Right Arm Power:", rightArmPower);

    leftArmMotor.set(ControlMode.PercentOutput, leftArmPower);
    rightArmMotor.set(ControlMode.PercentOutput, rightArmPower);
  }

  /** Gets the current target for use in SetArmsWithClamp() later in the code
   *  @return A double representing the location of the current target in terms of a motor encoder
   */
  public double getCurrentTarget()
  {
    return currentArmTarget;
  }

  /** Sets the current target for use in SetArmsWithClamp() later in the code
   *  @param target A double representing the location of the target in terms of a motor encoder
   */
  public void setCurrentTarget(double target)
  {
    currentArmTarget = target;
  }

  /** Gets the current speed for use in SetArmsWithClamp() later in the code
   *  @return A double representing the speed of the arms
   */
  public double getCurrentSpeed()
  {
    return currentArmSpeed;
  }

  /** Sets the current speed for use in SetArmsWithClamp() later in the code
   *  @param speed A double representing the speed of the arms
   */
  public void setCurrentSpeed(double speed)
  {
    currentArmSpeed = speed;
  }

  /** Gets the current stage the robot is on when climbing
   * @return An int representing the current stage
   */
  public int getCurrentStage()
  {
    return currentStage;
  }

  /** Sets the current stage the robot is on when climbing
   * @param stage An int representing what stage the robot currently is on
   */
  public void setCurrentStage(int stage)
  {
    currentStage = stage;
  }

  /** Sets whether or not the switch case has run in ClimbingCommand.java
   * @param hasSwitchRun A boolean representing whether or not the switch case has run
   */
  public void setHasRun(boolean hasSwitchRun)
  {
    hasRun = hasSwitchRun;
  }

  /** Gets whether or not the switch case has run in ClimbingCommand.java
   *  @return A boolean representing wheter or not the switch case has run
   */
  public boolean getHasRun()
  {
    return hasRun;
  }

  /** Gets the encoder position for the left motor 
   *  @return A double representing the left motor encoder position
   */
  public double getLeftCoderPos()
  {
    return leftArmMotor.getSelectedSensorPosition();
  }

  /** Gets the encoder position for the right motor
   *  @return A double representing the right motor encoder position
   */
  public double getRightCoderPos()
  {
    return rightArmMotor.getSelectedSensorPosition();
  }

  /** Checks if the arms are at the target position
   * @param target A double representing the location of the target position
   * @return A boolean representing whether or not that target position has been reached
   */
  public boolean hasReachedPosition(double target)
  {
    double pos = rightArmMotor.getSelectedSensorPosition();
    if(target > pos - Constants.ACCEPTABLE_ERROR &&
       target < pos + Constants.ACCEPTABLE_ERROR)
    {
      System.out.println("Target Reached");
      return true;
    }
    return false;
  }

  /** Gets the current target for use in SetArmsWithClamp() later in the code
   *  @return A double representing the location of the current target in terms of a motor encoder
   */
  public double getCurrentWinchTarget()
  {
    return currentWinchTarget;
  }

  /** Sets the current target for use in SetArmsWithClamp() later in the code
   *  @param target A double representing the location of the target in terms of a motor encoder
   */
  public void setCurrentWinchTarget(double target)
  {
    currentWinchTarget = target;
  }

  public void moveArmWinchToPosition(double targetPosition) {
    armWinchPosition = armWinchMotor.getSelectedSensorPosition();
    armWinchVelocity = armWinchMotor.getSelectedSensorVelocity();
    targetPosition = MathUtil.clamp(targetPosition, Constants.ARM_WINCH_MIN_POSITION, Constants.ARM_WINCH_MAX_POSITION);
    double armWinchPower = armWinchPID.runVelocityPID(targetPosition, armWinchPosition, armWinchVelocity);
    armWinchMotor.set(ControlMode.PercentOutput, armWinchPower);
    // SmartDashboard.putNumber("Arm Winch Position", armWinchPosition);
    // SmartDashboard.putNumber("Arm Winch Target", targetPosition);
    // SmartDashboard.putNumber("Arm Winch Power", armWinchPower);
  }


  @Override
  public void periodic() {} 
}
