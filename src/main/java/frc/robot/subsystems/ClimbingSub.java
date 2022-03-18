/** Subsystem for the climbing mechanism for the 2022 FRC competition Rapid React
 *  @author Cracker
 *  @author Icarus Innovated
 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;

import java.lang.Math;

public class ClimbingSub extends SubsystemBase {
  private Solenoid armSwingSolenoid;
  private TalonFX leftArmMotor;
  private TalonFX rightArmMotor;

  private double rightArmMotorVelocity;
  private double leftArmMotorVelocity;

  private double rightArmMotorPosition;
  private double leftArmMotorPosition;

  private double rightArmIntegral;
  private double leftArmIntegral;

  private double currentArmTarget;
  private int currentStage;
  private boolean hasRun;

  public ClimbingSub() {
      armSwingSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.ARM_SWING_SOLENOID);
      leftArmMotor = new TalonFX(Constants.LEFT_ARM_MOTOR);
      rightArmMotor = new TalonFX(Constants.RIGHT_ARM_MOTOR);

      leftArmMotor.configFactoryDefault();
      rightArmMotor.configFactoryDefault();

      leftArmMotor.setNeutralMode(NeutralMode.Brake);
      rightArmMotor.setNeutralMode(NeutralMode.Brake);

      leftArmMotor.setInverted(TalonFXInvertType.Clockwise);
      rightArmMotor.setInverted(TalonFXInvertType.CounterClockwise);

      leftArmMotor.setSensorPhase(true);
      rightArmMotor.setSensorPhase(false);

      currentArmTarget = Constants.MAX_ARM_POSITION;
      currentStage = 0;
      hasRun = false;
    }

  /** Extends the climbing arms
   *  WARNING! This command has no automatic stop programmed into it and will break the robot if not used correctly
   *  @deprecated
   */
  public void ExtendArms(){
    leftArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    rightArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    System.out.println("ExtendArms() is a deprecated function!");
  }

  /** Retracts the climbing arms
   *  WARNING! This command has no automatic stop programmed into it and will break the robot if not used correctly
   *  @deprecated
   */
  public void RetractArms(){
    leftArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    rightArmMotor.set(ControlMode.PercentOutput, Constants.LIFT_ARM_SPEED);
    System.out.println("RetractArms() is a deprecated function!");
  }

  /** Sets the arms to not move
   * 
   */
  public void ArmsStationary(){
    leftArmMotor.set(ControlMode.PercentOutput, 0);
    rightArmMotor.set(ControlMode.PercentOutput, 0);
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

  /** Proportional Integral Controller used for the arms
   *  Scales the motors power to reach the target position as fast as possible
   *  @param targetPosition A double representing the target position tracked by the encoder that the controller will attempt to reach
   */
  public void SetArmsWithClamp(double targetPosition)
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
    double armMin = Math.min(targetPosition, averageArmPosition + Constants.MAX_ARM_ERROR);
    double actualTarget = Math.max(armMin, averageArmPosition - Constants.MAX_ARM_ERROR);

    // Finds the difference between each arms position and the clamped target, then multiplies it by the max speed
    double leftArmError = (actualTarget - leftArmMotorPosition) * Constants.MAX_ARM_SPEED;
    double rightArmError = (actualTarget - rightArmMotorPosition) * Constants.MAX_ARM_SPEED;

    /** Finds the difference between each arms current velocity, and each arms error. 
     *  By using velocity and position, the controller is more stable than if we only used position, 
     *  as it will now slow down to stop directly on the target, rather than coasting past it
     */
    double leftArmVelocityError = leftArmError - leftArmMotorVelocity;
    double rightArmVelocityError = rightArmError - rightArmMotorVelocity;

    double leftArmProportional = leftArmVelocityError * Constants.ARM_PROPORTIONAL_GAIN;
    double rightArmProportional = rightArmVelocityError * Constants.ARM_PROPORTIONAL_GAIN;

    // This adds the current error in speed to the sum of all previous errors. This helps when the arms are under load
    leftArmIntegral = Math.min(Math.max((leftArmIntegral + leftArmVelocityError) * Constants.ARM_INTEGRAL_GAIN, -Constants.MAX_ARM_INTEGRAL), Constants.MAX_ARM_INTEGRAL);
    rightArmIntegral = Math.min(Math.max((rightArmIntegral + rightArmVelocityError) * Constants.ARM_INTEGRAL_GAIN, -Constants.MAX_ARM_INTEGRAL), Constants.MAX_ARM_INTEGRAL);

    double leftArmPower = leftArmProportional + leftArmIntegral;
    double rightArmPower = rightArmProportional + rightArmIntegral;

    SmartDashboard.putNumber("Left Arm Power", leftArmPower);
    SmartDashboard.putNumber("Left Arm Proportional", leftArmProportional);
    SmartDashboard.putNumber("Left Arm Integral", leftArmIntegral);

    SmartDashboard.putNumber("Right Arm Power", rightArmPower);
    SmartDashboard.putNumber("Right Arm Proportional", rightArmProportional);
    SmartDashboard.putNumber("Right Arm Integral", rightArmIntegral);

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

  @Override
  public void periodic() {} 
}
