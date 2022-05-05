package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;

/** A highly customizable PID controller
 *  @author Ian
 *  @author Icarus Innovated
 */
public class ConfigurablePID {
  private double proportionalGain = 0;
  private double integralGain = 0;
  private double derivitiveGain = 0;

  private double maxProportional = 0;
  private double maxIntegral = 0;
  private double maxDerivitive = 0;

  private double minOutput = 0;
  private double maxOutput = 0;

  private double proportional = 0;
  private double integral = 0;
  private double derivitive = 0;

  private double currentError = 0;
  private double pastError = 0;
  private double errorDelta = 0;

  private double output = 0;

  private double speed = 1;

  /** A configurable PID controller. which MUST be applied to something that directly affects the processVariable, otherwise very bad things may happen.
   * 
   * @param proportionalGain A double that is multipled by the error when running the controller. The result is the proportional component of the output.
   * @param integralGain A double that is multipled by the error and then added to an accumulator when running the controller. The result is the integral component of the output.
   * @param derivitiveGain A double that is multipled by the rate of change of the error when running the controller. The result is the derivitive component of the output.
   * @param maxProportional A double that controls the maximum allowed value (+-) for the proportional component of the controller.
   * @param maxIntegral A double that controls the maximum allowed value (+-) for the integral component of the controller.
   * @param maxDerivitive A double that controls the maximum allowed value (+-) for the derivitive component of the controller.
   * @param minOutput A double that controls the minimum allowed value for the output of the controller.
   * @param maxOutput A double that controls the maximum allowed value for the output of the controller.
   * @param speed A double representing the ratio between the error and the target velocity of the controller when being run in velocity mode.
   */
  public ConfigurablePID(double proportionalGain, double integralGain, double derivitiveGain, double maxProportional, double maxIntegral, double maxDerivitive, double minOutput, double maxOutput, double speed) {
    this.proportionalGain = proportionalGain;
    this.integralGain = integralGain;
    this.derivitiveGain = derivitiveGain;

    this.maxProportional = maxProportional;
    this.maxIntegral = maxIntegral;
    this.maxDerivitive = maxDerivitive;

    this.minOutput = minOutput;
    this.maxOutput = maxOutput;

    this.speed = speed;
  }

  /** Runs the PID controller in the standard way, where it targets the setpoint.
   * 
   * @param setpoint The desired value the controller should attempt to reach.
   * @param processVariable The current measured position of what the controller is controlling, usually an encoder or gyro.
   * @return The computed output of the controller, targeting the setpoint.
   */
  public double runPID(double setpoint, double processVariable) {

    this.currentError = setpoint - processVariable;
    this.errorDelta = this.currentError - this.pastError;
    this.pastError = this.currentError;

    this.proportional = MathUtil.clamp(this.currentError * this.proportionalGain, -this.maxProportional, this.maxProportional);
    this.integral = MathUtil.clamp(this.integral + (this.currentError * this.integralGain), -this.maxIntegral, this.maxIntegral);
    this.derivitive = MathUtil.clamp(this.errorDelta * this.derivitiveGain, -this.maxDerivitive, this.maxDerivitive);

    this.output = MathUtil.clamp(this.proportional + this.integral + this.derivitive, this.minOutput, this.maxOutput);

    return this.output;
  }

  /** Runs the PID controller in velocity mode. This causes the controller to target a speed that is based on the error. Typically this is significantly easier and safer than the standard mode.
   * 
   * @param setpoint The desired value the controller should attempt to reach.
   * @param processVariable The current measured position of what the controller is controlling, usually an encoder or gyro.
   * @param processVariableVelocity The current measued speed of what the controller is controlling.
   * @return The computed output of the controller, targeting a velocity based on the difference between the setpoint and the process variable.
   */
  public double runVelocityPID(double setpoint, double processVariable, double processVariableVelocity) {
    
    this.currentError = (setpoint - processVariable) * this.speed;
    this.currentError = this.currentError - processVariableVelocity;
    this.errorDelta = this.currentError - this.pastError;
    this.pastError = this.currentError;

    this.proportional = MathUtil.clamp(this.currentError * this.proportionalGain, -this.maxProportional, this.maxProportional);
    this.integral = MathUtil.clamp(this.integral + (this.currentError * this.integralGain), -this.maxIntegral, this.maxIntegral);
    this.derivitive = MathUtil.clamp(this.errorDelta * this.derivitiveGain, -this.maxDerivitive, this.maxDerivitive);

    this.output = MathUtil.clamp(this.proportional + this.integral + this.derivitive, this.minOutput, this.maxOutput);

    return this.output;
  }

  /**
   * Get the speed used by the controller when being run in velocity mode.
   *
   * @return A double representing the ratio between the error and the target velocity of the controller when being run in velocity mode.
   */
  public double getSpeed() {
    return this.speed;
  }

  /**
   * Set the speed used by the controller when being run in velocity mode.
   *
   * @param newSpeed A double representing the ratio between the error and the target velocity of the controller when being run in velocity mode.
   */
  public void setSpeed(double newSpeed) {
    this.speed = newSpeed;
  }

  /**
   * Get the minimum allowed output for the controller.
   *
   * @return minimum output
   */
  public double getMinOutput() {
    return this.minOutput;
  }

  /**
   * Set the minimum allowed output for the controller.
   *
   * @param newMinOutput the minimum output of the controller
   */
  public void setMinOutput(double newMinOutput) {
    this.minOutput = newMinOutput;
  }

  /**
   * Get the maximum allowed output for the controller.
   *
   * @return maximum output
   */
  public double getMaxOutput() {
    return this.maxOutput;
  }

  /**
   * Set the maximum allowed output for the controller.
   *
   * @param newMaxOutput the maximum output of the controller
   */
  public void setMaxOutput(double newMaxOutput) {
    this.maxOutput = newMaxOutput;
  }

  /**
   * Get the maximum allowed proportional component for the controller.
   *
   * @return maximum proportional component
   */
  public double getMaxProportional() {
    return this.maxProportional;
  }

  /**
   * Set the maximum allowed proportional component for the controller.
   *
   * @param newMaxProportional the maximum proportional component
   */
  public void setMaxProportional(double newMaxProportional) {
    this.maxProportional = newMaxProportional;
  }

  /**
   * Get the maximum allowed integral component for the controller.
   *
   * @return maximum integral component
   */
  public double getMaxIntegral() {
    return this.maxIntegral;
  }

  /**
   * Set the maximum allowed integral component for the controller.
   *
   * @param newMaxIntegral the maximum integral component
   */
  public void setMaxIntegral(double newMaxIntegral) {
    this.maxIntegral = newMaxIntegral;
  }

  /**
   * Get the maximum allowed derivitive component for the controller.
   *
   * @return maximum derivitive component
   */
  public double getMaxDerivitive() {
    return this.maxDerivitive;
  }

  /**
   * Set the maximum allowed derivitive component for the controller.
   *
   * @param newMaxDerivitive the maximum derivitive component
   */
  public void setMaxDerivitive(double newMaxDerivitive) {
    this.maxDerivitive = newMaxDerivitive;
  }

  /**
   * Get the proportional gain of the controller.
   *
   * @return proportional gain
   */
  public double getProportionalGain() {
    return this.proportionalGain;
  }

  /**
   * Set the proportional gain of the controller.
   *
   * @param newProportionalGain the proportional gain
   */
  public void setProportionalGain(double newProportionalGain) {
    this.proportionalGain = newProportionalGain;
  }

  /**
   * Get the integral gain of the controller.
   *
   * @return integral gain
   */
  public double getIntegralGain() {
    return this.integralGain;
  }

  /**
   * Get the total integral of the controller.
   *
   * @return integral
   */
  public double getIntegral() {
    return this.integral;
  }

  /**
   * Set the integral gain of the controller.
   *
   * @param newIntegralGain the integral gain
   */
  public void setIntegralGain(double newIntegralGain) {
    this.integralGain = newIntegralGain;
  }

  /**
   * Get the derivitive gain of the controller.
   *
   * @return derivitive gain
   */
  public double getDerivitiveGain() {
    return this.derivitiveGain;
  }

  /**
   * Set the derivitive gain of the controller.
   *
   * @param newDerivitiveGain the derivitive gain
   */
  public void setDerivitiveGain(double newDerivitiveGain) {
    this.derivitiveGain = newDerivitiveGain;
  }

  /**
   * Get the last computed error value
   * 
   * @return last computed error
   */
  public double getError() {
    return this.currentError;
  }

  /**
   * Reset values to 0.
   */
  public void resetValues() {
    this.proportional = 0;
    this.integral = 0;
    this.derivitive = 0;
    this.currentError = 0;
    this.pastError = 0;
  }
}
