package frc.robot.utilities;

import java.util.ArrayList;

public class TargetLeading {
  private vector position = new vector(0, 0, 0);
  private vector previousPosition = new vector(0, 0, 0);
  private ArrayList<vector> positions = new ArrayList<vector>();
  private vector averagePosition = new vector(0, 0, 0);
  private vector predictedPosition = new vector(0, 0, 0);
  private vector velocity = new vector(0, 0, 0);
  private vector previousVelocity = new vector(0, 0, 0);
  private ArrayList<vector> velocities = new ArrayList<vector>();
  private vector averageVelocity = new vector(0, 0, 0);
  private vector acceleration = new vector(0, 0, 0);
  private ArrayList<vector> accelerations = new ArrayList<vector>();
  private vector averageAcceleration = new vector(0, 0, 0);
  private double distance = 0;
  private double speed = 0;
  private double velocityThreshold = 0;
  private int age = 1;
  private int refreshThreshold = 0;
  private int maxMemory = 0;
  private int maxAccelerationPrediction = 0;
  private boolean updated = false;
  
  public TargetLeading(double velocityThreshold, int refreshThreshold, int maxMemory, int maxAccelerationPrediction) {
    this.velocityThreshold = velocityThreshold;
    this.refreshThreshold = refreshThreshold;
    this.maxMemory = maxMemory;
    this.maxAccelerationPrediction = maxAccelerationPrediction;
  }

  public void setNewTargetPosition(vector newPosition) {
    this.position = newPosition;
    this.updated = true;
  }
  public void updateTarget(vector robotPosition) {
    if (this.updated) {
      if (this.age > this.refreshThreshold) {
        this.positions.clear();
        this.velocities.clear();
        this.accelerations.clear();
      }
      this.positions = this.manageList(this.positions,this.position, this.maxMemory);
      this.averagePosition = this.avgVectorList(this.positions);
      this.predictedPosition.copy(this.averagePosition);
      this.velocity = this.averagePosition.getSubtraction(this.previousPosition);
      this.velocity.divide(this.age);
      this.velocities = this.manageList(this.velocities, this.velocity, this.maxMemory);
      this.averageVelocity = this.avgVectorList(this.velocities);
      this.previousPosition.copy(this.averagePosition);
      this.acceleration = this.averageVelocity.getSubtraction(this.previousVelocity);
      this.acceleration.divide(this.age);
      this.accelerations = this.manageList(this.accelerations, this.acceleration, this.maxMemory);
      this.averageAcceleration = this.avgVectorList(this.accelerations);
      this.previousVelocity.copy(this.averageVelocity);
      this.speed = this.averageVelocity.magnitude();
      if (this.speed < this.velocityThreshold) {
        this.averageVelocity.set(0,0,0);
        this.averageAcceleration.set(0,0,0);
      }
      this.age = 0;
      this.updated = false;
    }
    this.age = this.age + 1;
    this.predictedPosition.add(this.averageVelocity.getAddition(this.averageAcceleration.getMultiplication(Math.min(this.age, this.maxAccelerationPrediction))));
    this.distance = this.predictedPosition.getSubtraction(robotPosition).magnitude();
  }

  public double getTargetDistance() 
  {
    return this.distance;
  }

  /** Gets the average of a list of vectors
   * 
   * @param vectorList an ArrayList of vector objects
   * @return A vector representing the average value of the vectors in the list.
   */
  private vector avgVectorList(ArrayList<vector> vectorList) {
    if(vectorList.size() >= 1) {
      vector total = vectorList.get(0).clone();
      total.set(0, 0, 0);
      for(vector i : vectorList) {
          total.add(i);
      }
      total.divide(vectorList.size());
      return total;
    }
    return new vector(0, 0, 0);
  }

  private ArrayList<vector> manageList(ArrayList<vector> vectorList, vector newItem, int maxSize) {
    vectorList.add(newItem);
    if(vectorList.size() > maxSize) {
      vectorList.remove(0);
    }
    return vectorList;
  }
}