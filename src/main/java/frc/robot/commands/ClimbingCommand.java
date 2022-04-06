/** Command that climbs all of the monkey bars in the 2022 FRC competion Rapid React
 * Needs a seprate command to reset this command to original values (see JackFrickedUpCommand.java)
 * @author Cracker
 * @author Icarus Innovated
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbingSub;

public class ClimbingCommand extends CommandBase {
  private ClimbingSub m_climbingSub;
  private double currentTarget;
    //Current target the robot is attempting to reach
  private double currentSpeed;
    //Current speed the arms should travel at
  private int currentStage;
    //Current stage of climbing the robot is on

  public ClimbingCommand(ClimbingSub climbingSub) {
    m_climbingSub = climbingSub;
  }

  @Override
  public void initialize() {
    currentTarget = m_climbingSub.getCurrentTarget();
    currentSpeed = m_climbingSub.getCurrentSpeed();
    currentStage = m_climbingSub.getCurrentStage();
  }

  @Override
  public void execute() {
    currentTarget = m_climbingSub.getCurrentTarget();
    currentSpeed = m_climbingSub.getCurrentSpeed();
    currentStage = m_climbingSub.getCurrentStage();

    SmartDashboard.putNumber("Climb Stage:", currentStage);

    m_climbingSub.SetArmsWithClamp(currentTarget, currentSpeed);
      //Makes climbing arms go to the current Target

    if (m_climbingSub.hasReachedPosition(currentTarget))
      //Checks if the climbing arms are at the current target
    {
      switch (currentStage)
        //Checks what stage the robot is on
      {
        case 1:
          System.out.println("SETTING ARMS TO MIN POSITION");
          SmartDashboard.putString("Climb Target:", "1");
          m_climbingSub.setCurrentTarget(Constants.MIN_ARM_POSITION);
          m_climbingSub.setCurrentSpeed(Constants.ARM_SLOW_SPEED);
          break;
        
        case 2:
          System.out.println("CASE 2");
          SmartDashboard.putString("Climb Target:", "2");
          m_climbingSub.setCurrentTarget(Constants.ALMOST_MIN_POSITION);
          m_climbingSub.setCurrentSpeed(Constants.ARM_FAST_SPEED);
          break;

        case 3:
          System.out.println("CASE 3");
          SmartDashboard.putString("Climb Target:", "3");
          m_climbingSub.SwingArms();
          m_climbingSub.setCurrentTarget(Constants.MAX_ARM_POSITION);
          m_climbingSub.setCurrentSpeed(Constants.ARM_FAST_SPEED);
          break;
        case 4:
          System.out.println("CASE 4");
          SmartDashboard.putString("Climb Target:", "4");
          m_climbingSub.ReturnArms();
          break;
        case 5:
          // System.out.println("CASE 5");
          // SmartDashboard.putString("Climb Target:", "5");
          // m_climbingSub.setCurrentTarget(Constants.MIN_ARM_POSITION);
          // m_climbingSub.setCurrentSpeed(Constants.ARM_FAST_SPEED);
          m_climbingSub.setCurrentStage(1);
          break;
        case 6:
          System.out.println("CASE 6");
          SmartDashboard.putString("Climb Target:", "6");
          m_climbingSub.setCurrentTarget(Constants.MIN_ARM_POSITION);
          m_climbingSub.setCurrentSpeed(Constants.ARM_SLOW_SPEED);
          if(m_climbingSub.getRightCoderPos() < Constants.HALF_ARM_POSITION)
          {
            m_climbingSub.SwingArms();
          }
        case 7:
          if (m_climbingSub.getHasRun())
            m_climbingSub.setCurrentStage(8);
          else
          {
            m_climbingSub.setCurrentStage(3);
          }
          break;
          
        default:
        SmartDashboard.putString("Climb Target:", "Arms Broken");
        System.out.println("Invalid Case");
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_climbingSub.ArmsStationary();
    //Stops the arms
    m_climbingSub.setCurrentStage(currentStage + 1);
    //Increments the stage by one
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
