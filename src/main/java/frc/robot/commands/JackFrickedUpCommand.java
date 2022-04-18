/** Resets all values to its original state and sets the arms to its lowered position
 *  Used if Jack fricks up and runs CLimbingCommand.java too early
 * @author Cracker
 * @author Icarus Innovated
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbingSub;

public class JackFrickedUpCommand extends CommandBase {
  private ClimbingSub m_climbingSub;

  public JackFrickedUpCommand(ClimbingSub climbingSub) {
    m_climbingSub = climbingSub;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  //   m_climbingSub.SetArmsWithClamp(Constants.MIN_ARM_POSITION, Constants.ARM_SLOW_SPEED);
  //   SmartDashboard.putString("Climb Resetting?", "Yes");
      //Sets arm position to its retracted position
  }

  @Override
  public void end(boolean interrupted) {
    // m_climbingSub.setCurrentStage(0);
    // m_climbingSub.setCurrentTarget(Constants.MAX_ARM_POSITION);
    // m_climbingSub.ReturnArms();
    // m_climbingSub.setHasRun(false);
    //   //Sets arm variables to their original state 
    // m_climbingSub.ArmsStationary();
    // SmartDashboard.putString("Climb Resetting?", "No");
    m_climbingSub.setCurrentStage(m_climbingSub.getCurrentStage() - 1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
