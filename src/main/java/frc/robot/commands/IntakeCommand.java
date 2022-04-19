// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShootingSub;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private IntakeSub m_intakeSub;
  private ShootingSub m_shootSub;

  public IntakeCommand(IntakeSub intakeSub, ShootingSub shootSub) {
    m_intakeSub = intakeSub;
    m_shootSub = shootSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    if(!m_shootSub.getSwitch())
    {
      m_shootSub.feedOff();
      //System.out.println("off lol");
    }
    else
    {
      m_shootSub.intakeFeedOn();
      //System.out.println("ON LOL");
    }
    m_intakeSub.IntakeExtend();
    m_intakeSub.IntakeOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.IntakeRetract();
    m_intakeSub.IntakeOff();
    //m_shootSub.feedOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
