// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub;

public class DriveCommand extends CommandBase 
{
  /** Creates a new DriveCommand. */
  private DriveTrainSub driveTrain;
  private PS4Controller controller;
  public DriveCommand(DriveTrainSub subsystem, PS4Controller ps4Controller) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = subsystem;
    this.controller = ps4Controller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double speed = controller.getRawAxis(Constants.Y_AXIS);
    double turn = controller.getRawAxis(Constants.X_AXIS);
    driveTrain.setArcadeDrive(speed, turn);
    //System.out.println("Moving");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}