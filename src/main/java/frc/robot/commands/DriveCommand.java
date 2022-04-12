// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;

public class DriveCommand extends CommandBase 
{
  /** Creates a new DriveCommand. */
  private final DrivetrainSub m_drivetrain;
  private final PS4Controller m_controller;
  public DriveCommand(DrivetrainSub drivetrainSub, PS4Controller ps4Controller) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrainSub;
    m_controller = ps4Controller;
    // CvSource outStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    // MjpegServer mjpegServer2 = new MjpegServer("serve_blur", 1182);
    // mjpegServer2.setSource(outStream);
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_drivetrain.setPos(0, 0);
    m_drivetrain.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double speed = -m_controller.getRawAxis(Constants.PS4_RIGHT_STICK_Y_AXIS);
    double turn = m_controller.getRawAxis(Constants.PS4_RIGHT_STICK_X_AXIS);
    m_drivetrain.setArcadeDrive(speed, turn);
    m_drivetrain.drivetrainPositionIntegration();
    //System.out.println("Moving");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}