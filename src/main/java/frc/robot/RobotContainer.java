// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PS4Controller m_driverOne = new PS4Controller(Constants.DRIVER_ONE);

  private final ClimbingSub m_climbingSub = new ClimbingSub();
  // private final IntakeSub m_intakeSub = new IntakeSub();
  private final DriveTrainSub m_driveTrainSub = new DriveTrainSub();
  // private final ColorSub m_colorSub = new ColorSub();
  private final ShooterSub m_ShooterSub = new ShooterSub();
  private final StorageSub m_StorageSub = new StorageSub();
 
  
  private final JackFrickedUpCommand m_jackFrickedUpCommand = new JackFrickedUpCommand(m_climbingSub);
  private final ClimbingCommand m_climbingCommand = new ClimbingCommand(m_climbingSub);

  private final DriveCommand m_driveCommand =  new DriveCommand(m_driveTrainSub, m_driverOne);
  private final ConstantShootCommand m_ConstantShootCommand = new ConstantShootCommand(m_ShooterSub);
  private final FeedCommand m_FeedCommand = new FeedCommand(m_StorageSub);
  // private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSub);
  // private final ColorCommand m_colorCommand = new ColorCommand(m_colorSub);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
     CommandScheduler.getInstance().setDefaultCommand(m_driveTrainSub, m_driveCommand);
  //   CommandScheduler.getInstance().setDefaultCommand(m_computerVisionSub, m_intakeVisionCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton climbButton;
    final JoystickButton frickButton;
    final JoystickButton feedButton;
    final JoystickButton shootButton;

    climbButton = new JoystickButton(m_driverOne, 2);
    frickButton = new JoystickButton(m_driverOne, 9);
    feedButton = new JoystickButton(m_driverOne, 5);
    shootButton = new JoystickButton(m_driverOne, 6);

    climbButton.toggleWhenPressed(m_climbingCommand);
    frickButton.toggleWhenPressed(m_jackFrickedUpCommand);
    feedButton.toggleWhenPressed(m_ConstantShootCommand);
    shootButton.toggleWhenPressed(m_FeedCommand);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // Janky. Done to make robot work in tele-op. Will not actually work for autonomous.
    return null;
  }
}
