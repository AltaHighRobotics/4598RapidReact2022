// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ClimbingPistonCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.VisionTestCommand;
import frc.robot.subsystems.ClimbingSub;
import frc.robot.subsystems.ComputerVisionSub;
import frc.robot.subsystems.IntakeSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ComputerVisionSub m_visionSub = new ComputerVisionSub();
  private final ClimbingSub m_climbingSub = new ClimbingSub();
  private final IntakeSub m_intakeSub = new IntakeSub();

  private final VisionTestCommand m_visionCommand = new VisionTestCommand(m_visionSub);
  private final ClimbingPistonCommand m_climbingPistonCommand = new ClimbingPistonCommand(m_climbingSub);
  private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSub);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    CommandScheduler.getInstance().setDefaultCommand(m_visionSub, m_visionCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
