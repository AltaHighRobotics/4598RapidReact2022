// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  //private final PS4Controller m_driverTwo = new PS4Controller(1);

  private final ClimbingSub m_climbingSub = new ClimbingSub();
  private final IntakeSub m_intakeSub = new IntakeSub();
  private final DrivetrainSub m_drivetrainSub = new DrivetrainSub();
  private final ShootingSub m_shootingSub = new ShootingSub();
  
  private final JackFrickedUpCommand m_jackFrickedUpCommand = new JackFrickedUpCommand(m_climbingSub);
  private final ClimbingCommand m_climbingCommand = new ClimbingCommand(m_climbingSub);
  private final MultiballAutoCommand m_multiballAutoCommand = new MultiballAutoCommand(m_drivetrainSub, m_shootingSub);
  private final DriveCommand m_driveCommand =  new DriveCommand(m_drivetrainSub, m_driverOne);
  private final ShootCommand m_shootCommand = new ShootCommand(m_shootingSub, m_driverOne, m_intakeSub);
  private final ZeroShooterCommand m_zeroShooterCommand = new ZeroShooterCommand(m_shootingSub);
  private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_intakeSub);
  private final IntakeReverseCommand m_IntakeReverseCommand = new IntakeReverseCommand(m_intakeSub);
  private final FixedShootCommand m_fixedShootCommand = new FixedShootCommand(m_shootingSub, m_driverOne);

  //private final TestAutoCommand m_testAuto = new TestAutoCommand(m_drivetrainSub);

  private SendableChooser<Boolean> m_condition1 = new SendableChooser<>();
  private SendableChooser<Boolean> m_condition2 = new SendableChooser<>();
  private SendableChooser<Boolean> m_condition3 = new SendableChooser<>();
  private SendableChooser<Boolean> m_condition4 = new SendableChooser<>();
  private SendableChooser<Integer> m_positionChoose = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSub, m_driveCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_colorSub, m_colorCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_shootingSub, m_shootCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_computerVisionSub, m_intakeVisionCommand);

    //UsbCamera cam = new UsbCamera("cam", 0);
    
    
    m_condition1.setDefaultOption("Exclude Ball 1", false);
    m_condition1.addOption("Include Ball 1", true);

    m_condition2.setDefaultOption("Exclude Ball 2", false);
    m_condition2.addOption("Include Ball 2", true);

    m_condition3.setDefaultOption("Exclude Ball 3", false);
    m_condition3.addOption("Include Ball 3", true);

    m_condition4.setDefaultOption("Exclude Ball 4", false);
    m_condition4.addOption("Include Ball 4", true);

    m_positionChoose.setDefaultOption("Position1", 1);
    m_positionChoose.addOption("Position2", 2);
    m_positionChoose.addOption("Position3", 3);

    SmartDashboard.putData(m_condition1);
    SmartDashboard.putData(m_condition2);
    SmartDashboard.putData(m_condition3);
    SmartDashboard.putData(m_condition4);
    SmartDashboard.putData(m_positionChoose);
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
    final JoystickButton aimButton;
    final JoystickButton zeroButton;
    final JoystickButton intakeButton;
    final JoystickButton intakeReverseButton;
    final JoystickButton fixedShootButton;

    climbButton = new JoystickButton(m_driverOne, 2); // X button
    frickButton = new JoystickButton(m_driverOne, 9); // Share button
    aimButton = new JoystickButton(m_driverOne, 6); // Right Bumper
    zeroButton = new JoystickButton(m_driverOne, 3); // Circle
    intakeButton = new JoystickButton(m_driverOne, 4); // Square Button
    intakeReverseButton = new JoystickButton(m_driverOne, 5); // Left Bumper
    fixedShootButton = new JoystickButton(m_driverOne, 8);

    climbButton.toggleWhenPressed(m_climbingCommand);
    frickButton.toggleWhenPressed(m_jackFrickedUpCommand);
    aimButton.whileHeld(m_shootCommand);
    fixedShootButton.toggleWhenPressed(m_fixedShootCommand);
    zeroButton.toggleWhenPressed(m_zeroShooterCommand);
    intakeButton.toggleWhenPressed(m_IntakeCommand);
    intakeReverseButton.toggleWhenPressed(m_IntakeReverseCommand);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //MultiballAutoCommand m_multiballAutoCommand = new MultiballAutoCommand(m_drivetrainSub, m_shootingSub, m_intakeSub, m_positionChoose.getSelected(), m_condition1.getSelected(), m_condition2.getSelected(), m_condition3.getSelected(), m_condition4.getSelected());
    return m_multiballAutoCommand;
  }

  public void resetDashboard() {
    // Climb display
    SmartDashboard.putString("Climb Stage:", "NA");
    SmartDashboard.putString("Climb Target:", "NA");
    SmartDashboard.putString("Climb Resetting?", "No");
    SmartDashboard.putString("Left Arm Power:", "NA");
    SmartDashboard.putString("Right Arm Power:", "NA");

    // Shooter display
    SmartDashboard.putString("Shooter Status:", "NA");
    SmartDashboard.putString("Shooter Power:", "NA");
    SmartDashboard.putString("Target Elevation Angle:", "NA");
    SmartDashboard.putString("Current Elevation Angle:", "NA");

    // Intake display
    SmartDashboard.putString("Intake Piston Status:", "NA");
    SmartDashboard.putString("Intake Motor Status:", "NA");

    // Feed display
    SmartDashboard.putString("Feeder Status:", "NA");

    // Color display
    SmartDashboard.putString("Color Match:", "NA");
    SmartDashboard.putString("RED:", "NA");
    SmartDashboard.putString("GREEN:", "NA");
    SmartDashboard.putString("BLUE:", "NA");

    // Nav + Auto display
    SmartDashboard.putString("Robot X:", "NA");
    SmartDashboard.putString("Robot Y:", "NA");
    SmartDashboard.putString("Robot Heading:", "NA");
    SmartDashboard.putString("Heading Error:", "NA");
    SmartDashboard.putString("Distance to Waypoint:", "NA");
    SmartDashboard.putString("Auto Throttle:", "NA");
    SmartDashboard.putString("Auto Steering:", "NA");
    
  }

}
