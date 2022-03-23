// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
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

  private final ClimbingSub m_climbingSub = new ClimbingSub();
  //private final IntakeSub m_intakeSub = new IntakeSub();
  private final DriveTrainSub m_driveTrainSub = new DriveTrainSub();
  // private final ColorSub m_colorSub = new ColorSub();
  private final ShooterSub m_ShooterSub = new ShooterSub();
  private final StorageSub m_StorageSub = new StorageSub();
  private final FeedSub m_feedSub = new FeedSub();
  private final ElevationAngleSub m_ElevationAngleSub = new ElevationAngleSub();
  private final DriveTrainNavigationSub m_navSub = new DriveTrainNavigationSub(); 
  
  private final JackFrickedUpCommand m_jackFrickedUpCommand = new JackFrickedUpCommand(m_climbingSub);
  private final ClimbingCommand m_climbingCommand = new ClimbingCommand(m_climbingSub);

  private final DriveCommand m_driveCommand =  new DriveCommand(m_driveTrainSub, m_driverOne);
  private final ConstantShootCommand m_ConstantShootCommand = new ConstantShootCommand(m_ShooterSub);
  private final FeedCommand m_FeedCommand = new FeedCommand(m_feedSub);
  private final ElevationAngleCommand m_ElevationAngleCommand = new ElevationAngleCommand(m_ElevationAngleSub, m_driverOne);
  private final StorageCommand m_StorageCommand = new StorageCommand(m_StorageSub);
  //private final DriveTrainInegrationCommand m_dtIntegration = new DriveTrainInegrationCommand(m_navSub);
  //private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_intakeSub);
  // private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSub);
  // private final ColorCommand m_colorCommand = new ColorCommand(m_colorSub);

  private final TestAutoCommand m_testAuto = new TestAutoCommand(m_navSub);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    CommandScheduler.getInstance().setDefaultCommand(m_driveTrainSub, m_driveCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_navSub, m_dtIntegration);
    //CommandScheduler.getInstance().setDefaultCommand(m_ElevationAngleSub, m_ElevationAngleCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_computerVisionSub, m_intakeVisionCommand);

    // Adds all auto options to the selector
    m_autoChooser.setDefaultOption("Test Auto", m_testAuto);
    //m_autoChooser.addOption("Auto 1", m_testAuto);

    // Displays the auto selector on the dashboard
    SmartDashboard.putData(m_autoChooser);

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
    final JoystickButton storageButton;

    climbButton = new JoystickButton(m_driverOne, 2); // X button
    frickButton = new JoystickButton(m_driverOne, 9); // Share button
    feedButton = new JoystickButton(m_driverOne, 5); // Left bumper
    shootButton = new JoystickButton(m_driverOne, 5); // Right bumper
    storageButton = new JoystickButton(m_driverOne, 3); // Circle button

    climbButton.toggleWhenPressed(m_climbingCommand);
    frickButton.toggleWhenPressed(m_jackFrickedUpCommand);
    feedButton.toggleWhenPressed(m_ConstantShootCommand);
    shootButton.toggleWhenPressed(m_FeedCommand);
    storageButton.toggleWhenPressed(m_StorageCommand);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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
    SmartDashboard.putString("Shooter Left Power:", "NA");
    SmartDashboard.putString("Shooter Right Power:", "NA");
    SmartDashboard.putString("Target Elevation Angle:", "NA");
    SmartDashboard.putString("Current Elevation Angle:", "NA");
    SmartDashboard.putString("Elevation Angle Motor Power:", "NA");

    // Intake display
    SmartDashboard.putString("Intake Piston Status:", "NA");
    SmartDashboard.putString("Intake Motor Status:", "NA");

    // Feed display
    SmartDashboard.putString("Feeder Status:", "NA");
    
    // Storage display
    SmartDashboard.putString("Storage Status:", "NA");

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
