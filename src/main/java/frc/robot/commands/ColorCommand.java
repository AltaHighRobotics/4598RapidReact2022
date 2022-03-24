// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSub;;

public class ColorCommand extends CommandBase {
  /** Creates a new ColorCommand. */
  private ColorSub m_colorSub;

  SendableChooser<String> m_allianceChooser = new SendableChooser<>();

  public ColorCommand(ColorSub colorSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_colorSub = colorSub;
    addRequirements(m_colorSub);

    // Adds both alliance options to the selector
    m_allianceChooser.setDefaultOption("Red Alliance", "Red Alliance");
    m_allianceChooser.addOption("Blue Alliance", "Blue Alliance");

    // Displays the alliance selector on the dashboard
    SmartDashboard.putData(m_allianceChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_colorSub.getColor();
    m_colorSub.matchColorToAlliance(m_allianceChooser.getSelected());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Use this to get the selected alliance
   *
   * @return the selected alliance
   */
  public String getAlliance() {
    return m_allianceChooser.getSelected();
  }
}
