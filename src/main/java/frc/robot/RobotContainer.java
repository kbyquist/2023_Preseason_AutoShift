// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.ShiftStyle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  XboxController m_driveController = new XboxController(OIConstants.kDriveControllerInput);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
      new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driveController.getLeftY(), m_driveController.getRightX()),
            m_robotDrive)
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
    .whenPressed(new InstantCommand(() -> m_robotDrive.setShiftStyle(ShiftStyle.STATIC_HIGH), m_robotDrive))
    .whenReleased(new InstantCommand(() -> m_robotDrive.setShiftStyle(ShiftStyle.AUTO), m_robotDrive));
    new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value)
    .whenPressed(new InstantCommand(() -> m_robotDrive.setShiftStyle(ShiftStyle.STATIC_LOW), m_robotDrive))
    .whenReleased(new InstantCommand(() -> m_robotDrive.setShiftStyle(ShiftStyle.AUTO), m_robotDrive));
    new JoystickButton(m_driveController, XboxController.Button.kStart.value)
    .whenPressed(new InstantCommand(() -> m_robotDrive.setShiftStyle(ShiftStyle.STATIC_NEUTRAL), m_robotDrive))
    .whenReleased(new InstantCommand(() -> m_robotDrive.setShiftStyle(ShiftStyle.AUTO), m_robotDrive));
  }
}
