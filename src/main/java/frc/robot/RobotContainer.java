// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.ShiftStyle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;

import static java.lang.Math.*;

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
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerInput);
  RunCommand tunePIDCommand = new RunCommand(() -> m_robotDrive.tunePID(), m_robotDrive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    m_robotDrive.setDefaultCommand(
      new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driveController.getLeftY(), m_driveController.getRightX()),
            m_robotDrive)
    );

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kA.value)
    .whenPressed(() -> m_robotDrive.setShiftStyle(ShiftStyle.STATIC_LOW));
    new JoystickButton(m_driveController, XboxController.Button.kB.value)
    .whenPressed(() -> m_robotDrive.setShiftStyle(ShiftStyle.STATIC_HIGH));
    new JoystickButton(m_driveController, XboxController.Button.kY.value)
    .whenPressed(() -> m_robotDrive.setShiftStyle(ShiftStyle.STATIC_NEUTRAL));
    new JoystickButton(m_driveController, XboxController.Button.kX.value)
    .whenPressed(() -> m_robotDrive.setShiftStyle(ShiftStyle.AUTO));
    new JoystickButton(m_driveController, XboxController.Button.kStart.value)
    .toggleWhenPressed(tunePIDCommand);
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
