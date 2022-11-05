// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto1Command;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import java.util.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  public static ShooterSubsystem m_robotShooter = new ShooterSubsystem(); 
  private static final Command m_auto1 = new Auto1Command(m_robotDrive, m_robotIntake, m_robotShooter);

  private static final PS4Controller m_driveController = new PS4Controller(OIConstants.kDriveControllerInput);
  private static final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerInput);

  // A chooser for autonomous commands
  private static final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public HashMap<String, Command> eventMap = new HashMap<String, Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
            new RunCommand(() -> 
            m_robotDrive.arcadeDrive(
              -m_driveController.getLeftY(),
              m_driveController.getRightX()
              )
            ,m_robotDrive)
            );
    m_robotIntake.setDefaultCommand(new InstantCommand(m_robotIntake::retractIntake, m_robotIntake));
    m_robotShooter.setDefaultCommand(new InstantCommand(m_robotShooter::shootIdle, m_robotShooter));
    m_chooser.setDefaultOption("Auto 1", m_auto1);

    setEventMap();

  }
  
  public void setEventMap() {
      eventMap.put("intakeDeploy", new InstantCommand(m_robotIntake::extendIntake, m_robotIntake));
      eventMap.put("intakeRetract", new InstantCommand(m_robotIntake::retractIntake, m_robotIntake));
  }

  public HashMap<String, Command> getEventMap() {
      return eventMap;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
      .whenPressed(new InstantCommand(m_robotShooter::shootShoot, m_robotShooter));
  }


  
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
      return m_chooser.getSelected();
      }
}