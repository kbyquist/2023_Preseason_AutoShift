// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;



public class Auto1Command extends SequentialCommandGroup {

  ArrayList<PathPlannerTrajectory> pathGroup1 = 
              PathPlanner.loadPathGroup(
                "Auto1",
                new PathConstraints(
                  kMaxSpeedMetersPerSecond,
                  kMaxAccelerationMetersPerSecondSquared
                )
              );

  /**
   * Creates a new Auto1Command.
   *
   * @param driveSubsystem The drive subsystem this command will run on
   * @param intakeSubsystem The hatch subsystem this command will run on
   */
  public Auto1Command(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      driveSubsystem.followTrajectoryCommand(pathGroup1.get(0), true),
      new ParallelRaceGroup(
        new WaitCommand(pathGroup1.get(0).getEndWaitTimeSeconds()),
        new RunCommand(() -> shooterSubsystem.shoot(), shooterSubsystem)
      ),
      driveSubsystem.followTrajectoryCommand(pathGroup1.get(1), false)
    );
    
  }
}
