// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;



/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class Auto1 extends SequentialCommandGroup {

  ArrayList<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Auto1", new PathConstraints(4, 3));

  /**
   * Creates a new ComplexAutoCommand.
   *
   * @param driveSubsystem The drive subsystem this command will run on
   * @param hatchSubsystem The hatch subsystem this command will run on
   */
  public Auto1(DriveSubsystem driveSubsystem) {
    
  }
}
