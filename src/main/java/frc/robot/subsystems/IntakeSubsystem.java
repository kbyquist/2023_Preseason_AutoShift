// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_intakeSolenoid = 
      new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        kIntakeSolenoidID[0],
        kIntakeSolenoidID[1]
      );

  public void extendIntake() {
    m_intakeSolenoid.set(kForward);
  }

  public void retractIntake() {
    m_intakeSolenoid.set(kReverse);
  }
}
