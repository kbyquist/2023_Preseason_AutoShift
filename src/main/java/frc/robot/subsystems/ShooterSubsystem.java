// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax m_shootLead = new CANSparkMax(kShootMotorID[1], MotorType.kBrushless);

  public ShooterSubsystem() {
    m_shootLead.restoreFactoryDefaults();
    m_shootLead.setIdleMode(IdleMode.kCoast);
  }

  public void shootShoot() {
    m_shootLead.set(.25);
  }

  public void shootIdle() {
    m_shootLead.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
