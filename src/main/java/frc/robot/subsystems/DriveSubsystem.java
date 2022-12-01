// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShiftConstants.*;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

import static java.lang.Math.*;


public class DriveSubsystem extends SubsystemBase {

  enum ShiftingState{
    LOW,
    HIGH,
    NEUTRAL
  }
  ShiftingState shiftingStateCurrent;
  ShiftingState shiftingStateDesired;

  public enum ShiftStyle {
    AUTO,
    STATIC_LOW,
    STATIC_HIGH,
    STATIC_NEUTRAL
  }
  ShiftStyle shiftStyle;

  enum TurningState{
    TURNING,
    DRIVINGSTRAIGHT
  }
  TurningState turningState;

  /* Variables */
  double shiftRollingRPM;
  double driveSign;
  double lastFwd = 0.;
  double kSlewRateDrive = 1;

  /* Timers */
  private final Timer m_shiftDwellTimer = new Timer();
  private final Timer m_neutralEngagedTimer = new Timer();

  /*Pnuematics*/
  private final DoubleSolenoid p_ShiftA =
    new DoubleSolenoid(
      kPcmCanID,
      PneumaticsModuleType.CTREPCM,
      kShiftSolenoidA[0],
      kShiftSolenoidA[1]
    );

  private final DoubleSolenoid p_ShiftB =
    new DoubleSolenoid(
      kPcmCanID,
      PneumaticsModuleType.CTREPCM,
      kShiftSolenoidB[0],
      kShiftSolenoidB[1]
    );
  
  /* Define Motors in DriveSubsystem */
  //LEFT MOTOR
  private final CANSparkMax m_leftLead = new CANSparkMax(kLeftDriveMotors[0], MotorType.kBrushless);    
  //MIDDLE MOTOR
  private final CANSparkMax m_leftFollow1 = new CANSparkMax(kLeftDriveMotors[1], MotorType.kBrushless);
  //RIGHT MOTOR
  private final CANSparkMax m_leftFollow2 = new CANSparkMax(kLeftDriveMotors[2], MotorType.kBrushless);

  /* PID Controllers */
  private final SparkMaxPIDController m_leftPIDController = m_leftLead.getPIDController();

  /* Encoders */
  private final RelativeEncoder m_leftMotorEncoder = m_leftLead.getEncoder();

  private final Encoder m_leftWheelEncoder = 
      new Encoder(
        kLeftWheelEncoderPorts[0],
        kLeftWheelEncoderPorts[1],
        kLeftWheelEncoderReversed,
        EncodingType.k1X
      );

  // Creates a SlewRateLimiter
  SlewRateLimiter driveFilter;

  /** Creates a new Drivetrain. Initialize hardware here */
  public DriveSubsystem() {

    /* Restore Defaults of Motors. 
    Doing this confirms the settings will be the same no matter what physical controller is used  */
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow1.restoreFactoryDefaults();
    m_leftFollow2.restoreFactoryDefaults();
    
    /* Set Idle Mode */
    m_leftLead.setIdleMode(IdleMode.kCoast);
    m_leftFollow1.setIdleMode(IdleMode.kCoast);
    m_leftFollow2.setIdleMode(IdleMode.kCoast);

    /* Create Follow Groups */
    m_leftFollow1.follow(m_leftLead, true); //Inverted due to gearbox geometry
    m_leftFollow2.follow(m_leftLead, false); //Not inverted

    /* Set PID constants */
    m_leftPIDController.setP(kP);
    m_leftPIDController.setFF(kFF);

    /* Set Motor and Encoder Inversions */
    m_leftLead.setInverted(kLeftMotorInverted); //only need to set Lead motors

    /* Encoder Conversion */
    m_leftWheelEncoder.setDistancePerPulse(kEncoderDistancePerPulse); //RPM
    m_leftWheelEncoder.setSamplesToAverage(10);

    driveFilter = new SlewRateLimiter(kSlewRateDrive);

    timerRestart(m_shiftDwellTimer);
    setDesiredShiftState(ShiftingState.LOW);
    setShiftStyle(ShiftStyle.AUTO);
    lowGear(); //Set the default for robot init
  
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    if (getCurrentShiftState() == getDesiredShiftState()){m_leftLead.set(driveFilter.calculate(fwd));;}
    setTurningState(rot);
    switch (getShiftStyle()) {
      case AUTO:
        determineShiftState(fwd, rot);
        if (getCurrentShiftState() != getDesiredShiftState()) {
          shiftSmooth();
        }
        break;
      case STATIC_HIGH:
        if (getCurrentShiftState() != ShiftingState.HIGH) {
          setDesiredShiftState(ShiftingState.HIGH);
          highGear();
        }
        break;
      case STATIC_LOW:
        if (getCurrentShiftState() != ShiftingState.LOW) {
          setDesiredShiftState(ShiftingState.LOW);
          lowGear();
        } 
        break;
      case STATIC_NEUTRAL:
        if (getCurrentShiftState() != ShiftingState.NEUTRAL) {
          setDesiredShiftState(ShiftingState.NEUTRAL);
          neutralGear();
        }
        break;
    }
    lastFwd = fwd;
  }

  


  public void determineShiftState(double fwd, double rot) {
    switch (getCurrentShiftState()) {
      case LOW:
        if (getAverageMotorSpeed() > kRPMToUpshiftAt - kShiftDeadband &&
            readyUpshift(fwd) &&
            timerAboveCriteria(m_shiftDwellTimer, kShiftDwellTimer)) {
              setDesiredShiftState(ShiftingState.HIGH);
        }
        break;
      case HIGH:
        if (getAverageMotorSpeed() > kRPMToDownshiftAt - kShiftDeadband && 
            getAverageMotorSpeed() < kRPMToDownshiftAt + kShiftDeadband &&
            readyDownshift(fwd) &&
            timerAboveCriteria(m_shiftDwellTimer, kShiftDwellTimer)) {
              setDesiredShiftState(ShiftingState.LOW);
        } else if (getAverageMotorSpeed() < 100) {
          setDesiredShiftState(ShiftingState.LOW);
        }
        break;
      case NEUTRAL:
        break;
    }
  }


  public void shiftSmooth() {
    double driveMultiplier = getDriveSign();
    ShiftingState desiredState = getDesiredShiftState();
    ShiftingState currentState = getCurrentShiftState();


    if (desiredState == ShiftingState.LOW) {
      if (currentState == ShiftingState.HIGH) {
        neutralGear();
      } else if (currentState == ShiftingState.NEUTRAL) {
        if (!timerAboveCriteria(m_neutralEngagedTimer, kNeutralTime)) {
          //DO NOTHING
        } else {
          shiftRollingRPM = getAverageWheelEncoderSpeed() * kLowGearRatio * driveMultiplier;
          m_leftPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
        }
          if (getAverageMotorSpeed() > abs(shiftRollingRPM) - kShiftDeadband &&
              getAverageMotorSpeed() < abs(shiftRollingRPM) + kShiftDeadband) {
            lowGear();
          }
      }
    } else if (desiredState == ShiftingState.HIGH) {
      if (currentState == ShiftingState.LOW) {
        neutralGear();
      } else if (currentState == ShiftingState.NEUTRAL) {
        if (!timerAboveCriteria(m_neutralEngagedTimer, kNeutralTime)) {
          //Do nothing
        } else {
        shiftRollingRPM = getAverageWheelEncoderSpeed() * kHighGearRatio * driveMultiplier;
        m_leftPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
        }
        //Once within margin, set to low gear
          if (getAverageMotorSpeed() > abs(shiftRollingRPM) - kShiftDeadband &&
              getAverageMotorSpeed() < abs(shiftRollingRPM) + kShiftDeadband) {
            highGear();
          }
      }
    } else {}
  }

  /** lowGear is full retract = Air to PORT 2 on both */
  public void lowGear() {
    p_ShiftA.set(kReverse); //Air to PORT 2
    p_ShiftB.set(kReverse); //Air to PORT 2

    setCurrentShiftState(ShiftingState.LOW);
    timerRestart(m_shiftDwellTimer);
  }

  /** highGear is full extend = Air to PORT 1 on both */
  public void highGear(){
    p_ShiftA.set(kForward); //Air to PORT 1
    p_ShiftB.set(kForward); //Air to PORT 1

    setCurrentShiftState(ShiftingState.HIGH);
    timerRestart(m_shiftDwellTimer);
  }

  /** neutralGear is half extend = Air to PORT 1 on A and PORT 2 on B */
  public void neutralGear(){
    p_ShiftA.set(kForward); //Air to PORT 1
    p_ShiftB.set(kReverse); //Air to PORT 2

    setCurrentShiftState(ShiftingState.NEUTRAL);
    timerRestart(m_neutralEngagedTimer);
  }

  public double getAverageWheelEncoderSpeed() {
    return abs(m_leftWheelEncoder.getRate());
  }

  public double getAverageMotorSpeed() {
    return abs(getLeftMotorRPM());
  }

  public double getLeftMotorRPM() {
    return m_leftMotorEncoder.getVelocity();
  }

  public void timerRestart(Timer timer) {
    timer.reset();
    timer.start();
  } 

  private void setDriveSign() {
    if (getLeftMotorRPM() < 0) {
      driveSign = -1;
    } else { driveSign = 1;}
  }

  private TurningState getTurningState() { 
    return turningState; 
  }

  private void setTurningState(double RotationInput) {
    if (abs(RotationInput) < kTurnDeadband) {
      turningState = TurningState.DRIVINGSTRAIGHT;
    } else {turningState = TurningState.TURNING;}
  }

  private double getDriveSign() { 
    return driveSign; 
  }

  private void setCurrentShiftState(ShiftingState CurrentShiftState) {
    shiftingStateCurrent = CurrentShiftState;
  }

  private ShiftingState getCurrentShiftState() { 
    return shiftingStateCurrent; 
  }

  private void setDesiredShiftState(ShiftingState DesiredShiftingState) {
    shiftingStateDesired = DesiredShiftingState; 
  }

  private ShiftingState getDesiredShiftState() { 
    return shiftingStateDesired; 
  }

  public void setShiftStyle(ShiftStyle shiftingStyle) { 
    shiftStyle = shiftingStyle;
  }

  public ShiftStyle getShiftStyle() { 
    return shiftStyle; 
  }

  public Boolean readyUpshift(double fwd) {
    if ((lastFwd > 0.05 && fwd > 0.05) || (lastFwd < 0.05 && fwd <0.05)) {
      if (((abs(fwd) - abs(lastFwd)) > kUpshiftThrottleMin) || (abs(fwd) >= .75)) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  public Boolean readyDownshift(double fwd) {
    if (((abs(fwd) - abs(lastFwd) <= 0) || (abs(fwd) < 0.05)) && getAverageMotorSpeed()<kRPMToDownshiftAt && abs(fwd)<.5) {
      return true;
    } else {
        return false;
    }
  }

  public Boolean timerAboveCriteria(Timer timer, Double timer_criteria) {
    if (timer.get() > timer_criteria) {
      return true;
    } else {
      return false;
    }
  }
  
  @Override
  public void periodic() {
    setDriveSign();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
