// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShiftConstants.*;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import java.util.*;

import static java.lang.Math.*;


public class DriveSubsystem extends SubsystemBase {

  enum ShiftingState{
    LOW,
    HIGH,
    NEUTRAL
  }

  public enum ShiftStyle {
    AUTO,
    STATIC_LOW,
    STATIC_HIGH,
    STATIC_NEUTRAL
  }

  enum TurningState{
    TURNING,
    DRIVINGSTRAIGHT
  }

  /* Variables */
  boolean boolThrottleReadyUpshift;
  boolean boolThrottleReadyDownshift;
  boolean boolTimerShiftReady;

  double shiftRollingRPM;

  ShiftingState shiftingStateCurrent;
  ShiftingState shiftingStateDesired; //should only be high or low during auto shift
  ShiftStyle shiftStyle = ShiftStyle.AUTO;

  TurningState turningState;

  double driveSign;

  double lastFwd = 0.;

  /* Timers */
  private final Timer m_shiftDwellTimer = new Timer();

  /**Define Pnuematics in DriveSubsystem */
  private final DoubleSolenoid p_ShiftA =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kShiftSolenoidA[0],kShiftSolenoidA[1]);

  private final DoubleSolenoid p_ShiftB =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kShiftSolenoidB[0],kShiftSolenoidB[1]);
  
  /* Define Motors in DriveSubsystem */
  private final CANSparkMax m_leftLead = new CANSparkMax(kLeftDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_leftFollow1 = new CANSparkMax(kLeftDriveMotors[1], MotorType.kBrushless);
  private final CANSparkMax m_leftFollow2 = new CANSparkMax(kLeftDriveMotors[2], MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(kRightDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_rightFollow1 = new CANSparkMax(kRightDriveMotors[1], MotorType.kBrushless);
  private final CANSparkMax m_rightFollow2 = new CANSparkMax(kRightDriveMotors[2], MotorType.kBrushless);

  /* PID Controllers */
  private final SparkMaxPIDController m_leftPIDController = m_leftLead.getPIDController();
  private final SparkMaxPIDController m_rightPIDController = m_rightLead.getPIDController();

  /* Encoders */
  private final RelativeEncoder m_leftMotorEncoder = m_leftLead.getEncoder();
  private final RelativeEncoder m_rightMotorEncoder = m_rightLead.getEncoder();

  private final Encoder m_leftWheelEncoder = 
      new Encoder(
        kLeftWheelEncoderPorts[0],
        kLeftWheelEncoderPorts[1],
        kLeftWheelEncoderReversed,
        EncodingType.k1X);

  private final Encoder m_rightWheelEncoder = 
      new Encoder(
        kRightWheelEncoderPorts[0],
        kRightWheelEncoderPorts[1],
        kRightWheelEncoderReversed,
        EncodingType.k1X);

  //Robot Drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLead, m_rightLead);

  // Creates a SlewRateLimiter that limits the rate of change of the signal to defined constant units per second
  public double kSlewRateDrive = 3.5;
  public double kSlewRateRotate = 3.5;
  SlewRateLimiter rotateFilter;
  SlewRateLimiter driveFilter;

  /** Shuffleboard Setup */
  private ShuffleboardTab tabDrive = Shuffleboard.getTab(kDriveTabName);
  private NetworkTableEntry slewRateDrive = tabDrive.add("Drive Slew Rate", kSlewRateDrive).getEntry();
  private NetworkTableEntry slewRateTurn = tabDrive.add("Turn Slew Rate", kSlewRateRotate).getEntry();

  /** Creates a new Drivetrain. Initialize hardware here */
  public DriveSubsystem() {
    /* Restore Defaults of Motors. 
    Doing this confirms the settings will be the same no matter what physical controller is used  */
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow1.restoreFactoryDefaults();
    m_leftFollow2.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();
    m_rightFollow1.restoreFactoryDefaults();
    m_rightFollow2.restoreFactoryDefaults();

    /* Create Follow Groups */
    //Left
    m_leftFollow1.follow(m_leftLead);
    m_leftFollow2.follow(m_leftLead);
    //Right
    m_rightFollow1.follow(m_rightLead);
    m_rightFollow2.follow(m_rightLead);

    /* Set PID constants */
    setPIDController(m_leftPIDController);
    setPIDController(m_rightPIDController);

    /* Set Motor and Encoder Inversions */
    m_rightLead.setInverted(kRightMotorInverted); //only need to set Lead motors
    m_rightMotorEncoder.setInverted(kRightMotorInverted);
    m_leftLead.setInverted(kLeftMotorInverted); //only need to set Lead motors
    m_leftMotorEncoder.setInverted(kLeftMotorInverted);

    /* Encoder Conversion */
    m_leftWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);
    m_rightWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);

    timerRestart(m_shiftDwellTimer);
    driveFilter = new SlewRateLimiter(kSlewRateDrive);
    rotateFilter = new SlewRateLimiter(kSlewRateRotate);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(driveFilter.calculate(fwd), rotateFilter.calculate(rot));
    setTurningState(rot);
    switch (getShiftStyle()) {
      case AUTO:
        determineShiftState(fwd, rot);
        break;
      case STATIC_HIGH:
        if (getCurrentShiftState() != getDesiredShiftState()) {
          setDesiredShiftState(ShiftingState.HIGH);
          highGear(); //abrupt shifting to get to gear immediately
        }
        break;
      case STATIC_LOW:
        if (getCurrentShiftState() != getDesiredShiftState()) {
          setDesiredShiftState(ShiftingState.LOW);
          lowGear(); //abrupt shifting to get to gear immediately
        } 
        break;
      case STATIC_NEUTRAL:
        if (getCurrentShiftState() != getDesiredShiftState()) {
          setDesiredShiftState(ShiftingState.NEUTRAL);
          neutralGear();
        }
        break;
    }
    lastFwd = fwd;
  }

  public void determineShiftState(double fwd, double rot) {

    //Check if driver wants to speed up or slow down
    if ((lastFwd > 0 && fwd > 0) || (lastFwd < 0 && fwd <0)) {
      if (((abs(fwd) - abs(lastFwd)) > kUpshiftThrottleMin) || (abs(fwd) >= .95)) {
        boolThrottleReadyUpshift = true;
      }
      if (abs(fwd) - abs(lastFwd) < 0 || (abs(fwd) < 0.05)) {
        boolThrottleReadyDownshift = true;
      }
    } else {boolThrottleReadyUpshift = false; boolThrottleReadyDownshift = false;}

    //Check dwell timer against constant
    if (m_shiftDwellTimer.get() > kShiftDwellTimer) {
      boolTimerShiftReady = true;
    } else {boolTimerShiftReady = false;}

    //Logic to set desired shifting state
    switch (getCurrentShiftState()) {
      case LOW:
        if (getAverageMotorSpeed() > kRPMToUpshiftAt - kShiftDeadband && 
            getAverageMotorSpeed() < kRPMToUpshiftAt + kShiftDeadband &&
            getTurningState() == TurningState.DRIVINGSTRAIGHT &&
            boolThrottleReadyUpshift &&
            boolTimerShiftReady) {
              setDesiredShiftState(ShiftingState.HIGH);
        }
        break;
      case HIGH:
        if (getAverageMotorSpeed() > kRPMToDownshiftAt - kShiftDeadband && 
            getAverageMotorSpeed() < kRPMToDownshiftAt + kShiftDeadband &&
            getTurningState() == TurningState.DRIVINGSTRAIGHT &&
            boolThrottleReadyDownshift &&
            boolTimerShiftReady) {
              setDesiredShiftState(ShiftingState.LOW);
        }
        break;
      default:
        break;
    }

    if (getCurrentShiftState() != getDesiredShiftState()) {
      shiftSmooth();
    }
  }



  public void shiftSmooth() {
    double driveMultiplier = getDriveSign();

    switch (getDesiredShiftState()) {
      case LOW: //Downshift
        switch (getCurrentShiftState()) {
          case HIGH:
            neutralGear();
            m_leftPIDController.setReference(kRPMDownshiftSetPoint * driveMultiplier, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
            m_rightPIDController.setReference(kRPMDownshiftSetPoint * driveMultiplier, CANSparkMax.ControlType.kVelocity);
            break;
          case NEUTRAL:
            //keep setting PID controller until RPM is within margin of wheel speed * reductions
              shiftRollingRPM = getAverageWheelEncoderSpeed() * kLowGearRatio * driveMultiplier;
              m_leftPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
              m_rightPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
            //Once within margin, set to low gear
              if (getAverageMotorSpeed() > shiftRollingRPM - kShiftDeadband &&
                  getAverageMotorSpeed() < shiftRollingRPM + kShiftDeadband) {
                lowGear();
              }
          default:
            //Do nothing
            break;
        }
        break;
      case HIGH: //Upshift
        switch (getCurrentShiftState()) {
          case LOW:
            neutralGear();
            m_leftPIDController.setReference(kRPMUpshiftSetPoint * driveMultiplier, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
            m_rightPIDController.setReference(kRPMUpshiftSetPoint * driveMultiplier, CANSparkMax.ControlType.kVelocity);
            break;
            case NEUTRAL:
            //keep setting PID controller until RPM is within margin of wheel speed * reductions
              shiftRollingRPM = getAverageWheelEncoderSpeed() * kHighGearRatio * driveMultiplier;
              m_leftPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
              m_rightPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
            //Once within margin, set to low gear
              if (getAverageMotorSpeed() > shiftRollingRPM - kShiftDeadband &&
                  getAverageMotorSpeed() < shiftRollingRPM + kShiftDeadband) {
                highGear();
              }
          default:
            //Do nothing
            break;
        }
        break;
      default:
        break;
    }
  }

  /** lowGear is full retract = Air to PORT 2 on both */
  private void lowGear() {
    p_ShiftA.set(kReverse); //Air to PORT 2
    p_ShiftB.set(kReverse); //Air to PORT 2

    setCurrentShiftState(ShiftingState.LOW);
    timerRestart(m_shiftDwellTimer);
  }

  /** highGear is full extend = Air to PORT 1 on both */
  private void highGear(){
    p_ShiftA.set(kForward); //Air to PORT 1
    p_ShiftB.set(kForward); //Air to PORT 1

    setCurrentShiftState(ShiftingState.HIGH);
    timerRestart(m_shiftDwellTimer);
  }

  /** neutralGear is half extend = Air to PORT 1 on A and PORT 2 on B */
  private void neutralGear(){
    p_ShiftA.set(kForward); //Air to PORT 1
    p_ShiftB.set(kReverse); //Air to PORT 2

    setCurrentShiftState(ShiftingState.NEUTRAL);
    //timerRestart(m_shiftDwellTimer); //Do not reset timer as shift sequence is not complete
  }

  public double getAverageWheelEncoderSpeed() {
    return (abs(m_leftWheelEncoder.getRate()) + abs(m_rightWheelEncoder.getRate())) / 2.0;
  }

  public double getAverageMotorSpeed() {
    return (abs(getLeftMotorRPM()) + abs(getRightMotorRPM()))/2.0;
  }

  public double getLeftMotorRPM() {
    return m_leftMotorEncoder.getVelocity();
  }

  public double getRightMotorRPM() {
    return m_rightMotorEncoder.getVelocity();
  }

  public void timerRestart(Timer timer) {
    timer.reset();
    timer.start();
  } 

  /**
   * Sets P, I, D, Iz, FF, MinOutput, MaxOutput when defined as kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput in the subsystem
   * 
   * @param sparkmaxpidcontroller
   */
  public void setPIDController(SparkMaxPIDController sparkmaxpidcontroller) {
    sparkmaxpidcontroller.setP(kP);
    sparkmaxpidcontroller.setI(kI);
    sparkmaxpidcontroller.setD(kD);
    sparkmaxpidcontroller.setIZone(kIz);
    sparkmaxpidcontroller.setFF(kFF);
    sparkmaxpidcontroller.setOutputRange(kMinOutput, kMaxOutput);
  }

  private void setDriveSign() {
    if (getLeftMotorRPM() < 0) {
      driveSign = -1;
    } else { driveSign = 1;}
  }

  private TurningState getTurningState() { return turningState; }

  private void setTurningState(double RotationInput) {
    if (abs(RotationInput) < kTurnDeadband) {
      turningState = TurningState.DRIVINGSTRAIGHT;
    } else {turningState = TurningState.TURNING;}
  }

  private double getDriveSign() { return driveSign; }

  private void setCurrentShiftState(ShiftingState CurrentShiftState) {shiftingStateCurrent = CurrentShiftState;}

  private ShiftingState getCurrentShiftState() { return shiftingStateCurrent; }

  private void setDesiredShiftState(ShiftingState DesiredShiftingState) { shiftingStateDesired = DesiredShiftingState; }

  private ShiftingState getDesiredShiftState() { return shiftingStateDesired; }

  public void setShiftStyle(ShiftStyle shiftingStyle) { shiftStyle = shiftingStyle; }

  public ShiftStyle getShiftStyle() { return shiftStyle; }
  
  @Override
  public void periodic() {
    setDriveSign();

    // This method will be called once per scheduler run
    // Get updated numbers for slew rates
    double SlewRateDrive = slewRateDrive.getDouble(6.);
    double SlewRateRotate = slewRateTurn.getDouble(6.);
    // Compare current vs new slew rates
    if((SlewRateDrive != kSlewRateDrive)) {kSlewRateDrive = SlewRateDrive;}
    if((SlewRateRotate != kSlewRateRotate)) {kSlewRateRotate = SlewRateRotate;}
    // Update slew rate limiters
    driveFilter.reset(kSlewRateDrive);
    //driveFilter = new SlewRateLimiter(kSlewRateDrive); //Use this if above does not work
    rotateFilter.reset(kSlewRateRotate);
    //rotateFilter = new SlewRateLimiter(kSlewRateRotate); //Use this if above does not work
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
