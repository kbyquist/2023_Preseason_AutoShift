// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShiftConstants.*;

import frc.robot.commands.Drive;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
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
  /* PID Tuning */
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /* Variables */
  boolean boolRobotTurning;
  boolean boolThrottleReadyUpshift;
  boolean boolThrottleReadyDownshift;
  boolean boolTimerShiftReady;

  double shiftRollingRPM;

  ShiftingState shiftingStateCurrent;
  ShiftingState shiftingStateDesired; //should only be high or low during auto shift

  double lastFwd;

  /* Timers */
  private final Timer m_shiftDwellTimer = new Timer();


  /**Define Pnuematics in DriveSubsystem */
  private final DoubleSolenoid p_leftShiftAB =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kLeftShifterPortA,kLeftShifterPortB);
  
  private final Solenoid p_leftShiftC = 
    new Solenoid(kPcmCanID, PneumaticsModuleType.CTREPCM, kLeftShifterPortC);

  private final DoubleSolenoid p_rightShiftAB =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kRightShifterPortA,kRightShifterPortB);
  
  private final Solenoid p_rightShiftC = 
    new Solenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kRightShifterPortC);

  /* Define Motors in DriveSubsystem */
  private final CANSparkMax m_leftLead = new CANSparkMax(kLeftLeadMotorID, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow1 = new CANSparkMax(kLeftFollowMotor1ID, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow2 = new CANSparkMax(kLeftFollowMotor2ID, MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(kRightLeadMotorID, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow1 = new CANSparkMax(kRightFollowMotor1ID, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow2 = new CANSparkMax(kRightFollowMotor2ID, MotorType.kBrushless);

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

  // PID coefficients
  public double kP = 6e-5; 
  public double kI = 0;
  public double kD = 0; 
  public double kIz = 0; 
  public double kFF = 0.000015; 
  public double kMaxOutput = 1; 
  public double kMinOutput = -1;

  // Creates a SlewRateLimiter that limits the rate of change of the signal to defined constant units per second
  public double kSlewRateDrive = 3.5;
  public double kSlewRateRotate = 3.5;
  SlewRateLimiter rotateFilter;
  SlewRateLimiter driveFilter;


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

    /** Create Follow Groups */
    //Left
    m_leftFollow1.follow(m_leftLead);
    m_leftFollow2.follow(m_leftLead);
    //Right
    m_rightFollow1.follow(m_rightLead);
    m_rightFollow2.follow(m_rightLead);

    /** Set PID constants */
    m_leftPIDController.setP(kP);
    m_leftPIDController.setI(kI);
    m_leftPIDController.setD(kD);
    m_leftPIDController.setIZone(kIz);
    m_leftPIDController.setFF(kFF);
    m_leftPIDController.setOutputRange(kMinOutput, kMaxOutput);

    m_rightPIDController.setP(kP);
    m_rightPIDController.setI(kI);
    m_rightPIDController.setD(kD);
    m_rightPIDController.setIZone(kIz);
    m_rightPIDController.setFF(kFF);
    m_rightPIDController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    /** Set Motor Inversions */
    //Right is inverted, left is not
    m_rightLead.setInverted(true); //only need to set Lead motors
    m_leftLead.setInverted(false); //only need to set Lead motors

    /** Encoder Conversion */
    m_leftWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);
    m_rightWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);

    timerRestart(m_shiftDwellTimer);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // Put Slew Rates on SmartDashboard for fine tuning
    SmartDashboard.putNumber("Acceleration Rate", kSlewRateDrive);
    SmartDashboard.putNumber("Rotation Rate", kSlewRateRotate);
    // Get updated numbers for slew rates
    double SlewRateDrive = SmartDashboard.getNumber("Acceleration Rate", 6);
    double SlewRateRotate = SmartDashboard.getNumber("Rotation Rate", 6);
    // Compare current vs new slew rates
    if((SlewRateDrive != kSlewRateDrive)) {kSlewRateDrive = SlewRateDrive;}
    if((SlewRateRotate != kSlewRateRotate)) {kSlewRateRotate = SlewRateRotate;}
    // Update slew rate limiters
    driveFilter = new SlewRateLimiter(kSlewRateDrive);
    rotateFilter = new SlewRateLimiter(kSlewRateRotate);

    m_drive.arcadeDrive(driveFilter.calculate(fwd), rotateFilter.calculate(rot));
    autoShift(fwd, rot); //do not apply slew rate as desired driver input is needed
    lastFwd = fwd;
  }

  public void autoShift(double fwd, double rot) {

    //See if robot is turning
    if (abs(rot) < kTurnDeadband) {
      boolRobotTurning = false;
    } else {boolRobotTurning = true;}

    //Check if driver wants to speed up or slow down
    if ((lastFwd > 0 && fwd > 0) || (lastFwd < 0 && fwd <0)) {
      if ((abs(fwd) - abs(lastFwd) > kUpshiftThrottleMin) || (abs(fwd) >= .95)) {
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
    switch (shiftingStateCurrent) {
      case LOW:
        if (getAverageMotorSpeed() > kRPMToUpshiftAt - kShiftDeadband && 
            getAverageMotorSpeed() < kRPMToUpshiftAt + kShiftDeadband &&
            !boolRobotTurning &&
            boolThrottleReadyUpshift &&
            boolTimerShiftReady) {
              setDesiredShiftState(ShiftingState.HIGH);
        }
        break;
      case HIGH:
        if (getAverageMotorSpeed() > kRPMToDownshiftAt - kShiftDeadband && 
            getAverageMotorSpeed() < kRPMToDownshiftAt + kShiftDeadband &&
            !boolRobotTurning &&
            boolThrottleReadyDownshift &&
            boolTimerShiftReady) {
              setDesiredShiftState(ShiftingState.LOW);
        }
        break;
      default:
        break;
    }

    setShiftState();
  }

  private ShiftingState setCurrentShiftState(ShiftingState CurrentShiftState) {
    shiftingStateCurrent = CurrentShiftState;
  }

  private ShiftingState getCurrentShiftState() {
    return shiftingStateCurrent;
  }

  private ShiftingState setDesiredShiftState(ShiftingState DesiredShiftingState) {
    shiftingStateDesired = DesiredShiftingState;
  }

  private ShiftingState getDesiredShiftState() {
    return shiftingStateDesired;
  }

  public void setShiftState() {
    switch (getDesiredShifttState) {
      case LOW: //Upshifting
        switch (getCurrentShiftState) {
          case HIGH:
            neutralGear();
            m_leftPIDController.setReference(kRPMDownshiftSetPoint, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
            m_rightPIDController.setReference(kRPMDownshiftSetPoint, CANSparkMax.ControlType.kVelocity);
            break;
          case NEUTRAL:
            //keep setting PID controller until RPM is within margin of wheel speed * reductions
              shiftRollingRPM = getAverageWheelEncoderSpeed()*kHighGearRatio;
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
      case HIGH: //Downshifting
        switch (shiftingStateCurrent) {
          case LOW:
            neutralGear();
            m_leftPIDController.setReference(kRPMUpshiftSetPoint, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
            m_rightPIDController.setReference(kRPMUpshiftSetPoint, CANSparkMax.ControlType.kVelocity);
            break;
            case NEUTRAL:
            //keep setting PID controller until RPM is within margin of wheel speed * reductions
              shiftRollingRPM = getAverageWheelEncoderSpeed()*kLowGearRatio;
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
      case NEUTRAL: //Should only be if using PTO and don't want drive
        if (shiftingStateCurrent != ShiftingState.NEUTRAL) {
          neutralGear();
        }
        break;
    }
  }



  /** lowGear is full retract = Air to PORT C */
  public void lowGear() {
    p_leftShiftAB.set(kOff); //kOff = Air is off to PORT A, off to PORT B
    p_leftShiftC.set(true); //turn on PORT C air

    p_rightShiftAB.set(kOff); //kOff = Air is off to PORT A, off to PORT B
    p_rightShiftC.set(true); //turn on PORT C air
    setCurrentShiftState(ShiftingState.LOW);
    timerRestart(m_shiftDwellTimer);
  }

  /** highGear is full extend = Air to PORT B */
  public void highGear(){
    p_leftShiftAB.set(kReverse); //kReverse = Air is on to PORT B, off to PORT A
    p_leftShiftC.set(false); //turn off PORT C air

    p_rightShiftAB.set(kReverse); //kReverse = Air is on to PORT B, off to PORT A
    p_rightShiftC.set(false); //turn off PORT C air
    setCurrentShiftState(ShiftingState.HIGH);
    timerRestart(m_shiftDwellTimer);
  }

  /** neutralGear is half extend = Air to PORT A */
  public void neutralGear(){
    p_leftShiftAB.set(kForward); //kForward = Air is on to PORT A, off to PORT B
    p_leftShiftC.set(false); //turn off PORT C air

    p_rightShiftAB.set(kForward); //kForward = Air is on to A, off to B
    p_rightShiftC.set(false); //turn off PORT C air
    setCurrentShiftState(ShiftingState.NEUTRAL);
    timerRestart(m_shiftDwellTimer);
  }

  /**
   * Gets the average absolute speed of the left and right set of wheel encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageWheelEncoderSpeed() {
    return (abs(m_leftWheelEncoder.getRate()) + abs(m_rightWheelEncoder.getRate())) / 2.0;
  }

  /**
   * Gets the average absolute rpm of the left and right transmission motors
   * 
   * @return the average RPMs of the motor encoders
   */
  public double getAverageMotorSpeed() {
    return (abs(getLeftMotorRPM()) + abs(getRightMotorRPM()))/2.0;
  }

  /**
   * Gets the left drive motor RPM
   * 
   * @return the left transmission motors RPM
   */
  public double getLeftMotorRPM() {
    return m_leftMotorEncoder.getVelocity();
  }

  /**
   * Gets the right drive motor RPM
   * 
   * @return the right transmission motors RPM
   */
  public double getRightMotorRPM() {
    return m_rightMotorEncoder.getVelocity();
  }

  public void timerRestart(Timer timer) {
    timer.reset();
    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
