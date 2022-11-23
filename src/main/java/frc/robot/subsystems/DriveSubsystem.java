// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShiftConstants.*;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import java.util.*;
import java.util.function.Supplier;

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
  ShiftStyle shiftStyle;

  TurningState turningState;

  double driveSign;

  double lastFwd = 0.;

  int intCount = 0;

  /* Timers */
  private final Timer m_shiftDwellTimer = new Timer();

  /**Define Pnuematics in DriveSubsystem */
  private final DoubleSolenoid p_ShiftA =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kShiftSolenoidA[0],kShiftSolenoidA[1]);

  private final DoubleSolenoid p_ShiftB =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kShiftSolenoidB[0],kShiftSolenoidB[1]);
  
  /* Define Motors in DriveSubsystem */
  private final CANSparkMax m_leftLead = new CANSparkMax(kLeftDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_leftFollow1 = new CANSparkMax(kLeftDriveMotors[1], MotorType.kBrushless); //MUST BE MIDDLE MOTOR
  private final CANSparkMax m_leftFollow2 = new CANSparkMax(kLeftDriveMotors[2], MotorType.kBrushless);
  // private final CANSparkMax m_rightLead = new CANSparkMax(kRightDriveMotors[0], MotorType.kBrushless);
  // private final CANSparkMax m_rightFollow1 = new CANSparkMax(kRightDriveMotors[1], MotorType.kBrushless); //MUST BE MIDDLE MOTOR
  // private final CANSparkMax m_rightFollow2 = new CANSparkMax(kRightDriveMotors[2], MotorType.kBrushless);

  /* PID Controllers */
  private final SparkMaxPIDController m_leftPIDController = m_leftLead.getPIDController();
  // private final SparkMaxPIDController m_rightPIDController = m_rightLead.getPIDController();

  /* Encoders */
  private final RelativeEncoder m_leftMotorEncoder = m_leftLead.getEncoder();
  // private final RelativeEncoder m_rightMotorEncoder = m_rightLead.getEncoder();

  private final Encoder m_leftWheelEncoder = 
      new Encoder(
        kLeftWheelEncoderPorts[0],
        kLeftWheelEncoderPorts[1],
        kLeftWheelEncoderReversed,
        EncodingType.k1X);

  // private final Encoder m_rightWheelEncoder = 
  //     new Encoder(
  //       kRightWheelEncoderPorts[0],
  //       kRightWheelEncoderPorts[1],
  //       kRightWheelEncoderReversed,
  //       EncodingType.k1X);

  //Robot Drive
  // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLead, m_rightLead);

  // Creates a SlewRateLimiter that limits the rate of change of the signal to defined constant units per second
  public double kSlewRateDrive = 3.5;
  SlewRateLimiter driveFilter;

  /** Shuffleboard Setup */
  private ShuffleboardTab tabDrive = Shuffleboard.getTab(kDriveTabName);
  private NetworkTableEntry slewRateDrive = tabDrive.add("Drive Slew Rate", kSlewRateDrive).getEntry();
  private NetworkTableEntry netMotorRPMEntry = tabDrive.add("Current Motor RPM", 0).getEntry();
  private NetworkTableEntry netWheelRPMEntry = tabDrive.add("Current Wheel RPM", 0).getEntry();
  private NetworkTableEntry netCurrentShiftStateEntry = tabDrive.add("Current Shift State", 1).getEntry();
  private NetworkTableEntry netDesShiftStateEntry = tabDrive.add("Desired Shift State", 1).getEntry();
  private NetworkTableEntry netDwellTimerEntry = tabDrive.add("Dwell Timer", 0).getEntry();
  private NetworkTableEntry netFwdEntry = tabDrive.add("Fwd", 0).getEntry();
  private NetworkTableEntry netLastFwdEntry = tabDrive.add("Last Fwd", 0).getEntry();
  private NetworkTableEntry netBoolThrotUpEntry = tabDrive.add("Upshift Ready", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  private NetworkTableEntry netBoolThrotDownEntry = tabDrive.add("Downshift Ready", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  private NetworkTableEntry netBoolTimerReadyEntry = tabDrive.add("Timer Ready", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  private NetworkTableEntry netSetpointEntry = tabDrive.add("Setpoint", 0).getEntry();

  /** Creates a new Drivetrain. Initialize hardware here */
  public DriveSubsystem() {

    /* Restore Defaults of Motors. 
    Doing this confirms the settings will be the same no matter what physical controller is used  */
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow1.restoreFactoryDefaults();
    m_leftFollow2.restoreFactoryDefaults();
    // m_rightLead.restoreFactoryDefaults();
    // m_rightFollow1.restoreFactoryDefaults();
    // m_rightFollow2.restoreFactoryDefaults();

    m_leftLead.setIdleMode(IdleMode.kCoast);
    m_leftFollow1.setIdleMode(IdleMode.kCoast);
    m_leftFollow2.setIdleMode(IdleMode.kCoast);

    /* Create Follow Groups */
    //Left
    m_leftFollow1.follow(m_leftLead, true); //Inverted due to gearbox geometry
    m_leftFollow2.follow(m_leftLead, false); //Not inverted
    //Right
    // m_rightFollow1.follow(m_rightLead, true);
    // m_rightFollow2.follow(m_rightLead, false);

    /* Set PID constants */
    setPIDController(m_leftPIDController);
    // setPIDController(m_rightPIDController);

    /* Set Motor and Encoder Inversions */
    // m_rightLead.setInverted(kRightMotorInverted); //only need to set Lead motors
    // m_rightMotorEncoder.setInverted(kRightMotorInverted);
    m_leftLead.setInverted(kLeftMotorInverted); //only need to set Lead motors
    // m_leftMotorEncoder.setInverted(kLeftMotorInverted);

    /* Encoder Conversion */
    m_leftWheelEncoder.setDistancePerPulse(kEncoderDistancePerPulse); //RPM
    m_leftWheelEncoder.setSamplesToAverage(10);
    // m_rightWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);

    timerRestart(m_shiftDwellTimer);
    setDesiredShiftState(ShiftingState.LOW);
    setShiftStyle(ShiftStyle.AUTO);
    driveFilter = new SlewRateLimiter(kSlewRateDrive);

    lowGear(); //Set the default for robot init
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // m_drive.arcadeDrive(driveFilter.calculate(fwd), rot);
    if (getCurrentShiftState() == getDesiredShiftState()){m_leftLead.set(fwd);}
    setTurningState(rot);
    switch (getShiftStyle()) {
      case AUTO:
        determineShiftState(fwd, rot);
        break;
      case STATIC_HIGH:
        if (getCurrentShiftState() != ShiftingState.HIGH) {
          setDesiredShiftState(ShiftingState.HIGH);
          highGear(); //abrupt shifting to get to gear immediately
        }
        break;
      case STATIC_LOW:
        if (getCurrentShiftState() != ShiftingState.LOW) {
          setDesiredShiftState(ShiftingState.LOW);
          lowGear(); //abrupt shifting to get to gear immediately
        } 
        break;
      case STATIC_NEUTRAL:
        if (getCurrentShiftState() != ShiftingState.NEUTRAL) {
          setDesiredShiftState(ShiftingState.NEUTRAL);
          neutralGear();
        }
        break;
    }
    netLastFwdEntry.setNumber(lastFwd);
    netFwdEntry.setNumber(fwd);
    lastFwd = fwd;
  }

  public void determineShiftState(double fwd, double rot) {

    //Check if driver wants to speed up or slow down ****NEED TO CHANGE TO RPM BASED****
    if ((lastFwd > 0.05 && fwd > 0.05) || (lastFwd < 0.05 && fwd <0.05)) {
      if (((abs(fwd) - abs(lastFwd)) > kUpshiftThrottleMin) || (abs(fwd) >= .95)) {
        boolThrottleReadyUpshift = true;
      } else {boolThrottleReadyUpshift = false;}
    } else {boolThrottleReadyUpshift = false;}

    
    if ( ((abs(fwd) - abs(lastFwd) <= 0) || (abs(fwd) < 0.05)) && getAverageMotorSpeed()<kRPMToDownshiftAt && abs(fwd)<.5) {
        boolThrottleReadyDownshift = true;
      } else {boolThrottleReadyDownshift = false;}

    netBoolThrotDownEntry.setBoolean(boolThrottleReadyDownshift);
    netBoolThrotUpEntry.setBoolean(boolThrottleReadyUpshift);

    //Check dwell timer against constant
    if (m_shiftDwellTimer.get() > kShiftDwellTimer) {
      boolTimerShiftReady = true;
    } else {boolTimerShiftReady = false;}

  netBoolTimerReadyEntry.setBoolean(boolTimerShiftReady);
  netDwellTimerEntry.setNumber(m_shiftDwellTimer.get());

    //Logic to set desired shifting state
    switch (getCurrentShiftState()) {
      case LOW:
        if (getAverageMotorSpeed() > kRPMToUpshiftAt - kShiftDeadband &&
            // getTurningState() == TurningState.DRIVINGSTRAIGHT &&
            boolThrottleReadyUpshift &&
            boolTimerShiftReady) {
              setDesiredShiftState(ShiftingState.HIGH);
        } //else if (getAverageMotorSpeed() > kRPMToUpshiftAt) {setDesiredShiftState(ShiftingState.HIGH);}
        break;
      case HIGH:
        if (getAverageMotorSpeed() > kRPMToDownshiftAt - kShiftDeadband && 
            getAverageMotorSpeed() < kRPMToDownshiftAt + kShiftDeadband &&
            // getTurningState() == TurningState.DRIVINGSTRAIGHT &&
            boolThrottleReadyDownshift &&
            boolTimerShiftReady) {
              setDesiredShiftState(ShiftingState.LOW);
        }
        else if (getAverageMotorSpeed() < 100) {
          setDesiredShiftState(ShiftingState.LOW);
        }
        break;
      default:
        // if (getAverageMotorSpeed() > kRPMToUpshiftAt - kShiftDeadband) { setDesiredShiftState(ShiftingState.HIGH);}
        // else {setDesiredShiftState(ShiftingState.LOW);}
        break;
    }

    if (getCurrentShiftState() != getDesiredShiftState()) {
      shiftSmooth();
    }
  }



  public void shiftSmooth() {
    double driveMultiplier = getDriveSign();
    ShiftingState desiredState = getDesiredShiftState();
    ShiftingState currentState = getCurrentShiftState();


    if (desiredState == ShiftingState.LOW) { //Downshift
      if (currentState == ShiftingState.HIGH) {
        neutralGear();
        //Set PID
        m_leftPIDController.setReference(kRPMDownshiftSetPoint * driveMultiplier, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
        netSetpointEntry.getDouble(kRPMDownshiftSetPoint*driveMultiplier);
      } else if (currentState == ShiftingState.NEUTRAL) {
        //keep setting PID controller until RPM is within margin of wheel speed * reductions
          // shiftRollingRPM = getAverageWheelEncoderSpeed() * kLowGearRatio * driveMultiplier;
          // netSetpointEntry.setDouble(shiftRollingRPM);
          // m_leftPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
        //Once within margin, set to low gear
          if (getAverageMotorSpeed() > abs(kRPMDownshiftSetPoint) - kShiftDeadband &&
              getAverageMotorSpeed() < abs(kRPMDownshiftSetPoint) + kShiftDeadband) {
            lowGear();
          }

      }
    } else if (desiredState == ShiftingState.HIGH) {
      if (currentState == ShiftingState.LOW) {
        neutralGear();
        m_leftPIDController.setReference(kRPMUpshiftSetPoint * driveMultiplier, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
        netSetpointEntry.getDouble(kRPMUpshiftSetPoint*driveMultiplier);
      } else if (currentState == ShiftingState.NEUTRAL) {
        //keep setting PID controller until RPM is within margin of wheel speed * reductions
        // shiftRollingRPM = getAverageWheelEncoderSpeed() * kHighGearRatio * driveMultiplier;
        // netSetpointEntry.setDouble(shiftRollingRPM);
        // m_leftPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
        //Once within margin, set to low gear
          if (getAverageMotorSpeed() > abs(kRPMUpshiftSetPoint) - kShiftDeadband &&
              getAverageMotorSpeed() < abs(kRPMUpshiftSetPoint) + kShiftDeadband) {
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
    //timerRestart(m_shiftDwellTimer); //Do not reset timer as shift sequence is not complete
  }

  public double getAverageWheelEncoderSpeed() {
    return abs(m_leftWheelEncoder.getRate());
    // return (abs(m_leftWheelEncoder.getRate()) + abs(m_rightWheelEncoder.getRate())) / 2.0;
  }

  public double getAverageMotorSpeed() {
    return abs(getLeftMotorRPM());
    // return (abs(getLeftMotorRPM()) + abs(getRightMotorRPM()))/2.0;
  }

  public double getLeftMotorRPM() {
    return m_leftMotorEncoder.getVelocity();
  }

  // public double getRightMotorRPM() {
  //   return m_rightMotorEncoder.getVelocity();
  // }

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

  private void setCurrentShiftState(ShiftingState CurrentShiftState) {
    shiftingStateCurrent = CurrentShiftState;
    int intCurrentState;
    switch (CurrentShiftState) {
      case LOW:
        intCurrentState = 1;
        break;
      case HIGH:
        intCurrentState = 2;
        break;
      case NEUTRAL:
        intCurrentState = 3;
        break;
      default:
        intCurrentState = 4;
        break;
    }
    netCurrentShiftStateEntry.setNumber(intCurrentState);
  }

  private ShiftingState getCurrentShiftState() { return shiftingStateCurrent; }

  private void setDesiredShiftState(ShiftingState DesiredShiftingState) {
    shiftingStateDesired = DesiredShiftingState; 

    int intDesiredState;
    switch (DesiredShiftingState) {
      case LOW:
        intDesiredState = 1;
        break;
      case HIGH:
        intDesiredState = 2;
        break;
      case NEUTRAL:
        intDesiredState = 3;
        break;
      default:
        intDesiredState = 4;
        break;
    }
    netDesShiftStateEntry.setNumber(intDesiredState);
  }

  private ShiftingState getDesiredShiftState() { return shiftingStateDesired; }

  public void setShiftStyle(ShiftStyle shiftingStyle) { shiftStyle = shiftingStyle;}

  
  public ShiftStyle getShiftStyle() { return shiftStyle; }
  
  @Override
  public void periodic() {
    setDriveSign();
    netWheelRPMEntry.setNumber(m_leftWheelEncoder.getRate());
    netMotorRPMEntry.setNumber(getLeftMotorRPM());
    // This method will be called once per scheduler run
    // Get updated numbers for slew rates
    double SlewRateDrive = slewRateDrive.getDouble(6.);
    // Compare current vs new slew rates
    if((SlewRateDrive != kSlewRateDrive)) {kSlewRateDrive = SlewRateDrive;}
    // Update slew rate limiters
    driveFilter.reset(kSlewRateDrive);
    //driveFilter = new SlewRateLimiter(kSlewRateDrive); //Use this if above does not work
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
