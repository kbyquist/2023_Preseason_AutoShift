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
import edu.wpi.first.wpilibj.Solenoid;
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
  private final DoubleSolenoid p_ShiftA =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kShiftSolenoidA[0],kShiftSolenoidA[1]);

  private final DoubleSolenoid p_ShiftB =
    new DoubleSolenoid(kPcmCanID,PneumaticsModuleType.CTREPCM,kShiftSolenoidB[0],kShiftSolenoidB[1]);
  
  /* Define Motors in DriveSubsystem */
  private final CANSparkMax m_testLead = new CANSparkMax(kTestDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_testFollow1 = new CANSparkMax(kTestDriveMotors[1], MotorType.kBrushless); //Must be middle motor
  private final CANSparkMax m_testFollow2 = new CANSparkMax(kTestDriveMotors[2], MotorType.kBrushless);
  // private final CANSparkMax m_rightLead = new CANSparkMax(kRightLeadMotorID, MotorType.kBrushless);
  // private final CANSparkMax m_rightFollow1 = new CANSparkMax(kRightFollowMotor1ID, MotorType.kBrushless);
  // private final CANSparkMax m_rightFollow2 = new CANSparkMax(kRightFollowMotor2ID, MotorType.kBrushless);

  /* PID Controllers */
  private final SparkMaxPIDController m_testPIDController = m_testLead.getPIDController();

  /* Encoders */
  private final RelativeEncoder m_testMotorEncoder = m_testLead.getEncoder();

  private final Encoder m_testWheelEncoder = 
      new Encoder(
        ktestWheelEncoderPorts[0],
        ktestWheelEncoderPorts[1],
        ktestWheelEncoderReversed,
        EncodingType.k1X);

  // PID coefficients
  public double kP = 9e-5; //default = 6e-5 
  public double kI = 8e-7; //default = 0
  public double kD = 0; 
  public double kIz = 0; 
  public double kFF = 0.000015; 
  public double kMaxOutput = 1; 
  public double kMinOutput = -1;
  public double kSetPoint = 0;
  public boolean kInvertMotor = false;

  // Creates a SlewRateLimiter that limits the rate of change of the signal to defined constant units per second
  public double kSlewRateDrive = 3.5;
  public double kSlewRateRotate = 3.5;
  SlewRateLimiter rotateFilter;
  SlewRateLimiter driveFilter;

  /** Shuffleboard Setup */
  private ShuffleboardTab tabPID = Shuffleboard.getTab(kDriveTabName);
  private NetworkTableEntry pGain = tabPID.add("P Gain", kP).getEntry();
  private NetworkTableEntry iGain = tabPID.add("I Gain", kI).getEntry();
  private NetworkTableEntry dGain = tabPID.add("D Gain", kD).getEntry();
  private NetworkTableEntry iZone = tabPID.add("I Zone", kIz).getEntry();
  private NetworkTableEntry fF = tabPID.add("Feed Forward", kFF).getEntry();
  private NetworkTableEntry setPointEntry = 
          tabPID.add("RPM", kSetPoint)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", kMotorMaxRPM))
          .getEntry();
  private NetworkTableEntry reverseDirection = 
          tabPID.add("Reverse Drive", false)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();
  private NetworkTableEntry motorVelocity =
          tabPID.add("Current Velocity", 0.)
          .withWidget(BuiltInWidgets.kGraph)
          .getEntry();
  private NetworkTableEntry setPointGraph = 
          tabPID.add("Setpoint Velocity", 0.)
          .getEntry();
  private NetworkTableEntry motor_setInverted = 
          tabPID.add("Motor Inversion", false)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();

  /** Creates a new Drivetrain. Initialize hardware here */
  public DriveSubsystem() {
    /* Restore Defaults of Motors. 
    Doing this confirms the settings will be the same no matter what physical controller is used  */
    m_testLead.restoreFactoryDefaults();
    m_testFollow1.restoreFactoryDefaults();
    m_testFollow2.restoreFactoryDefaults();

    /** Create Follow Groups */
    m_testFollow1.follow(m_testLead, true); //Must be middle motor
    m_testFollow2.follow(m_testLead, false);

    /** Set PID constants */
    m_testPIDController.setP(kP);
    m_testPIDController.setI(kI);
    m_testPIDController.setD(kD);
    m_testPIDController.setIZone(kIz);
    m_testPIDController.setFF(kFF);
    m_testPIDController.setOutputRange(kMinOutput, kMaxOutput);

    /** Encoder Conversion */
    m_testWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);
    // m_rightWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);

    timerRestart(m_shiftDwellTimer);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {

    //m_drive.arcadeDrive(driveFilter.calculate(fwd), rot);
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

  public void setShiftState() {
    switch (getDesiredShiftState()) {
      case LOW: //Upshifting
        switch (getCurrentShiftState()) {
          case HIGH:
            neutralGear();
            m_testPIDController.setReference(kRPMDownshiftSetPoint, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
            // m_rightPIDController.setReference(kRPMDownshiftSetPoint, CANSparkMax.ControlType.kVelocity);
            break;
          case NEUTRAL:
            //keep setting PID controller until RPM is within margin of wheel speed * reductions
              shiftRollingRPM = getAverageWheelEncoderSpeed()*kHighGearRatio;
              m_testPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
              // m_rightPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
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
            m_testPIDController.setReference(kRPMUpshiftSetPoint, CANSparkMax.ControlType.kVelocity); //May need to deal with directionallity
            // m_rightPIDController.setReference(kRPMUpshiftSetPoint, CANSparkMax.ControlType.kVelocity);
            break;
            case NEUTRAL:
            //keep setting PID controller until RPM is within margin of wheel speed * reductions
              shiftRollingRPM = getAverageWheelEncoderSpeed()*kLowGearRatio;
              m_testPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
              // m_rightPIDController.setReference(shiftRollingRPM, CANSparkMax.ControlType.kVelocity);
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

  public void tunePID() {
    double p = pGain.getDouble(0.);
    double i = iGain.getDouble(0.);
    double d = dGain.getDouble(0.);
    double iz = iZone.getDouble(0.);
    double ff = fF.getDouble(0.);
    double setPoint = setPointEntry.getDouble(0.);
    boolean driveReverse = reverseDirection.getBoolean(false);
    boolean invertMotor = motor_setInverted.getBoolean(false);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_testPIDController.setP(p); kP = p; }
    if((i != kI)) { m_testPIDController.setI(i); kI = i; }
    if((d != kD)) { m_testPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_testPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_testPIDController.setFF(ff); kFF = ff; }
    if((setPoint != kSetPoint)) { kSetPoint = setPoint; }
    if(driveReverse) {kSetPoint *= -1;}

    //Need to test if setting the motor setInverted property affects which way is forward for the PID setReference method
    if((invertMotor != kInvertMotor)) { m_testLead.setInverted(kInvertMotor); kInvertMotor = invertMotor; }
    
    m_testPIDController.setReference(kSetPoint, CANSparkMax.ControlType.kVelocity);
    
    motorVelocity.setDouble(getTestMotorRPM());
    setPointGraph.setDouble(kSetPoint);
  }

  /** lowGear is full retract = Air to PORT C */
  public void lowGear() {
    p_ShiftA.set(kForward);
    p_ShiftB.set(kForward);

    setCurrentShiftState(ShiftingState.LOW);
    timerRestart(m_shiftDwellTimer);
  }

  // /** highGear is full extend = Air to PORT B */
  public void highGear(){
    p_ShiftA.set(kReverse);
    p_ShiftB.set(kReverse);

    setCurrentShiftState(ShiftingState.HIGH);
    timerRestart(m_shiftDwellTimer);
  }

  // /** neutralGear is half extend = Air to PORT A */
  public void neutralGear(){
    p_ShiftA.set(kForward); //kForward = Air is on to PORT A, off to PORT B
    p_ShiftB.set(kReverse); //turn off PORT C air

    setCurrentShiftState(ShiftingState.NEUTRAL);
  }

  /**
   * Gets the average absolute speed of the left and right set of wheel encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageWheelEncoderSpeed() {
    return abs(m_testWheelEncoder.getRate());

    // return (abs(m_leftWheelEncoder.getRate()) + abs(m_rightWheelEncoder.getRate())) / 2.0;
  }

  /**
   * Gets the average absolute rpm of the left and right transmission motors
   * 
   * @return the average RPMs of the motor encoders
   */
  public double getAverageMotorSpeed() {
    return abs(getTestMotorRPM());
    // return (abs(getLeftMotorRPM()) + abs(getRightMotorRPM()))/2.0;
  }

  /**
   * Gets the test drive motor RPM
   * 
   * @return the test transmission motors RPM
   */
  public double getTestMotorRPM() {
    return m_testMotorEncoder.getVelocity();
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
