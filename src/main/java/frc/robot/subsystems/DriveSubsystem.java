// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import java.util.*;

import static java.lang.Math.*;

public class DriveSubsystem extends SubsystemBase {

  DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /* Define Motors in DriveSubsystem */
  private final CANSparkMax m_leftLead = new CANSparkMax(kLeftDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_leftFollow1 = new CANSparkMax(kLeftDriveMotors[1], MotorType.kBrushless);
  private final CANSparkMax m_leftFollow2 = new CANSparkMax(kLeftDriveMotors[2], MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(kRightDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_rightFollow1 = new CANSparkMax(kRightDriveMotors[1], MotorType.kBrushless);
  private final CANSparkMax m_rightFollow2 = new CANSparkMax(kRightDriveMotors[2], MotorType.kBrushless);

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
    //Pathfollowing
    resetWheelEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

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

    /* Set Motor and Encoder Inversions */
    m_rightLead.setInverted(kRightMotorInverted);
    m_leftLead.setInverted(kLeftMotorInverted);

    /* Encoder Conversion */
    m_leftWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);
    m_rightWheelEncoder.setDistancePerPulse(360./kWheelEncoderCountsPerRevolution);

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
  }

  public double getAverageWheelEncoderDistance() {
    return (m_leftWheelEncoder.getDistance() + m_rightWheelEncoder.getDistance()) / 2.0;
  }

  public double getAverageWheelEncoderSpeed() {
    return (abs(m_leftWheelEncoder.getRate()) + abs(m_rightWheelEncoder.getRate())) / 2.0;
  }

  public Encoder getLeftWheelEncoder() { return m_leftWheelEncoder; }

  public Encoder getRightWheelEncoder() { return m_rightWheelEncoder; }

  public void resetWheelEncoders() {
    m_leftWheelEncoder.reset();
    m_rightWheelEncoder.reset();
  }


  public Pose2d getPose() { return m_odometry.getPoseMeters(); }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftWheelEncoder.getRate(), m_rightWheelEncoder.getRate());
  }

  public void zeroHeading() { m_gyro.reset(); }

  public double getHeading() { return m_gyro.getRotation2d().getDegrees(); }

  public double getTurnRate() { return -m_gyro.getRate(); }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLead.setVoltage(leftVolts);
    m_rightLead.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetWheelEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  

  @Override
  public void periodic() {
  // Update the odometry in the periodic block
  m_odometry.update(
  m_gyro.getRotation2d(), m_leftWheelEncoder.getDistance(), m_rightWheelEncoder.getDistance());

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

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();
  eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  //eventMap.put("intakeDown", new IntakeDown());

  return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            this.resetOdometry(traj.getInitialPose());
        }
      }),
      new PPRamseteCommand(
          traj, 
          this::getPose, // Pose supplier
          new RamseteController(),
          new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
          this.kinematics, // DifferentialDriveKinematics
          new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
          this::tankDriveVolts, // Voltage biconsumer
          eventMap, // This argument is optional if you don't use event markers
          this // Requires this drive subsystem
      )
  );
}

}


