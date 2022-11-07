// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int[] kTestDriveMotors = new int[] {9,10,11};

        public static final int[] kShiftSolenoidA = new int[] {2,3}; //Plumb shifting cylinders to the same solenoids
        public static final int[] kShiftSolenoidB = new int[] {4,5};

        public static final int kPcmCanID = 33;

        public static final boolean ktestMotorInverted = false;

        public static final int[] ktestWheelEncoderPorts = new int[] {0, 1};
        public static final boolean ktestWheelEncoderReversed = false;

        public static final int kWheelEncoderCountsPerRevolution = 2048;

        public static final double kWheelDiameter = 4.;
        public static final double kHighGearSpeed = 11.;
        public static final double kLowGearSpeed = 5.35;
        public static final double kHighGearRatio = 7.29;
        public static final double kLowGearRatio = 15.;

        public static final double kP= 6e-5; 
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0.000015; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;

        public static final String kDriveTabName = "Drive Subsystem";
    }

    public static final class ShiftConstants {
        public static final int kMotorMaxRPM = 5700;
        public static final double kUpshiftPercent = .85; //percentage of max to upshift at
        public static final double kUpshiftThrottleMin = .15;
        public static final double kDownshiftThrottleMin = .1;
        public static final double kDownshiftPercent = .55;
        public static final double kShiftDwellTimer = .500; //seconds
        public static final double kTurnDeadband = 0.075;
        public static final double kRPMToDownshiftAt = (DriveConstants.kLowGearSpeed*kDownshiftPercent)/(DriveConstants.kHighGearSpeed/kMotorMaxRPM);
        public static final double kRPMToUpshiftAt = kUpshiftPercent * kMotorMaxRPM;
        public static final double kRPMUpshiftSetPoint = (DriveConstants.kLowGearSpeed*kUpshiftPercent)/(DriveConstants.kHighGearSpeed/kMotorMaxRPM);
        public static final double kRPMDownshiftSetPoint = kDownshiftPercent * kMotorMaxRPM;

        public static final double kShiftDeadband = 125.;



    }

    public static final class OIConstants {
        public static final int kDriveControllerInput = 0;
        public static final int kOperatorControllerInput = 1;
    }
}
