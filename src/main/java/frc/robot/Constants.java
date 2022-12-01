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

        public static final int[] kShiftSolenoidA = new int[] {0,1};
        public static final int[] kShiftSolenoidB = new int[] {2,3};
        public static final int kPcmCanID = 33;
        
        public static final int[] kLeftDriveMotors = new int[] {41,42,43};
        public static final boolean kLeftMotorInverted = false;
        public static final int[] kLeftWheelEncoderPorts = new int[] {0, 1};
        public static final boolean kLeftWheelEncoderReversed = true;
        public static final int kWheelEncoderCountsPerRevolution = 2048;
        public static final double kEncoderDistancePerPulse = (360./kWheelEncoderCountsPerRevolution)*0.1666667;

        public static final double kWheelDiameter = 4.;
        public static final double kHighGearSpeed = 14.77;
        public static final double kLowGearSpeed = 11.36;
        public static final double kHighGearRatio = 6.706;
        public static final double kLowGearRatio = 8.718;

        public static final double kP = 0.00018; 
        public static final double kFF = 0.00018; 

        public static final String kDriveTabName = "Drive Subsystem";
    }

    public static final class ShiftConstants {
        public static final int kMotorMaxRPM = 5700;
        public static final double kUpshiftPercent = .75; //percentage of max to upshift at
        public static final double kUpshiftThrottleMin = .05;
        public static final double kDownshiftThrottleMin = .1;
        public static final double kDownshiftPercent = .55;
        public static final double kShiftDwellTimer = 1.500; //seconds
        public static final double kTurnDeadband = 0.075;
        public static final double kRPMToDownshiftAt = (DriveConstants.kLowGearSpeed*kDownshiftPercent)/(DriveConstants.kHighGearSpeed/kMotorMaxRPM);
        public static final double kRPMToUpshiftAt = kUpshiftPercent * kMotorMaxRPM;
        public static final double kRPMUpshiftSetPoint = (DriveConstants.kLowGearSpeed*kUpshiftPercent)/(DriveConstants.kHighGearSpeed/kMotorMaxRPM);
        public static final double kRPMDownshiftSetPoint = kDownshiftPercent * kMotorMaxRPM;
        public static final double kNeutralTime = 0.060;
        public static final double kShiftDeadband = 150.;
    }

    public static final class OIConstants {
        public static final int kDriveControllerInput = 0;
    }
}
