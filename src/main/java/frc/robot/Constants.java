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
        public static final int kLeftLeadMotorID = 0;
        public static final int kLeftFollowMotor1ID = 1;
        public static final int kLeftFollowMotor2ID = 2;
        public static final int kRightLeadMotorID = 3;
        public static final int kRightFollowMotor1ID = 4;
        public static final int kRightFollowMotor2ID = 5;

        public static final int kLeftShifterPortA = 0;
        public static final int kLeftShifterPortB = 1;
        public static final int kLeftShifterPortC = 2;

        public static final int kRightShifterPortA = 3;
        public static final int kRightShifterPortB = 4;
        public static final int kRightShifterPortC = 5;

        public static final int kPcmCanID = 33;

        public static final int[] kLeftWheelEncoderPorts = new int[] {0, 1};
        public static final int[] kRightWheelEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftWheelEncoderReversed = false;
        public static final boolean kRightWheelEncoderReversed = true;
        public static final int kWheelEncoderCountsPerRevolution = 2048;

        public static final double kWheelDiameter = 4.;
        public static final double kHighGearSpeed = 11.;
        public static final double kLowGearSpeed = 5.35;
        public static final double kHighGearRatio = 7.29;
        public static final double kLowGearRatio = 15.;

        public static final String kDriveTabName = "Drive Subsystem";
    }

    public static final class ShiftConstants {
        public static final int kMotorMaxRPM = 5676;
        public static final double kUpshiftPercent = .85; //percentage of max to upshift at
        public static final double kUpshiftThrottleMin = .15;
        public static final double kDownshiftThrottleMin = .1;
        public static final double kDownshiftPercent = .55;
        public static final double kShiftDwellTimer = 500.; //ms
        public static final double kTurnDeadband = 0.075;
        public static final double kRPMToDownshiftAt = (DriveConstants.kLowGearSpeed*kDownshiftPercent)/(DriveConstants.kHighGearSpeed/kMotorMaxRPM);
        public static final double kRPMToUpshiftAt = kUpshiftPercent * kMotorMaxRPM;
        public static final double kRPMUpshiftSetPoint = (DriveConstants.kLowGearSpeed*kUpshiftPercent)/(DriveConstants.kHighGearSpeed/kMotorMaxRPM);
        public static final double kRPMDownshiftSetPoint = kDownshiftPercent * kMotorMaxRPM;

        public static final double kShiftDeadband = 250.;



    }

    public static final class OIConstants {
        public static final int kDriveControllerInput = 0;
        public static final int kOperatorControllerInput = 1;
    }
}
