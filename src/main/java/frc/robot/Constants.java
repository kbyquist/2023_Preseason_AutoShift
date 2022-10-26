// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



public final class Constants {
    public static final class DriveConstants {
        public static final int[] kLeftDriveMotors = new int[] {0,1,2};
        public static final int[] kRightDriveMotors = new int[] {3,4,5};

        public static final int[] kShiftSolenoidA = new int[] {0,1}; //Plumb shifting cylinders to the same solenoids
        public static final int[] kShiftSolenoidB = new int[] {2,3};

        public static final int kPcmCanID = 33;

        public static final boolean kLeftMotorInverted = false;
        public static final boolean kRightMotorInverted = true;

        public static final int[] kLeftWheelEncoderPorts = new int[] {0, 1};
        public static final int[] kRightWheelEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftWheelEncoderReversed = false;
        public static final boolean kRightWheelEncoderReversed = true;
        public static final int kWheelEncoderCountsPerRevolution = 2048;

        public static final double kWheelDiameterIN = 4.;
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
        public static final double kTrackwidthMeters = 0.69;

        public static final double kWheelDiameterMeters = kWheelDiameterIN/39.37;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kWheelEncoderCountsPerRevolution;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }

    public static final class OIConstants {
        public static final int kDriveControllerInput = 0;
        public static final int kOperatorControllerInput = 1;
    }
    

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}