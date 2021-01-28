package org.wint3794.frc.robot.util;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    public static final class Drivetrain {

        public static final double kMaxSpeed = 3.0;
        public static final double kMaxAngularSpeed = Math.PI;
    
        public static final double kTrackWidth = 0.5786;
        public static final double kWheelRadius = Units.inchesToMeters(3);

        public static final int kEncoderResolution = -4096;

        public static final double kDistancePerPulse = 
            2 * Math.PI * Drivetrain.kWheelRadius / Drivetrain.kEncoderResolution;

        public static final int[][] kMotorIDs = {{0, 1, 2},{3, 4, 5}};

        public static final int[] kRightEncoderPorts = {0, 1};
        public static final int[] kLeftEncoderPorts = {2, 3};

        public static final double[] kPID = {8.5, 0, 0};

		public static final double kMass = 45;

		public static final double kGearingReduction = 7.29;

		public static double kMOI = 7.5;
    }
}
