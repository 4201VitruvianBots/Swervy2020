/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // USB PORTS
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;

    // CAN ADDRESSES
    public static final int frontRightDriveMotor = 20;
    public static final int frontRightTurningMotor = 21;
    public static final int frontLeftDriveMotor = 22;
    public static final int frontLeftTurningMotor = 23;
    public static final int backLeftDriveMotor = 24;
    public static final int backLeftTurningMotor = 25;
    public static final int backRightDriveMotor = 26;
    public static final int backRightTurningMotor = 27;

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.5;
        //Distance between centers of right and left wheels on robot. Meters?
        public static final double kWheelBase = 0.5;
        //Distance between front and back wheels on robot. Meters?

        public static Translation2d[] modulePositions = {
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            modulePositions[0],
            modulePositions[1],
            modulePositions[2],
            modulePositions[3]
        );

        public static final boolean kGyroReversed = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these

        // values for your robot.
        public static final double ksVolts = 0.657;
        public static final double kvVoltSecondsPerMeter = 0.225;
        public static final double kaVoltSecondsSquaredPerMeter = 0.00924;

        public static final double kMaxSpeedMetersPerSecond = 6;
        public static final double kMaxChassisRotationSpeed = 10 * Math.PI;
    }

    public static final class ModuleConstants {
        public static final double kDriveMotorGearRatio = 6.89; //6.89 to 1
        public static final double kTurningMotorGearRatio = 12; //12 to 1
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.1016; //10.16 cm

        //Increase max speed and decrease acceleration? 2/7/21
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 12*2 * Math.PI/kTurningMotorGearRatio;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12*2 * Math.PI/kTurningMotorGearRatio;

        public static final double kDriveEncoderDistancePerPulse =
                (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR*kDriveMotorGearRatio);

        public static final double kTurningEncoderDistancePerPulse =
                (double) ((2.0 * Math.PI) / (kTurningMotorGearRatio * kEncoderCPR));

        public static final double kPModuleTurningController = 0.75;
        public static final double kDModuleTurningController = 0;

        public static final double kPModuleDriveController = 0.26;
        public static final double kDModuleDriveController = 0.0025;
        public static final double kaVoltSecondsSquaredPerRadian = 0.0348; // originally 0.3
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.5; //4.383024
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 6 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 6 * Math.PI;

        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.5;

        //Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularSpeedRadiansPerSecondSquared);

    }
}
