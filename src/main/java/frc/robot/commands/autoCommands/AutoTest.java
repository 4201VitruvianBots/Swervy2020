package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.SetSwerveModules;
import frc.robot.subsystems.SwerveDrive;
//import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.List;

public class AutoTest extends SequentialCommandGroup {
    public AutoTest(SwerveDrive swerveDrive) {
        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        // new Translation2d(0, 0),
                        // new Translation2d(Units.feetToMeters(10), 0)
                        // new Translation2d(Units.feetToMeters(10), Units.feetToMeters(5))
                ),
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(5), new Rotation2d(0)),
                config
        );

        SwerveControllerCommand autoTest = new SwerveControllerCommand(
                exampleTrajectory,
                swerveDrive::getPose, //Functional interface to feed supplier
                Constants.DriveConstants.kDriveKinematics,

                //Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints),

                swerveDrive::setModuleStates,

                swerveDrive

        );
//                RamseteCommand driveStraight = new RamseteCommand(
//                exampleTrajectory,
//                swerveDrive::getPose,
//                new RamseteController(),
//                new SimpleMotorFeedforward(0.683, 3.19, 0.227),
//                new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth),
//                swerveDrive::getSpeeds,
//                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                swerveDrive::setVoltageOutput,
//                swerveDrive
//        );
        addCommands(new ResetOdometry(swerveDrive).andThen(() -> swerveDrive.zeroHeading()),
                new SetSwerveModules(swerveDrive, new SwerveModuleState[] {
                        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(180))),
                        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(180))),
                        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(180))),
                        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(180)))}),
                autoTest.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));// Run path following command, then stop at the end.
    }
}
