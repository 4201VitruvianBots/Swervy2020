package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(SwerveDrive swerveDrive) {


        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);

        int[][] waypointsRaw = {
                {0,0,0},
                {120,0,0},
                {120,60,0}
        };

        Pose2d prevPose = null;

        // Loop to convert numbers all the way to commands all in one
        for (int i = 0; i < waypointsRaw.length; i++) {
            // Modify constraints based on indices
            config.setStartVelocity(i == 0 ? 0 : config.getMaxVelocity());                      // If we've just started, start at 0
            config.setEndVelocity(i == waypointsRaw.length - 1 ? 0 : config.getMaxVelocity());  // If we're about to end, end at 0

            // Create the first pose from raw numbers
            Pose2d pose2d = new Pose2d(
                    Units.inchesToMeters(waypointsRaw[i][0]),
                    Units.inchesToMeters(waypointsRaw[i][1]),
                    new Rotation2d(Units.degreesToRadians(waypointsRaw[i][2]))
            );

            // Create the trajectory if possible
            Trajectory trajectory;
            if (prevPose != null) {
                trajectory = TrajectoryGenerator.generateTrajectory(
                        prevPose,
                        List.of(),
                        pose2d,
                        config
                );

                // Generate & append command from trajectory
                SwerveControllerCommand command = new SwerveControllerCommand(
                        trajectory,
                        swerveDrive::getPose,
                        Constants.DriveConstants.kDriveKinematics,

                        // PIDs
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController,
                                0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints
                        ),
                        swerveDrive::setModuleStates,
                        swerveDrive
                );

                // Append that command
                addCommands(command);
            }

            prevPose = pose2d;              // Finally, set the previous pose to this new one
        }
    }
}
