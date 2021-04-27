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

public class Barrel extends SequentialCommandGroup {
    public Barrel(SwerveDrive swerveDrive) {


        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);

        int[][] waypointsRaw = {
                {40,90,0},
                {150,90,0},
                {176,45,-0},
                {124,45,0},
                {150,90,0},
                {240,90,0},
                {270,135,0},
                {210,120,-0},     // That's hilarious
                {260,54,-0},
                {338,60,0},
                {285,90,0},
                {140,0,0},
                {30,90,0}
        };

        Pose2d prevPose = null;

        // Loop to convert numbers all the way to commands all in one
        for (int i = 0; i < waypointsRaw.length - 1; i++) {
            // Modify constraints based on indices
            config.setStartVelocity(i == 0 ? 0 : config.getMaxVelocity());                      // If we've just started, start at 0
            config.setEndVelocity(i == waypointsRaw.length - 2 ? 0 : config.getMaxVelocity());  // If we're about to end, end at 0

            // Create the first pose from raw numbers
            Pose2d pose2d = new Pose2d(
                    Units.feetToMeters(waypointsRaw[i][0]),
                    Units.feetToMeters(waypointsRaw[i][1]),
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
