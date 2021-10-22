package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

//import frc.vitruvianlib.utils.TrajectoryUtils;

/**
 * Uses tank drive to drive the robot a certain distance in a certain direction
 */
public class DriveAngleTank extends SequentialCommandGroup {
    /**
     * Creates a new DriveAngleTank.
     * 
     * @param swerveDrive    The swerveDrive used by the command.
     * @param headingDegrees The direction for the robot to move.
     * @param distanceMeters The distance for the robot to move.
     */
    public DriveAngleTank(SwerveDrive swerveDrive, double headingDegrees, double distanceMeters) {
        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(headingDegrees))),
                List.of(
                        // new Translation2d(1, 0),
                        // new Translation2d(2, 0)
                ),
                new Pose2d(
                    distanceMeters * Math.cos(Units.degreesToRadians(360-headingDegrees)),
                    distanceMeters * Math.sin(Units.degreesToRadians(360-headingDegrees)),
                    new Rotation2d(Units.degreesToRadians(headingDegrees))
                ),
                config
        );

        RamseteCommand driveStraight = new RamseteCommand(
            exampleTrajectory,
            swerveDrive::getTankPose,
            new RamseteController(),
            Constants.DriveConstants.kTankKinematics,
            swerveDrive::setTankSpeeds,
            swerveDrive
        );

        addCommands(new ResetOdometry(swerveDrive)
                .andThen(()-> swerveDrive.setTankDirection(new Rotation2d(Units.degreesToRadians(headingDegrees)))),   
                driveStraight.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));// Run path following command, then stop at the end.
    }
}
