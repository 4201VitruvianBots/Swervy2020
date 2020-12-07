package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;

public class GenerateRamseteCommand {
    VitruvianRamseteCommand ramseteCommand;

    public GenerateRamseteCommand(DriveTrain driveTrain, ArrayList<Pose2d> path, TrajectoryConfig config) {
        var trajectory = TrajectoryGenerator.generateTrajectory(path, config);

        ramseteCommand = new VitruvianRamseteCommand(
                trajectory,
                driveTrain::getRobotPose,
                new RamseteController(),
                driveTrain.getFeedforward(),
                driveTrain.getDriveTrainKinematics(),
                driveTrain::getSpeeds,
                driveTrain.getLeftPIDController(),
                driveTrain.getRightPIDController(),
                driveTrain::setVoltageOutput,
                driveTrain,
                path,
                config
        );
    }

    public VitruvianRamseteCommand getRamseteCommand() {
        return ramseteCommand;
    }
}