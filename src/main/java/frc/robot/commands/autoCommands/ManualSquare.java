package frc.robot.commands.autoCommands;
import frc.robot.commands.autoCommands.ManualDrive;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class ManualSquare extends SequentialCommandGroup {
    public ManualSquare(SwerveDrive swerveDrive) {
        addCommands(
            new ManualDrive(swerveDrive, 0, 1, 0.2),
            new ManualDrive(swerveDrive, 90, 1, 0.2),
            new ManualDrive(swerveDrive, 180, 1, 0.2),
            new ManualDrive(swerveDrive, 270, 1, 0.2),
            new ManualDrive(swerveDrive, 0, 0, 0.2)
        );
    } 

}