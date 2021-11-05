package frc.robot.commands.autoCommands;
import frc.robot.commands.autoCommands.DriveAngleTank;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class TankSquare extends SequentialCommandGroup {
    public TankSquare(SwerveDrive swerveDrive) {
        addCommands(
            new DriveAngleTank(swerveDrive, 0, 1),
            new DriveAngleTank(swerveDrive, 90, 1)
            // new DriveAngleTank(swerveDrive, 180, 1),
            // new DriveAngleTank(swerveDrive, 270, 1)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false))
        );
    } 

}