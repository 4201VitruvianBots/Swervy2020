/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Sets all the swerve modules to a certain direction to run a tank auto.
 */
public class SetTankDirection extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final double m_headingDegrees;
    private final Rotation2d headingRotation;

  /**
   * Creates a new SetTankDirection.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetTankDirection(SwerveDrive swerveDriveSubsystem, double headingDegrees) {
    m_swerveDrive = swerveDriveSubsystem;
    m_headingDegrees = headingDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
    headingRotation = new Rotation2d(Units.degreesToRadians(headingDegrees));
      SwerveModuleState stateStationary = new SwerveModuleState(0, headingRotation);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setTankDirection(headingRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_swerveDrive.getTankPose().getRotation().getDegrees() - m_headingDegrees) < 5;
  }
}
