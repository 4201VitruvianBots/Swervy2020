/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * Makes the robot move at a specific voltage for a set amount of time
 */
public class DumbDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final double m_timeSeconds;

  private SwerveModuleState stateStationary;
  private SwerveModuleState stateMoving;
  private double startTime;
  

  /**
   * Creates a new DumbDrive.
   *
   * @param swerveDrive The subsystem used by this command.
   * @param heading The direction for the robot to move in degrees
   * @param timeSeconds The amount of time for the command to run 
   * @param percentOutput The output of the motors
   */
  public DumbDrive(SwerveDrive swerveDrive, double heading, double timeSeconds, double percentOutput) {
    m_swerveDrive = swerveDrive;
    m_timeSeconds = timeSeconds;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);

    stateStationary = new SwerveModuleState(0,
      new Rotation2d(Units.degreesToRadians(heading))
    );

    stateMoving = new SwerveModuleState(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond * percentOutput,
      new Rotation2d(Units.degreesToRadians(heading))
    );
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setModuleStates(new SwerveModuleState[] {
      stateStationary,
      stateStationary,
      stateStationary,
      stateStationary
    });

    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.setModuleStates(new SwerveModuleState[] {
      stateMoving,
      stateMoving,
      stateMoving,
      stateMoving
    });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    initialize(); // dUMB SJORTCUT
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > m_timeSeconds;
  }
}
