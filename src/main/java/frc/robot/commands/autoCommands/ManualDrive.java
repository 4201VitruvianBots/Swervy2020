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
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * Sets the module states manually and stops when the encoder distance passes the distance
 */
public class ManualDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final double m_distanceMeters;

  private SwerveModuleState stateStationary;
  private SwerveModuleState stateMoving;
  private double startTime;
  

  /**
   * Creates a new ManualDrive.
   *
   * @param swerveDrive The subsystem used by this command.
   * @param heading The direction for the robot to move in degrees
   * @param distanceMeters The distance for the robot to travel 
   * @param percentOutput The output of the motors
   */
  public ManualDrive(SwerveDrive swerveDrive, double heading, double distanceMeters, double percentOutput) {
    m_swerveDrive = swerveDrive;
    m_distanceMeters = distanceMeters;

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
    m_swerveDrive.setModuleStates(new SwerveModuleState[] {
      stateStationary,
      stateStationary,
      stateStationary,
      stateStationary
    });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double averageDistance = (
      m_swerveDrive.getSwerveModule(0).getDriveMotor().getSelectedSensorPosition() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse + 
      m_swerveDrive.getSwerveModule(1).getDriveMotor().getSelectedSensorPosition() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse + 
      m_swerveDrive.getSwerveModule(2).getDriveMotor().getSelectedSensorPosition() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse + 
      m_swerveDrive.getSwerveModule(3).getDriveMotor().getSelectedSensorPosition() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse
    ) / 2;
    
    return averageDistance > m_distanceMeters;
  }
}
