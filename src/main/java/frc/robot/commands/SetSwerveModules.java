/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;

/**
 * Command to set the swerve modules to a desired state.
 */
public class SetSwerveModules extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final SwerveModuleState[] m_desiredStates;

  double startTime;

  /**
   * Creates a new SetSwerveModules.
   *
   * @param swerveDrive   The subsystem used by this command.
   * @param desiredStates The states to set the module to.
   */
  public SetSwerveModules(SwerveDrive swerveDrive, SwerveModuleState[] desiredStates) {
    m_swerveDrive = swerveDrive;
    m_desiredStates = desiredStates;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_swerveDrive.setModuleStates(m_desiredStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return /*m_swerveDrive.ModulesAtThreshhold(m_desiredStates) ||*/ Timer.getFPGATimestamp() - startTime > 1;
  }
}
