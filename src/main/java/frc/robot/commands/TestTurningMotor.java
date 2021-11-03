/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class TestTurningMotor extends CommandBase {

    /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public TestTurningMotor(SwerveDrive swerveDriveSubsystem, DoubleSupplier leftX) {
      // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Forward/Back Trottle, Left/Right Strafe, Left/Right Turn
//    m_swerveDrive.testTurningMotor(m_leftX.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    m_swerveDrive.testTurningMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
