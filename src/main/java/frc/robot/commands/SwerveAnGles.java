/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SwerveAngles extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final DoubleSupplier m_leftX, m_leftY;
  private final IntSupplier m_angSupply;
  private double m_ang;
  private final PIDController pidcontroller = new PIDController(0.1, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SwerveAngles(SwerveDrive swerveDriveSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, IntSupplier angSupply) {
    pidcontroller.enableContinuousInput(-180, 180);
    m_swerveDrive = swerveDriveSubsystem;
    m_leftX = leftX;
    m_leftY = leftY;
    m_angSupply = angSupply;
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
    m_ang = m_angSupply.getAsInt();
    if(m_ang >= 0 ) {
      m_ang = -m_ang;
      if(RobotBase.isReal())
        m_swerveDrive.drive(m_leftY.getAsDouble(), m_leftX.getAsDouble(), Units.degreesToRadians(pidcontroller.calculate(m_swerveDrive.getHeading() ,m_ang)),false);
      else
        m_swerveDrive.drive(-m_leftY.getAsDouble(), m_leftX.getAsDouble(), Units.degreesToRadians(pidcontroller.calculate(m_swerveDrive.getHeading() ,m_ang)),false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_swerveDrive.getHeading() - m_ang) < 2 ;
  }
}
