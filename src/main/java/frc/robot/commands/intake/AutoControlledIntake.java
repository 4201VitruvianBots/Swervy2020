/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Enums.IntakeStates;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * An example command that uses an example subsystem.
 */
public class AutoControlledIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final Intake m_intake;

  private double intakeRPM = 5000;
  private double indexRPM = 300;
  private double timestamp, intakeTimestamp, indexerTimestamp, fourBallTimestamp;
  private boolean intaking, haveFour, haveFourTripped;

  private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoControlledIntake(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakingState(true);
    timestamp = Timer.getFPGATimestamp();

    if(m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FIVE_BALLS;
    else if(m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FOUR_BALLS;
    else
      intakeState = IntakeStates.INTAKE_ONE_BALL;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (intakeState) {
      case INTAKE_FIVE_BALLS:
        m_intake.setIntakePercentOutput(0);
        m_indexer.setKickerOutput(0);
        m_indexer.setIndexerOutput(0);
        break;
      case INTAKE_FOUR_BALLS:
        m_intake.setIntakePercentOutput(0.9);
        m_indexer.setKickerOutput(0);
        if (m_indexer.getIntakeSensor())
          intakeState = IntakeStates.INTAKE_FIVE_BALLS;
        break;
      case INTAKE_ONE_BALL:
      default:
        m_intake.setIntakePercentOutput(0.9);
        m_indexer.setKickerOutput(-0.4);
        if (m_indexer.getIndexerBottomSensor() && !intaking) {
          //indexerTimestamp = Timer.getFPGATimestamp();
          intaking = true;
          m_indexer.setIndexerOutput(0.95);
//          m_indexer.setRPM(indexRPM);
        } else {
          intaking = false;
          m_indexer.setIndexerOutput(0);
//          m_indexer.setRPM(0);
        }

//        if(m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor() && !haveFourTripped) {
//          fourBallTimestamp = Timer.getFPGATimestamp();
//          haveFourTripped = true;
//        } else if(!m_indexer.getIndexerBottomSensor() || !m_indexer.getIndexerTopSensor()){
//          fourBallTimestamp = 0;
//          haveFourTripped = false;
//          haveFour = false;
//        }

        if(m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor() ) {
          m_indexer.setRPM(0);
          intakeState = IntakeStates.INTAKE_FOUR_BALLS;
        }
        break;
    }

    //updateTimedRollers();
  }

  private void updateTimedRollers() {
    timestamp = Timer.getFPGATimestamp();

    if(fourBallTimestamp != 0)
      if((timestamp - fourBallTimestamp) > 0.5)
        haveFour = true;
      else
        haveFour = false;

    if(intakeState != IntakeStates.INTAKE_EMPTY)
      if(indexerTimestamp != 0)
        if(timestamp - indexerTimestamp < 0.1)
          m_indexer.setRPM(indexRPM);
        else {
          m_indexer.setRPM(0);
          intaking = false;
        }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakingState(false);
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerOutput(0);
    m_indexer.setKickerOutput(0);
    if(intakeState == IntakeStates.INTAKE_FIVE_BALLS)
      m_intake.setintakePiston(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
