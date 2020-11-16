/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import com.team254.lib.util.MinTimeBoolean;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Enums.IntakeStates;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * An example command that uses an example subsystem.
 */
public class ControlledIntakeOld extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final Intake m_intake;

  private double intakeRPM = 500;
  private double indexRPM = 200;
  private double timestamp, intakeTimestamp, indexerTimestamp;
  private MinTimeBoolean fourBallTrigger;
  private boolean intaking, delaying, haveFour;

  private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ControlledIntakeOld(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    haveFour = false;
    fourBallTrigger = new MinTimeBoolean(1);
    timestamp = Timer.getFPGATimestamp();
    if(m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FIVE_BALLS;
    else if(m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FOUR_BALLS;
    else if(m_indexer.getIndexerBottomSensor())
      intakeState = IntakeStates.INTAKE_ONE_BALL;
    else
      intakeState = IntakeStates.INTAKE_EMPTY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Intake State", intakeState.toString());

    switch (intakeState) {
      case INTAKE_FIVE_BALLS:
//        m_intake.setRPM(0);
        m_indexer.setKickerOutput(0);
        m_indexer.setIndexerOutput(0);
        break;
      case INTAKE_FOUR_BALLS:
        // TODO: Verify this logic
        intaking = false;
//        m_intake.setRPM(intakeRPM);
        m_indexer.setKickerOutput(0);
        if (m_indexer.getIntakeSensor() && !intaking) {
          intakeTimestamp = Timer.getFPGATimestamp();
          indexerTimestamp = Timer.getFPGATimestamp();
          intaking = true;
        }
        if(intaking && (timestamp - intakeTimestamp) > 0.05) {
          intaking = false;
          intakeState = IntakeStates.INTAKE_FIVE_BALLS;
        }
        break;
      case INTAKE_ONE_BALL:
//        m_intake.setRPM(intakeRPM);
        m_indexer.setKickerOutput(-0.25);
        if (m_indexer.getIntakeSensor() && !delaying) {
          intakeTimestamp = Timer.getFPGATimestamp();
          delaying = true;
        }
        if(delaying && (timestamp - intakeTimestamp) > 0.1){
          delaying = false;
          indexerTimestamp = Timer.getFPGATimestamp();
          intaking = true;
        }

        haveFour = fourBallTrigger.update(m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor(), timestamp);
        if(haveFour) {
          m_indexer.setRPM(0);
          intakeState = IntakeStates.INTAKE_FOUR_BALLS;
        }
        break;
      case INTAKE_EMPTY:
      default:
//        m_intake.setRPM(intakeRPM);
//        m_indexer.setKickerOutput(-0.25);
//        if (m_indexer.getIntakeSensor() && !intaking) {
//          indexerTimestamp = Timer.getFPGATimestamp();
//          intaking = true;
//        } else if(intaking && m_indexer.getRPM() == 0)
//          intakeState = IntakeStates.INTAKE_ONE_BALL;
//        m_intake.setRPM(intakeRPM);
        m_indexer.setKickerOutput(-0.25);
        if (m_indexer.getIntakeSensor()) {
          m_indexer.setRPM(225);
          intaking = true;
        }
        if (m_indexer.getIndexerBottomSensor() && intaking) {
          m_indexer.setRPM(0);
          intaking = false;
          intakeState = IntakeStates.INTAKE_ONE_BALL;
        }
        break;
    }
    timestamp = Timer.getFPGATimestamp();
    updateTimedRollers();
  }

  private void updateTimedRollers() {
//    if(timestamp - intakeTimestamp < 0.1)
//      m_intake.setRPM(intakeRPM / 2.0);
//    else
//      m_intake.setRPM(0);
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
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerOutput(0);
    m_indexer.setKickerOutput(0);
    //if(intakeState == IntakeStates.INTAKE_FIVE_BALLS)
      //m_intake.setintakePiston(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
