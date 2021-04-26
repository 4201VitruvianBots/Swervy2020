/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AlignToPowerCell;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.commands.autoCommands.DriveStraight;
import frc.robot.commands.autoCommands.Slalom;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.JoystickWrapper;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PowerDistributionPanel pdp = new PowerDistributionPanel();

  private final SwerveDrive m_swerveDrive = new SwerveDrive(pdp);

  private final Vision m_vision = new Vision(m_swerveDrive);


  private enum CommandSelector {
    DRIVE_STRAIGHT
  }

  SendableChooser<Integer> m_autoChooser = new SendableChooser();

  private SelectCommand m_autoCommand;

  static JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
  static JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);
  static JoystickWrapper xBoxController = new JoystickWrapper(Constants.xBoxController);
  static JoystickWrapper testController = new JoystickWrapper(3);
  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[8];
  public Button xBoxLeftTrigger, xBoxRightTrigger;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser.addDefault("Drive Straight", CommandSelector.DRIVE_STRAIGHT.ordinal());
    for (Enum commandEnum : CommandSelector.values())
      if (commandEnum != CommandSelector.DRIVE_STRAIGHT)
        m_autoChooser.addOption(commandEnum.toString(), commandEnum.ordinal());

    SmartDashboard.putData(m_autoChooser);

    m_autoCommand = new SelectCommand(
            Map.ofEntries(
//                    entry(CommandSelector.SHOOT_AND_DRIVE_BACK, new ShootAndDriveBack(m_driveTrain,m_intake,m_indexer,m_turret,m_shooter,m_vision)),
                    entry(CommandSelector.DRIVE_STRAIGHT, new DriveStraight(m_swerveDrive))
//                        entry(CommandSelector.TEST_SEQUENTIAL_REVERSE_AUTO, new TestSequentialSwitching(m_driveTrain))
            ),
            this::selectCommand
    );

    initializeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
  }

  public void initializeSubsystems() {
//    m_swerveDrive.setDefaultCommand((new TestTurningMotor(m_swerveDrive, () -> leftJoystick.getRawAxis(0))));

//    SmartDashboardTab.putData("SwerveDrive","SetSwerveDrive", new SetSwerveDrive(m_swerveDrive,
//              () -> leftJoystick.getRawAxis(0), //left x
//              () -> leftJoystick.getRawAxis(1), //left y
//              () -> rightJoystick.getRawAxis(0))); //right x
//    SmartDashboardTab.putData("SwerveDrive","manualTurnCommand", new RunCommand(() -> m_swerveDrive.testTurningMotor(rightJoystick.getRawAxis(0)))); //right x


    if(RobotBase.isReal()) {

      m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
              () -> xBoxController.getRawAxis(0), //left x
              () -> xBoxController.getRawAxis(1), //left y
              () -> xBoxController.getRawAxis(2))); //right x
    } else {
      m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
              () -> testController.getRawAxis(0), //left x
              () -> testController.getRawAxis(1), //left y
              () -> testController.getRawAxis(2))); //right x
    }
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xBoxButtons[0].whenHeld(new AlignToPowerCell(m_vision, m_swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private CommandSelector selectCommand() {
    return CommandSelector.values()[m_autoChooser.getSelected()];
  }

  public Command getAutonomousCommand() {
//    return m_autoCommand;
    return new Slalom(m_swerveDrive);
       // return new WaitCommand(0);
  }

  public void initalizeLogTopics() {
//    m_controls.initLogging();
  }
}
