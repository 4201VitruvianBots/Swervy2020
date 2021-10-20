/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.commands.TestTurningMotor;
import frc.robot.commands.autoCommands.AutoTestCommand;
import frc.robot.commands.autoCommands.Bounce;
import frc.robot.commands.autoCommands.DriveStraight;
import frc.robot.commands.autoCommands.DumbDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.commands.SwerveAngles;
import frc.robot.commands.autoCommands.Slalom;
import frc.robot.commands.autoCommands.AutoTest;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.vitruvianlib.utils.JoystickWrapper;
// import java.awt.Button;
import frc.vitruvianlib.utils.XBoxTrigger;

import java.util.Map;

import static java.util.Map.entry;

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
  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);


  private enum CommandSelector {
    DRIVE_STRAIGHT,
    AUTO_TEST,
    DUMB_DRIVE
  }

  SendableChooser<Integer> m_autoChooser = new SendableChooser();

  private SelectCommand m_autoCommand;
  
  private boolean batteryFront = false; // Set this to false to drive with the battery at the back

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
    m_autoChooser.setDefaultOption("Drive Straight", CommandSelector.DRIVE_STRAIGHT.ordinal());
    for (CommandSelector commandEnum : CommandSelector.values())
      // if (commandEnum != CommandSelector.DRIVE_STRAIGHT)
        m_autoChooser.addOption(commandEnum.toString(), commandEnum.ordinal());

    SmartDashboard.putData(m_autoChooser);
    
    m_autoCommand = new SelectCommand(
            Map.ofEntries(
//                    entry(CommandSelector.SHOOT_AND_DRIVE_BACK, new ShootAndDriveBack(m_driveTrain,m_intake,m_indexer,m_turret,m_shooter,m_vision)),
                    entry(CommandSelector.DRIVE_STRAIGHT, new DriveStraight(m_swerveDrive)),
                    entry(CommandSelector.AUTO_TEST, new AutoTest(m_swerveDrive)),
                    entry(CommandSelector.DUMB_DRIVE, new DumbDrive(m_swerveDrive, 0, 2, 0.2))
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

  //  SmartDashboardTab.putData("SwerveDrive","SetSwerveDrive", new SetSwerveDrive(m_swerveDrive,
  //            () -> leftJoystick.getRawAxis(0), //left x
  //            () -> leftJoystick.getRawAxis(1), //left y
  //            () -> rightJoystick.getRawAxis(0))); //right x
//   SmartDashboardTab.putData("SwerveDrive","manualTurnCommand", new RunCommand(() -> m_swerveDrive.testTurningMotor(rightJoystick.getRawAxis(0)))); //right x


    if(RobotBase.isReal()) {

      m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
              () -> Math.pow(leftJoystick.getRawAxis(0),3), //left x
              () -> Math.pow(leftJoystick.getRawAxis(1),3), //left y
              () -> Math.pow(rightJoystick.getRawAxis(0),3))); //right x
    } else {
      m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
              () -> Math.pow(leftJoystick.getRawAxis(0),3), //left x
              () -> Math.pow(leftJoystick.getRawAxis(1),3), //left y
              () -> Math.pow(rightJoystick.getRawAxis(0),3))); //right x
    }
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // leftJoystick.invertRawAxis(0, true);
    // rightJoystick.invertRawAxis(0, true);
    leftJoystick.setAxisDeadband(0, 0.01);
    leftJoystick.setAxisDeadband(1, 0.01);
    rightJoystick.setAxisDeadband(0, 0.01);
    rightJoystick.setAxisDeadband(1, 0.01);
    leftJoystick.invertRawAxis(0, batteryFront);
    leftJoystick.invertRawAxis(1, batteryFront);
    // rightJoystick.invertRawAxis(0, true);
    xBoxController.invertRawAxis(1, true);
    xBoxController.invertRawAxis(5, true);
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
    xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
    xBoxRightTrigger = new XBoxTrigger(xBoxController, 3); 

    xBoxButtons[0].whileHeld(new SwerveAngles(
      m_swerveDrive,
      
      () -> leftJoystick.getRawAxis(0),
      () -> leftJoystick.getRawAxis(1),
      () -> xBoxController.getPOV())
    );
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
   return m_autoCommand;
    // return new TestAuto(m_swerveDrive);
    // return new DriveStraight(m_swerveDrive);
    // return new SwerveAngles(m_swerveDrive, () -> 1, () -> 1, () -> 30);
       // return new WaitCommand(0);
  }

  public void initalizeLogTopics() {
//    m_controls.initLogging();
  }

  public void teleopInit() {
    m_swerveDrive.setSwerveDriveNeutralMode(true); // Brake
  }

  public void autonomousInit() {
    m_swerveDrive.setSwerveDriveNeutralMode(true); // Brake
    m_swerveDrive.setModuleStates(new SwerveModuleState[]{
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    });
  }

  public void disabledInit() {
    m_swerveDrive.setSwerveDriveNeutralMode(false); // Coast
  }

  public void initSim() {
    m_fieldSim.initSim();
  }

  public void simulationPeriodic() {
    m_fieldSim.simulationPeriodic();
  }

  public void teleopPeriodic() {
    SmartDashboardTab.putNumber("SwerveDrive", "LeftJoystick0", leftJoystick.getRawAxis(0));
    SmartDashboardTab.putNumber("SwerveDrive", "LeftJoystick1", leftJoystick.getRawAxis(1));
    SmartDashboardTab.putNumber("SwerveDrive", "RightJoystick0", rightJoystick.getRawAxis(0));
  }
}
