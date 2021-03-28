/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  int mModuleNumber;

  public final TalonFX mTurningMotor;
  public final TalonFX mDriveMotor;
  double mZeroOffset;
  boolean mInverted;

  static double kF;
  static double kP;
  static double kI;
  static double kD;
  int kI_Zone = 900;
  int kMaxIAccum = 1000000;
  int kErrorBand = 50;

  int kCruiseVelocity = 14000;
  int kMotionAcceleration = kCruiseVelocity * 10;


  private double kS = Constants.DriveConstants.ksVolts;//0.19;
  private double kV = Constants.DriveConstants.kvVoltSecondsPerMeter; //2.23;
  private double kA = Constants.DriveConstants.kaVoltSecondsSquaredPerMeter;//0.0289;

  private double mLastError = 0, mLastTargetAngle = 0, mTargetAngle, mVelocity = 0;

  private static final long STALL_TIMEOUT = 2000;
  private long mStallTimeBegin = Long.MAX_VALUE;

  private double turnOutput;
  private double driveOutput;

  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private final PIDController m_drivePIDController = new PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(Constants.ModuleConstants.kPModuleTurningController, 0, 0,
          new TrapezoidProfile.Constraints(Constants.ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, Constants.ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  public SwerveModule(int moduleNumber, TalonFX TurningMotor, TalonFX driveMotor, double zeroOffset, boolean invertTurn, boolean invertThrottle) {
    mModuleNumber = moduleNumber;
    mTurningMotor = TurningMotor;
    mDriveMotor = driveMotor;
    mZeroOffset = zeroOffset;

    mTurningMotor.configFactoryDefault();
    mTurningMotor.configOpenloopRamp(0.1);
    mTurningMotor.configClosedloopRamp(0.1);

    mTurningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
//    mTurningMotor.setSensorPhase(invertTurn);
    mDriveMotor.setSensorPhase(invertThrottle);
    mTurningMotor.setInverted(invertTurn);
    mDriveMotor.setInverted(invertThrottle);

    mTurningMotor.config_kF(0,kF);
    mTurningMotor.config_kP(0,kP);
    mTurningMotor.config_kI(0,kI);
    mTurningMotor.config_IntegralZone(0, kI_Zone);
    mTurningMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
    mTurningMotor.config_kD(0,kD);
    mTurningMotor.configMotionCruiseVelocity(kCruiseVelocity);
    mTurningMotor.configMotionAcceleration(kMotionAcceleration);
    mTurningMotor.configAllowableClosedloopError(0, kErrorBand);

    mTurningMotor.setNeutralMode(NeutralMode.Brake);
    mDriveMotor.setNeutralMode(NeutralMode.Brake);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */
  public void resetEncoders() {
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
  }


  /**
   * Returns the current angle of the module.
   *
   * @return The current angle of the module in radians.
   */
  public double getTurningRadians() {
    return mTurningMotor.getSelectedSensorPosition() * Constants.ModuleConstants.kTurningEncoderDistancePerPulse;
  }

  /**
   * Returns the current angle of the module.
   *
   * @return The current angle of the module in degrees.
   */
  public double getTurnAngle() {
    return getTurningRadians() * 180.0 / Math.PI;
  }


  /**
   * Returns the current velocity of the module.
   *
   * @return The current velocity of the module.
   */
  public double getVelocity() {
    return mDriveMotor.getSelectedSensorVelocity() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse * 10;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), new Rotation2d(getTurningRadians()));
  }


  /**
   * The last angle setpoint of the module.
   *
   * @return The last angle setpoint of the module.
   */
  public double getTargetAngle() {
    return mLastTargetAngle;
  }


  /**
   * Sets the target angle of the module.
   *
   * @param targetAngle desired direction of travel for the module.
   *
   * @return The angle setpoint of the module.
   * It also inverts the driveMotor if setpoint +- 180 = targetAngle
   */
  public double setTargetAngle(double targetAngle) { //not in use in favor of using libraries
    mLastTargetAngle = targetAngle;

    targetAngle %= 360; //makes 0 to 359
    targetAngle += mZeroOffset;

    double currentAngle = getTurnAngle(); //gets current angle
    double currentAngleMod = currentAngle % 360; //gets current angle from 0 to 360.
    if (currentAngleMod < 0)
      currentAngleMod += 360;

    double error = currentAngle - targetAngle; //gets error

    if(error > 90 || error < -90){
      if (error > 90)
        targetAngle += 180; //if error is greater than 90, then I make the error 180 more, so that it goes to the closer position and inverts the motors
      else if (error < -90)
        targetAngle -= 180; //same thing for -90, but other way
      mDriveMotor.setInverted(!mInverted);
    } else {
      mDriveMotor.setInverted(!mInverted);    // Maybe put this outside if/else?
    }

    targetAngle += currentAngle - currentAngleMod; //re adds back the part lost when we set the currentAngleMod from 0 to 360.

    double currentError = error;
    if (Math.abs(currentError - mLastError) < 7.5 && //checks for a stall
            Math.abs(currentAngle - targetAngle) > 5) {
      if (mStallTimeBegin == Long.MAX_VALUE) mStallTimeBegin = System.currentTimeMillis();
      if (System.currentTimeMillis() - mStallTimeBegin > STALL_TIMEOUT) {
        throw new MotorStallException(String.format("Angle motor on swerve module '%d' has stalled.", mModuleNumber));
      }
    } else {
      mStallTimeBegin = Long.MAX_VALUE;
    }
    mLastError = currentError; //logs error to check for stall later

    mTargetAngle = targetAngle;
    return targetAngle; //returns target angle
  }


  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState outputState = SwerveModuleState.optimize(state, new Rotation2d(getTurningRadians()));

    // Calculate the drive output from the drive PID controller.
    driveOutput = m_drivePIDController.calculate(
            getVelocity(), outputState.speedMetersPerSecond);

    //don't use ff
    //double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    turnOutput = m_turningPIDController.calculate(getTurningRadians(), outputState.angle.getRadians());

    //don't use ff
    //double turnFeedforward =
    //        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

//    driveOutput=0;
//    System.out.println("Turn PID Output: " + turnOutput);
    mDriveMotor.set(ControlMode.PercentOutput,(driveOutput));
    mTurningMotor.set(ControlMode.PercentOutput,(turnOutput));
  }

  //for testing
  public void setPercentOutput(double speed) {
    mDriveMotor.set(ControlMode.PercentOutput, speed);
  }

  //returns the turning motor for this module
  public TalonFX getTurningMotor() {
    return mTurningMotor;
  }

  //returns the drive motor for this module
  public TalonFX getDriveMotor() {
    return mDriveMotor;
  }


  private void updateSmartDashboard() {
    SmartDashboardTab.putNumber("SwerveDrive","Turning PID " + mModuleNumber, turnOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}
