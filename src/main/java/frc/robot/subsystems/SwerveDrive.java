/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.simulation.SimulationReferencePose;
import edu.wpi.first.wpilibj.controller.PIDController;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // 3 meters per second NOT IN USE. Go to Constants.DriveConstants to find real number
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private boolean isFieldOriented;
    private final double throttle = 0.8;
    private final double turningThrottle = 0.5;

    private AHRS mNavX = new AHRS(SerialPort.Port.kMXP); //NavX
    private int navXDebug = 0;

    private double thetaSetPoint = -mNavX.getAngle();
    private final PIDController rotationController = new PIDController(0.2, 0, 0);
    private boolean setpointPending = false;
    // private boolean deltaThetaDead = false; // Whether rate of turn is within the dead zone
    private double pTheta; // Past heading
    private double rotationOutput;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics, getRotation());
//    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
//        getRotation(),
//        new Pose2d(),
//        kDriveKinematics,
//        VecBuilder.fill(0.1, 0.1, 0.1),
//        VecBuilder.fill(0.05),
//        VecBuilder.fill(0.1, 0.1, 0.1));

  PowerDistributionPanel m_pdp;
  /**
   * Just like a graph's quadrants
   * 0 is Front Left
   * 1 is Back Left
   * 2 is Front Right
   * 3 is Back Right
   */
  private SwerveModule[] mSwerveModules = new SwerveModule[] {
          new SwerveModule(0, new TalonFX(Constants.frontLeftTurningMotor), new TalonFX(Constants.frontLeftDriveMotor), 0, true, false),
          new SwerveModule(1, new TalonFX(Constants.frontRightTurningMotor), new TalonFX(Constants.frontRightDriveMotor), 0, true, false), //true
          new SwerveModule(2, new TalonFX(Constants.backLeftTurningMotor), new TalonFX(Constants.backLeftDriveMotor), 0, true, false),
          new SwerveModule(3, new TalonFX(Constants.backRightTurningMotor), new TalonFX(Constants.backRightDriveMotor), 0, true, false) //true
  };

    private double m_trajectoryTime;
    private Trajectory currentTrajectory;
  int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

    private Rotation2d headingTarget;
    private Pose2d headingTargetPosition = new Pose2d(-1, -1, new Rotation2d());
  public void testTurningMotor(double speed){
    mSwerveModules[0].mTurningMotor.set(ControlMode.PercentOutput,speed);
  } //implemnted to test to see if the motors were responding correctly

  public void resetOdometry(Pose2d pose, Rotation2d rotation) {
    m_odometry.resetPosition(pose, rotation);
  } //resets both position and rotation (0,0,0)

  public SwerveDrive(PowerDistributionPanel pdp) {
    m_pdp = pdp; //initilize pdp
    rotationController.enableContinuousInput(-180, 180);

    SmartDashboardTab.putData("SwerveDrive","swerveDriveSubsystem", this); //update smartdashboard
  }

    /**
   * Returns the raw angle of the robot in degrees
  mNavX.getAngle()*
   * @return The angle of the robot
   */
  public double getRawGyroAngle() {
    try {
      return mNavX.getAngle();
    } catch (Exception e) {
      navXDebug = 1;
      return 0;
    }
  }


    public AHRS getNavX() {
        return mNavX;
    }

    public double getGyroRate() {
        return mNavX.getRate();
    }
  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getRotation() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(getRawGyroAngle());
  }


    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return mNavX.getRate();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        try {
            return Math.IEEEremainder(-mNavX.getAngle(), 360);
        } catch (Exception e) {
            System.out.println("Cannot Get NavX Heading");
            return 0;
        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        for (int i = 0; i < 4; i++){
            mSwerveModules[i].resetEncoders();
        }
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        mNavX.reset();
    }


    public SwerveModule getSwerveModule(int i) {
        return mSwerveModules[i];
    }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward). from -1 to 1
   * @param ySpeed        Speed of the robot in the y direction (sideways). from -1 to 1
   * @param rot           Angular rate of the robot. from -1 to 1
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   *
   * Calculates needed position and sets each module to the correct state
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    if (Math.abs(xSpeed) <= 0.01)
        xSpeed = 0;
    if (Math.abs(ySpeed) <= 0.01)
        ySpeed = 0;
    if (Math.abs(rot) <= 0.01) {
        rot = 0; //takes care of the dead zone
        if (Math.abs(getHeading() - pTheta) < 0.1 && setpointPending) { //Dead zone
          thetaSetPoint = getHeading();
          setpointPending = false;
        } 
        // if (setpointPending) {
        //   // thetaSetPoint = getHeading();
          
        //   setpointPending = false;
        // }
    } else if (!setpointPending) {
      setpointPending = true;
    }
    
    xSpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond; //Scales to max speed (the library wants it in m/s, not from -1,1)
    ySpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond; //Scales to max speed (the library wants it in m/s, not from -1,1)
    rot *= 6.28 * 4;
    //thetaSetPoint -= 0.1 * rot;
    pTheta = getHeading();
    if (setpointPending) {
        rotationOutput = rot;
    } else {
        rotationOutput = rotationController.calculate(getHeading(),thetaSetPoint);
    }
    var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates( //using libraries to do what we used to do
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotationOutput, getRotation())
                    : new ChassisSpeeds(xSpeed, ySpeed, rotationOutput)
    );

    //todo: rotationSpeed += PIDOutput //this PID calculates the speed needed to turn to a setpoint based off of a button input. Probably from the D-PAD
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboardTab.putNumber("SwerveDrive","Desired State",swerveModuleStates[0].angle.getDegrees());
    mSwerveModules[0].setDesiredState(swerveModuleStates[0]); //be careful of order. 0->0.2->1.1->2. 3->3.
    mSwerveModules[1].setDesiredState(swerveModuleStates[1]);
    mSwerveModules[2].setDesiredState(swerveModuleStates[2]);
    mSwerveModules[3].setDesiredState(swerveModuleStates[3]);
  }
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    mSwerveModules[0].setDesiredState(desiredStates[0]); //be careful of order. 0->0.2->1.1->2. 3->3.
    mSwerveModules[1].setDesiredState(desiredStates[1]);
    mSwerveModules[2].setDesiredState(desiredStates[2]);
    mSwerveModules[3].setDesiredState(desiredStates[3]);
  }

  /* This is from before the libararies existed. I'm keeping it for refrences for math later.


  public void holonomicDrive(double forward, double strafe, double rotationSpeed) {
    forward *= throttle; //because if they are both 1, then max output is sqrt(2), which is more than 1.
    strafe *= throttle;
    rotationSpeed *= turningThrottle; //I'll also have to check to make sure this isn't too high.
    if (isFieldOriented) { //checks to see if it's field oriented and then calculates
      double angleRad = Math.toRadians(getRawGyroAngle());
      double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad); //calculates new forward
      strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad); //calculates new strafe
      forward = temp;
    }

    double a = strafe - rotationSpeed * (WHEELBASE / 2); //calculations from document. https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383.
    double b = strafe + rotationSpeed * (WHEELBASE / 2); //the first document "Derivation of Inverse Kinematics for Swerve" is what I got my math from
    double c = forward - rotationSpeed * (TRACKWIDTH / 2);
    double d = forward + rotationSpeed * (TRACKWIDTH / 2);

    double[] angles = new double[]{ //calculates the angle needed for each module
            Math.atan2(b, c) * 180 / Math.PI,
            Math.atan2(b, d) * 180 / Math.PI,
            Math.atan2(a, d) * 180 / Math.PI,
            Math.atan2(a, c) * 180 / Math.PI
    };

    double[] speeds = new double[]{ //calculates the speed needed for each module
            Math.sqrt(b * b + c * c),
            Math.sqrt(b * b + d * d),
            Math.sqrt(a * a + d * d),
            Math.sqrt(a * a + c * c)
    };

    double max = speeds[0];

    for (double speed : speeds) {
      if (speed > max) {
        max = speed; //looks for the max
      }
    }

    if(max > 1) { //this makes sure that no speed is greater than 1.
      for (int i = 0; i < 4; i++){
        speeds[i] /= max; //if one is, scale them all down by the max.
      }
    }

    for (int i = 0; i < 4; i++) {
      if (Math.abs(forward) > 0.05 ||
              Math.abs(strafe) > 0.05 ||
              Math.abs(rotationSpeed) > 0.05) {
        mSwerveModules[i].setTargetAngle(angles[i] + 180); //to get it within 0 to 360. It was in -180 to 180
      } else {
        mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
      }
      mSwerveModules[i].setPercentOutput(speeds[i]);
    }
  }*/

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    

    public void setHeadingToTargetHeading(Rotation2d targetHeading) {
        headingTarget = targetHeading;
    }

    public void setHeadingToTargetPosition(Pose2d targetPosition) {
        headingTargetPosition = targetPosition;
    }

    public Rotation2d getHeadingToTarget() {
        return headingTarget;
    }
    public Pose2d getTargetPose() {
        return headingTargetPosition;
    }
  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
      getRotation(),
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    );
    // Update module positions based on the chassis' position, but keep the module heading
    for (int i = 0; i < mSwerveModules.length; i++) {
      var modulePositionFromChassis = modulePositions[i].rotateBy(getRotation()).plus(getPose().getTranslation());
      mSwerveModules[i].setPose(new Pose2d(modulePositionFromChassis, mSwerveModules[i].getHeading().plus(getRotation())));
    }
  }

  private void updateSmartDashboard() { //updates smart dashboard
    SmartDashboardTab.putNumber("SwerveDrive","Angle",getHeading());
    SmartDashboardTab.putNumber("SwerveDrive","Front Left Angle",mSwerveModules[0].getTurnAngle());
    SmartDashboardTab.putNumber("SwerveDrive","Back Left Angle",mSwerveModules[1].getTurnAngle());
    SmartDashboardTab.putNumber("SwerveDrive","Front Right Angle",mSwerveModules[2].getTurnAngle());
    SmartDashboardTab.putNumber("SwerveDrive","Back Right Angle",mSwerveModules[3].getTurnAngle());

    SmartDashboardTab.putNumber("SwerveDrive","navXDebug",navXDebug);
    SmartDashboardTab.putNumber("SwerveDrive","State",mSwerveModules[0].getState().angle.getDegrees());

    SmartDashboardTab.putNumber("SwerveDrive", "X", Units.metersToInches(getPose().getX()));
    SmartDashboardTab.putNumber("SwerveDrive", "Y", Units.metersToInches(getPose().getY()));

    SmartDashboardTab.putNumber("SwerveDrive", "Rotation Setpoint",thetaSetPoint);
    SmartDashboardTab.putNumber("SwerveDrive", "Change in heading", getHeading() - pTheta);
    SmartDashboardTab.putBoolean("SwerveDrive", "setpointPending", setpointPending);

//    SmartDashboardTab.putNumber("SwerveDrive","Front Left Speed",mSwerveModules[1].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Back Left Speed",mSwerveModules[2].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Back Right Speed",mSwerveModules[3].getState().speedMetersPerSecond);
    }

    @Override
    public void periodic() {
        sampleTrajectory();
        updateOdometry();
        updateSmartDashboard();

        setHeadingToTargetPosition(new Pose2d(4.5, 4, new Rotation2d()));
        if(headingTargetPosition.getX() != -1 && headingTargetPosition.getY() != -1) {
            double yDelta = headingTargetPosition.getY() - getPose().getY();
            double xDelta = headingTargetPosition.getX() - getPose().getX();
            var target = new Rotation2d(Math.atan2(yDelta, xDelta));

//            if(inputTurnInversion == -1)
//                target = target.unaryMinus();

            setHeadingToTargetHeading(target);
//            System.out.println("Target Heading: " + getHeadingToTarget());
        }

        // This method will be called once per scheduler run
    }

    public Pose2d[] getModulePoses() {
        Pose2d[] modulePoses = {
            mSwerveModules[0].getPose(),
            mSwerveModules[1].getPose(),
            mSwerveModules[2].getPose(),
            mSwerveModules[3].getPose()
        };
        return modulePoses;
    }

    double yaw = 0;
    @Override
    public void simulationPeriodic() {
        SwerveModuleState[] moduleStates = {
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState()
        };

        var chassisSpeed = kDriveKinematics.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

        yaw += chassisRotationSpeed * 0.02;
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
//        angle.set(Math.IEEEremainder(-swerveChassisSim.getHeading().getDegrees(), 360));
        angle.set(Units.radiansToDegrees(yaw));
    }

    private void sampleTrajectory() {
        if(DriverStation.getInstance().isAutonomous()) {
            try {
                var currentTrajectoryState = currentTrajectory.sample(Timer.getFPGATimestamp() - startTime);

                System.out.println("Trajectory Time: " + (Timer.getFPGATimestamp() - startTime));
                System.out.println("Trajectory Pose: " + currentTrajectoryState.poseMeters);
                System.out.println("Trajectory Speed: " + currentTrajectoryState.velocityMetersPerSecond);
                System.out.println("Trajectory angular speed: " + currentTrajectoryState.curvatureRadPerMeter);
            } catch (Exception e) {

            }
        }

    }

    public void setTrajectoryTime(double trajectoryTime) {
        m_trajectoryTime = trajectoryTime;
    }

    double startTime;
    public void setCurrentTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        startTime = Timer.getFPGATimestamp();
    }
}
