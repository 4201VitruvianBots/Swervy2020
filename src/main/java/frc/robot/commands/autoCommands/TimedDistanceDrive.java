/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/**
 * Makes the robot move at a specific voltage for a set amount of time
 * 
 * @deprecated Timing should not be relied on because it requires a full battery, use {@link ManualDrive} instead
 */
@Deprecated
public class TimedDistanceDrive extends TimedDrive {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  @Deprecated
  public TimedDistanceDrive(SwerveDrive swerveDrive, double heading, double distanceMeters, double percentOutput) {
    super(swerveDrive, heading, percentOutput * distanceMeters / Constants.AutoConstants.kMaxSpeedMetersPerSecond, percentOutput);
  }
}
