/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivetrain.DriveByPorcupine;
import frc.robot.commands.drivetrain.DriveForDistance;
import frc.robot.commands.drivetrain.DrivePIDDistance;
import frc.robot.commands.multiarm.HPPushWhileHeld;
import frc.robot.commands.multiarm.HatchFingerHold;

public class PlaceAndReverse extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PlaceAndReverse() {
    addSequential(new DriveByPorcupine(62, 62, 0.3, 0.0, true));
    addParallel(new HPPushWhileHeld());
    addSequential(new DriveForDistance(-4.0, 0.2));
    addSequential(new HatchFingerHold());
  }
}
