/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivetrain.DriveForDistance;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.multiarm.HatchFingerHold;
import frc.robot.subsystems.Elevator;

public class GrabLiftBackUp extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GrabLiftBackUp() {
    addSequential(new HatchFingerHold());
    addSequential(new SetElevatorHeight(Elevator.HP_LEVEL_1));
    addSequential(new DriveForDistance(-6.0, 0.3));
  }
}
