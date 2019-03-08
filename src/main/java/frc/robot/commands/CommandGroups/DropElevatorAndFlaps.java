/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.multiarm.BothFlapsDown;

public class DropElevatorAndFlaps extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DropElevatorAndFlaps() {
    addSequential(new SetElevatorHeight(0));
    addSequential(new BothFlapsDown());

  }
}
