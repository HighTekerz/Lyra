/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.DisableElevator;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.multiarm.StartHatchPanelMode;
import frc.robot.subsystems.Elevator;

public class PanelFeederMode extends CommandGroup {
  public PanelFeederMode() {
    addParallel(new DisableElevator());
    addSequential(new StartHatchPanelMode());
  }
}
