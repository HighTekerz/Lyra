/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.hablifter.RetractLegs;
import frc.robot.commands.hablifter.RunClimberWheels;
import frc.robot.commands.hablifter.SetHabArmPosition;
import frc.robot.subsystems.HabLifter;

public class RetractLegsButRunWheels extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RetractLegsButRunWheels() {
    addSequential(new SetHabArmPosition(HabLifter.END_DEGREES_FOR_HAB_CLIMB + 6.0));
    addParallel(new RunClimberWheels(.2));
    addSequential(new RetractLegs());    
  }
}
