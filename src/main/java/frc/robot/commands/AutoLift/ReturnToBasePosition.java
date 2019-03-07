/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoLift;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.hablifter.RunClimberWheels;
import frc.robot.commands.hablifter.SetHabArmPosition;
import frc.robot.subsystems.HabLifter;

public class ReturnToBasePosition extends CommandGroup {
  /**
   * 
   */
  public ReturnToBasePosition() {
    addSequential(new SetHabArmPosition(HabLifter.TOP_DEAD_CENTER));
    addSequential(new RunClimberWheels(0.0));
  }
}
