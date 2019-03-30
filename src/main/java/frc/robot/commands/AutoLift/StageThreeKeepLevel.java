/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoLift;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivetrain.StartCoastMode;
import frc.robot.commands.hablifter.DeployLegs;
import frc.robot.commands.hablifter.KeepLevel;
import frc.robot.commands.hablifter.KeepLevelUntilAnAngle;
import frc.robot.commands.hablifter.RunClimberWheels;
import frc.robot.commands.hablifter.RunClimberWheelsOnAngle;

public class StageThreeKeepLevel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public StageThreeKeepLevel() {
    addSequential(new StartCoastMode());
    addParallel(new RunClimberWheelsOnAngle(0.15, 0.75, -135));
    addSequential(new DeployLegs());
    addSequential(new KeepLevel(-7.0, 0.5));
  }
}