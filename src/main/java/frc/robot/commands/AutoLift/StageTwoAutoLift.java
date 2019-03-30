/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoLift;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drivetrain.DriveForDistance;
import frc.robot.commands.hablifter.RunClimberWheels;
import frc.robot.commands.hablifter.SetHabArmPosition;

public class StageTwoAutoLift extends CommandGroup {
  /**
   * Add your docs here.
   */
  public StageTwoAutoLift() {
    addSequential(new SetHabArmPosition(-100.0, 0.5));
    addSequential(new SetHabArmPosition(-145.0, 0.5));
    addParallel(new RunClimberWheels(.4));
    addSequential(new DriveForDistance(-20.0, 0.4));
    addSequential(new SetHabArmPosition(-170.0, 0.5));
    addSequential(new DriveForDistance(-14.0, 0.4));
    addSequential(new SetHabArmPosition(-90.0));
    addSequential(new DoNothing(.5));
    addSequential(new DriveForDistance(-4.0, 0.2));
  }
}
