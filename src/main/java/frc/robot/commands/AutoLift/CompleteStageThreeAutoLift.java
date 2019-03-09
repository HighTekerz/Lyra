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
import frc.robot.commands.hablifter.DeployLegs;
import frc.robot.commands.hablifter.RunClimberWheels;
import frc.robot.commands.hablifter.SetHabArmPosition;
import frc.robot.subsystems.HabLifter;

public class CompleteStageThreeAutoLift extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CompleteStageThreeAutoLift() {
    addSequential(new DeployLegs());
    addSequential(new DoNothing(1.0));
    addSequential(new SetHabArmPosition(HabLifter.END_DEGREES_FOR_HAB_CLIMB * .85));
    addParallel(new RunClimberWheels(0.75));
    addSequential(new SetHabArmPosition(HabLifter.END_DEGREES_FOR_HAB_CLIMB));
    // addSequential(new DriveForDistance(-12.0, .75));
  }
}
