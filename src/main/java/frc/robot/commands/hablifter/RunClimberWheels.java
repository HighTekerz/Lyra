/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hablifter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.HabLifter;
import frc.robot.subsystems.HabLifterWheels;
import frc.robot.tekerz.utilities.L;

public class RunClimberWheels extends Command {
  
  HabLifterWheels hLW = Robot.Subsystems.habLifterWheels;
  HabLifter hL = Robot.Subsystems.habLifter;
  double wheelSpeed;

  public RunClimberWheels(double wheelSpeed) {
    requires(hLW);
    this.wheelSpeed = wheelSpeed;
  }

  // public RunClimberWheels(double wheelSpeed, boolean neverStop) {
  //   requires(hL);
  //   this.wheelSpeed = wheelSpeed;
  // }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    hL.driveWheels(wheelSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    L.ogCmdEnd(this);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    L.ogCmdInterrupted(this);
    hL.driveWheels(0);
  }
}
