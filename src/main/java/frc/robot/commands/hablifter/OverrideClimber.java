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

public class OverrideClimber extends Command {
  HabLifter hL = Robot.Subsystems.habLifter;
  double setpoint;
  public OverrideClimber() {
    requires(hL);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    hL.enablePid();
    setpoint = hL.getArmOrPitchPositionDegrees();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    this.setpoint += Robot.oi.getRightStickYRip();
    if (setpoint > -20.0) setpoint = -20.0;
    hL.setArmSetpoint(this.setpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
