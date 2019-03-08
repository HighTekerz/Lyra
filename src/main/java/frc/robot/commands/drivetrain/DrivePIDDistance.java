/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DrivePIDDistance extends Command {
  Drivetrain dt = Robot.Subsystems.drivetrain;
  double targetDistance;
  public DrivePIDDistance(double distance) {
    this.targetDistance = distance;
    // Use requires() here to declare subsystem dependencies
    requires(dt);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt.setPIDSetpoint(100.0, 100.0);
    dt.enablePID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // dt.disablePID();
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
