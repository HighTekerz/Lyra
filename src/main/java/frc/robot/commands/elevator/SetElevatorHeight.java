/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.tekerz.utilities.L;

public class SetElevatorHeight extends Command {
  private Elevator el = Robot.Subsystems.elevator;
  private double 
    targetHeightInInches = 0.0;

  public SetElevatorHeight(double targetHeightInInches) {
    // Use requires() here to declare subsystem dependencies
    this.targetHeightInInches = targetHeightInInches;
    requires(el);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		L.ogCmdInit(this);
    el.setSetpoint(targetHeightInInches);
    el.enableElevator();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ?el.getElevatorPosition(), false:true;
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
  }
}