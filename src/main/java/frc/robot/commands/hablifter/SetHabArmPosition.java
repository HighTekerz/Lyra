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
import frc.robot.tekerz.utilities.L;

public class SetHabArmPosition extends Command {
  private double 
    positionInDegrees = 0.0,
    maximumOutput = 0.2;

  HabLifter h = Robot.Subsystems.habLifter;

  public SetHabArmPosition(double positionInDegrees){
    this(positionInDegrees, 0.2);
  }

  public SetHabArmPosition(double positionInDegrees, double maximumOutput) {
    this.positionInDegrees = positionInDegrees;
    this.maximumOutput = maximumOutput;
    requires(h);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
    h.setArmSetpoint(positionInDegrees, maximumOutput);
    L.og("setpoint in degrees: " + positionInDegrees);
    h.enablePid();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return h.getArmOrPitchPositionDegrees() <= (positionInDegrees + 4) && h.getArmOrPitchPositionDegrees() >= (positionInDegrees - 4);
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
    h.disablePid();
  }
}