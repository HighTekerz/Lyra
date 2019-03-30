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

public class KeepLevelUntilAnAngle extends Command {

  double 
    extraDegrees,
    maximumOutput;

  HabLifter hL = Robot.Subsystems.habLifter;
  
  /**
   * @param extraDegrees degrees past level you'd like to stop at
   */
  public KeepLevelUntilAnAngle(double extraDegrees, double maximumOutput){
    requires(hL);
    this.extraDegrees = extraDegrees;
    this.maximumOutput = maximumOutput;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
    hL.runOnPidgeon(true);
    hL.setArmSetpoint(hL.getArmOrPitchPositionDegrees() + extraDegrees, maximumOutput);
    L.og("setpoint in degrees: " + hL.getArmOrPitchPositionDegrees() + extraDegrees);
    hL.enablePid();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.Subsystems.habLifter.getDegrees() < -137){
      return true;
    }
    else{
      return false;
    }
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
    hL.runOnPidgeon(false);
    hL.disablePid();
  }
}
