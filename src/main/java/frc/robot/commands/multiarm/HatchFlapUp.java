/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.multiarm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.tekerz.utilities.L;

public class HatchFlapUp extends Command {
  public HatchFlapUp() {
    requires(Robot.Subsystems.multiArm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.Subsystems.multiArm.hPFlapUp();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(timeSinceInitialized() > .25){
      return true;
    }
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
