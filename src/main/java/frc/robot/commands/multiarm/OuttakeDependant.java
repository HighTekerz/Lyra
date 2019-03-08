/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.multiarm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MultiArm;

public class OuttakeDependant extends Command {
  
  MultiArm m = Robot.Subsystems.multiArm;

  private boolean hpKickoff;

  public OuttakeDependant() {
    requires(m);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (m.getHPFlapIsUp()){
      hpKickoff = true;
    }
    else{
      hpKickoff = false;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(hpKickoff){
      m.setFingerDown();
      m.hPPusherOut();
    } 
    else{
      m.runIntake(0.5);
    }
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
    m.hPPusherIn();
    m.runIntake(0.0);
  }
}
