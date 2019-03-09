/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.tekerz.utilities.L;

public class TestSolenoiods extends Command {
  private Solenoid testSolenoid;

  private TestSolenoiods() {
    requires(Robot.Subsystems.multiArm);
    requires(Robot.Subsystems.habLifter);
  }

  public TestSolenoiods(Solenoid testSolenoid, String name){
    this();
    this.setName(name);
    this.testSolenoid = testSolenoid;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.oi.getButtonALip()){
      testSolenoid.set(true);
    } else if(Robot.oi.getButtonBLip()){
      testSolenoid.set(false);
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
    L.ogCmdEnd(this);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    L.ogCmdInterrupted(this);
  }
}
