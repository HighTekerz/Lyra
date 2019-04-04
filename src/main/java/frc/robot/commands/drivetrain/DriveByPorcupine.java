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

public class DriveByPorcupine extends Command {
  Drivetrain dt = Robot.Subsystems.drivetrain;
  double targetLeft; double targetRight; double speed; double rotation; boolean stopAtEnd    ;
                  Boolean isDone = false;
      double calcTargetLeft, calcTargetRight;

  public DriveByPorcupine(double targetLeft, double targetRight, double speed, double rotation, boolean stopAtEnd) {
    requires(dt);
    this.targetLeft = targetLeft;
    this.targetRight = targetRight;
    this.speed = speed;
    this.rotation = rotation;
    this.stopAtEnd = stopAtEnd;
  }

  @Override
  protected void initialize() {
    isDone = false;
    calcTargetLeft = targetLeft + dt.getEnc(true);
    calcTargetRight = targetRight + dt.getEnc(false);
  }

  @Override
  protected void execute() {
    double leftEnc = dt.getEnc(true);

    if (leftEnc < calcTargetLeft) {  // && dt.getEnc(false) < calcTargetRight) {
      if (stopAtEnd) {
        if (calcTargetLeft - leftEnc < 15.0) {
          double speedCalc = speed * (calcTargetLeft - leftEnc) / 15.0;
          if(speedCalc < .1) {
            speedCalc = 0.1;
          }
          dt.arcadeDrive(speedCalc, rotation *(calcTargetLeft - leftEnc) / 15.0);
        } else {
          dt.arcadeDrive(speed, rotation);
        }
      } else {
        dt.arcadeDrive(speed, rotation);
      }
    } else {
      if (stopAtEnd) {
        dt.arcadeDrive(0.0, 0.0);
      }
      isDone = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
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
