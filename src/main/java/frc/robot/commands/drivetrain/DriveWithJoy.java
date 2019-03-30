/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoy extends Command {
  Drivetrain drivetrain = Robot.Subsystems.drivetrain;
  OI oi = Robot.oi;

  private double
    throttle,
    turn;

  public DriveWithJoy() {
    requires(drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    throttle = speedModifications(oi.getRightStickYDip(), 5);
    turn = speedModifications(oi.getLeftStickXDip(), 5);
    
    // turn = lowerTurnByThrottle(turn, throttle);

    drivetrain.arcadeDrive(throttle, turn);
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

  private double speedModifications(double speed, int exponent) {
    if (Math.abs(speed) < 0.1) {
      speed = 0;
    } else if (Math.abs(speed) > 0.9) {
      speed = 0.9 * (Math.abs(speed) / speed);
    }
    // if (speed < 0.0){
    //   speed = -Math.pow(speed, exponent);
    // } else{
    //   speed = Math.pow(speed, exponent);
    // }
    speed = Math.pow(speed, exponent);
    if (Robot.Subsystems.elevator.elevatorTicks < -900 && Math.abs(speed) > .15) {
      speed = 0.15 * (Math.abs(speed) / speed);
    }
    // if (Math.abs(speed) > 0.9) {
    //   speed = 0.9 * (Math.abs(speed) / speed);
    // }
    return speed;
  }

  final double koolKidKonstant = 1.0;

  private double lowerTurnByThrottle(double turn, double throttle){
    return turn * (1 - (Math.abs(throttle) * koolKidKonstant));
  }

  private double johnScale(double speed){
    double direction = Math.abs(speed) / speed;
      if(Math.abs(speed) < 0.8){
        return (speed / 2.0) * direction;
      }
      else{
        return speed * 3.0 - (2.0 * direction);
      }
  }
}
