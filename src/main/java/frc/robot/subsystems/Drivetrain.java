/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.DriveWithJoy;
import frc.robot.tekerz.utilities.L;

public class Drivetrain extends Subsystem {
  CANSparkMax 
    rightMotorLead = RobotMap.Sparks.rightMotorLead,
    rightMotorFollower = RobotMap.Sparks.rightMotorFollower,
    leftMotorLead = RobotMap.Sparks.leftMotorLead,
    leftMotorFollower = RobotMap.Sparks.leftMotorFollower;

  PigeonIMU ageSwine = RobotMap.Pigeon.imu;

  CANEncoder
    driveEncLeft = leftMotorLead.getEncoder(),
    driveEncRight = rightMotorLead.getEncoder();

  public static double
  //TODO: fix this number
    TICKS_PER_INCH = 0.5;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoy());
  }

  public Drivetrain() {
    rightMotorLead.restoreFactoryDefaults();
    rightMotorFollower.restoreFactoryDefaults();
    leftMotorLead.restoreFactoryDefaults();
    leftMotorFollower.restoreFactoryDefaults();

    rightMotorFollower.follow(rightMotorLead);
    leftMotorFollower.follow(leftMotorLead);

    rightMotorLead.setInverted(true);
    rightMotorFollower.setInverted(true);

    startBrakeMode();

    rightMotorLead.setClosedLoopRampRate(0.5);
    rightMotorLead.setOpenLoopRampRate(0.5);
    leftMotorLead.setClosedLoopRampRate(0.5);
    leftMotorLead.setOpenLoopRampRate(0.5);
  }

  public void setWheelSpeed(double leftSpeed, double rightSpeed){
    rightMotorLead.set(rightSpeed);
    leftMotorLead.set(leftSpeed);
  }

  public void arcadeDrive(double straightSpeed, double turnSpeed){
    double leftSpeed = straightSpeed + turnSpeed;
    double rightSpeed = straightSpeed - turnSpeed;

    setWheelSpeed(leftSpeed, rightSpeed);
  }

  public void startBrakeMode(){
    rightMotorLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotorLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }
  /**
   * a method to check the encoder vaues of the drivetrain wheels, 1 at a time.
   * 
   * @param leftEnc True to get the left value, false to get the right
   * @return
   */
  public double getEnc(boolean leftEnc){
    if(leftEnc){
      return driveEncLeft.getPosition();
    } else{
      return driveEncRight.getPosition();
    }
  }

  public double getAngle(){
    return ageSwine.getFusedHeading();
  }

  public void log() {
    // L.ogSD("Left Drive Encoder", getEnc(true));
    // L.ogSD("Right Drive Encoder", getEnc(false));
    L.ogSD("Robot Angle", getAngle());
    // L.ogSD("Drivetrain", this);
  }
}