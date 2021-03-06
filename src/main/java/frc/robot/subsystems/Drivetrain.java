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

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.DriveWithJaci;
import frc.robot.commands.drivetrain.DriveWithJoy;
import frc.robot.commands.drivetrain.StartBrakeMode;
import frc.robot.tekerz.utilities.L;
import jaci.pathfinder.Waypoint;

public class Drivetrain extends Subsystem {
  CANSparkMax 
    rightMotorLead = RobotMap.Sparks.rightMotorLead,
    rightMotorFollower = RobotMap.Sparks.rightMotorFollower,
    leftMotorLead = RobotMap.Sparks.leftMotorLead,
    leftMotorFollower = RobotMap.Sparks.leftMotorFollower;

  PigeonIMU ageSwine = RobotMap.Pigeon.imu;

  CANEncoder driveEncLeft = leftMotorLead.getEncoder(), driveEncRight = rightMotorLead.getEncoder();

  public static double
  // TODO: fix this number
  TICKS_PER_INCH = .2;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoy());
  }

  private double 
  rampRate = 0.35; // 0.1

  public Drivetrain() {
    prepareMotor(leftMotorLead);
    prepareMotor(leftMotorFollower);
    prepareMotor(rightMotorLead);
    prepareMotor(rightMotorFollower);

    rightMotorFollower.follow(rightMotorLead);
    leftMotorFollower.follow(leftMotorLead);

    rightMotorLead.setInverted(true);
    rightMotorFollower.setInverted(true);

    startBrakeMode();

    

  }

  private void prepareMotor(CANSparkMax motor){
    motor.restoreFactoryDefaults();
    motor.setClosedLoopRampRate(rampRate);
    motor.setSmartCurrentLimit(60);
  }

  public void setWheelSpeed(double leftSpeed, double rightSpeed) {
    rightMotorLead.set(rightSpeed);
    leftMotorLead.set(leftSpeed);
  }

  public void arcadeDrive(double straightSpeed, double turnSpeed) {
    double leftSpeed = straightSpeed + turnSpeed;
    double rightSpeed = straightSpeed - turnSpeed;

    setWheelSpeed(leftSpeed, rightSpeed);
  }

  public void startBrakeMode() {
    rightMotorLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotorLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotorFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
  } 

  public void startCoastMode() {
    rightMotorLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightMotorFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leftMotorLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leftMotorFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
  } 

  /**
   * a method to check the encoder vaues of the drivetrain wheels, 1 at a time.
   * 
   * @param leftEnc True to get the left value, false to get the right
   * @return Returns the ticks of the chosen encoder
   */
  public double getEnc(boolean leftEnc) {
    if (leftEnc) {
      return driveEncLeft.getPosition();
    } else {
      return driveEncRight.getPosition();
    }
  }
  public double getEncLSpeed() {
    return driveEncLeft.getVelocity();
  }
  
  public double getEncRSpeed() {
    return driveEncRight.getVelocity();
  }

  double[] yawPitchRollArray = new double[3];

  public double getAngle() {
    ageSwine.getYawPitchRoll(yawPitchRollArray);
    return yawPitchRollArray[0];
  }

  public double getPitch(){
    ageSwine.getYawPitchRoll(yawPitchRollArray);
    return yawPitchRollArray[1];
  }

  public double getRoll(){
    ageSwine.getYawPitchRoll(yawPitchRollArray);
    return yawPitchRollArray[2];
  }

  public void clearEncoders() {
    driveEncLeft.setPosition(0.0);
    driveEncRight.setPosition(0.0);
  }

  public void resetYaw(){
    ageSwine.setYaw(0.0);
  }

  public void log() {
    L.ogSD("left", getEnc(true));
    L.ogSD("Speed Left", getEncLSpeed());

    L.ogSD("Robot Angle", getAngle());
    L.ogSD("Robot Pitch", getPitch());
  }

  /*
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  * STUFF FOR PID 
  */
  double 
    p = 0.0001, 
    i = 0.0, 
    d = 0.0, 
    loopLengthInSeconds = .005;

  private final PIDOutput pIDLeftOutput = this::setPIDLeftOutput;
  private final PIDOutput pIDRightOutput = this::setPIDRightOutput;
  private final PIDSource pIDLeftinput = new PIDSource() {
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {}

    @Override
    public double pidGet() {
      return getPIDLeftPosition();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  };

  private final PIDSource pIDRightInput = new PIDSource() {
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {}

    @Override
    public double pidGet() {
      return getPIDRightPosition();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  };

  private final PIDController pIDLeftLoop = new PIDController(p, i, d, pIDLeftinput, pIDLeftOutput, loopLengthInSeconds);
  private final PIDController pIDRightLoop = new PIDController(p, i, d, pIDRightInput, pIDRightOutput, loopLengthInSeconds);
  private void setPIDLeftOutput(double out) {
    leftMotorLead.set(out);
  }

  private void setPIDRightOutput(double out) {
    rightMotorLead.set(out);
  }

  public double getPIDLeftPosition() {
    return driveEncLeft.getPosition();
  }

  public double getPIDRightPosition() {
    return driveEncRight.getPosition();
  }

  /**
   * 
   * @param setpointInDegrees the angle (in degrees) you want the arm to travel
   *                          to.
   */
  public void setPIDSetpoint(double leftSetpoint, double rightSetpoint) {
    this.pIDLeftLoop.setSetpoint(leftSetpoint);
    this.pIDRightLoop.setSetpoint(rightSetpoint);
  }

  public void enablePID() {
    this.pIDLeftLoop.enable();
    this.pIDRightLoop.enable();
  }

  public void disablePID() {
    this.pIDLeftLoop.disable();
    this.pIDRightLoop.disable();
  }

}