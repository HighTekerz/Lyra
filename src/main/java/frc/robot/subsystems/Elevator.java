/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.tekerz.PIDF;
import frc.robot.tekerz.utilities.L;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static double MOTOR_HOLD_VALUE = 0.1;
  public static double TICKS_PER_INCH = 360;

  TalonSRX liftLead = RobotMap.Talons.liftLead, liftFollower = RobotMap.Talons.liftFollower;

  double p = .0001, i = 0.0, d = 0.0, loopLengthInSeconds = .005;

  private final PIDOutput output = this::setElevatorPIDOutput;
  private final PIDSource input = new PIDSource(){
  
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }
  
    @Override
    public double pidGet() {
      return getElevatorPosition();
    }
  
    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  };

  private final PIDController pIDLoop=new PIDController(p,i,d,input,output,loopLengthInSeconds){@Override protected double calculateFeedForward(){return feedForwardAmount();}};

  public Elevator() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    liftLead.configAllSettings(config);
    liftFollower.configAllSettings(config);

    liftFollower.follow(liftLead);
  }

  private void setElevatorPIDOutput(double out) {
    liftLead.set(ControlMode.PercentOutput, out);
    L.ogSD("ARM PID ouput", out);
  }

  public double getElevatorPosition() {
    return liftLead.getSelectedSensorPosition();
  }

  public void setSetpoint(double setpoint) {
    this.pIDLoop.setSetpoint(setpoint);
  }

  public void enableElevator() {
    this.pIDLoop.enable();
  }

  public void disableElevator() {
    this.pIDLoop.disable();
  }

  private double feedForwardAmount() {
    // MULTIPLYU
    return MOTOR_HOLD_VALUE;
  }

  public void log() {

  }

  @Override
  protected void initDefaultCommand() {
  }
}
