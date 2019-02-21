/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static class Talons {
    public static TalonSRX 
      habLifterRollingLead = new TalonSRX(7),
      habLifterRollingFollower = new TalonSRX(8),
      liftLead = new TalonSRX(9),
      liftFollower = new TalonSRX(10),
      intake = new TalonSRX(11);

  }

  public static class Sparks {
    public static CANSparkMax 
      rightMotorLead = new CANSparkMax(1, MotorType.kBrushless),
      rightMotorFollower = new CANSparkMax(1, MotorType.kBrushless),
      leftMotorLead = new CANSparkMax(1, MotorType.kBrushless),
      leftMotorFollower = new CANSparkMax(1, MotorType.kBrushless),
      habLifterArmsLead = new CANSparkMax(1, MotorType.kBrushless),
      habLifterArmsFollower = new CANSparkMax(1, MotorType.kBrushless);

  }

  public static class Pneumatics {
    public static Solenoid
      intakeFlap = new Solenoid(30,0),
      hPFlap = new Solenoid(30,1),
      hPPusher = new Solenoid(30,2),
      hPStabilizer = new Solenoid(30,3),
      habLifterLegs1 = new Solenoid(31,0),
      habLifterLegs2 = new Solenoid(31,1);
  }

  public static class Analog {
    public static AnalogInput
      sensor0 = new AnalogInput(0),
      sensor1 = new AnalogInput(1),
      sensor2 = new AnalogInput(2),
      sensor3 = new AnalogInput(3),
      sensor4 = new AnalogInput(4),
      sensor5 = new AnalogInput(5),
      sensor6 = new AnalogInput(6),
      sensor7 = new AnalogInput(7);
  }

  public static class Pigeon {
    public static PigeonIMU
      imu = new PigeonIMU(RobotMap.Talons.intake);
  }

  public static class Switches {
    public static DigitalInput
      cargoSensor = new DigitalInput(0);
  }

}
