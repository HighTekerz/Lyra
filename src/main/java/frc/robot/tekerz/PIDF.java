/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tekerz;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * PIDFConstantF is an implementation of a PIDF loop for a steady state F
 * to use: call start, then call loop regularly
 * 
 * how to set F:
 * 
 * Elevator that holds itself at setpoint at constant value:
 * set to motor output that holds posistion
 * 
 * System that requires no ouput at setpoint:
 * set to 0
 * 
 */
public class PIDF extends Thread {
    enum FType {
        CONSTANT,
        SPEED_DEPENDENT,
        GRAVITY_ARM
    }
    double p,i,d,f;
    PIDF.FType fType = FType.CONSTANT;
    double //system parameters
        iMax = Double.MAX_VALUE,
        setpoint = 0.0,
        motorFullReverse = -1.0,
        motorFullStop = 0.0,
        motorFullForward = 1.0,
        motorForwardRange = 1.0,
        rampRateMax = 0.0,
        sensorMin = 0.0,
        sensorMax = 0.0,
        sensorAtTop = 0.0,
        sensorCountPerDegree = 0.0;

    double //calculation assists
        error = 0.0, 
        accumulatedError = 0.0,
        previousError = 0.0,
        changeInError = 0.0,
        output = 0.0,
        lastOutput = 0.0,
        changeInOutputRate = 0.0,
        elapsedTime = 0.0,
        timeNow = 0.0,
        timeLast = 0.0,
        fPart = 0.0;

    Consumer<Double> outputMethod;
    Supplier<Double> feedbackMethod;

    public PIDF(double p, double i, double d) {
        this.setPIDF(p, i, d);
    }

    public void setPIDF(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void setOutputMethod(Consumer<Double> setter) {
        this.outputMethod = setter;
    }

    public void setOutputMethod(Consumer<Double> setter, double fullReverse, double fullStop, double fullForward) {
        this.motorFullForward = fullForward;
        this.motorFullStop = fullStop;
        this.motorFullReverse = fullReverse;
        this.motorForwardRange = fullForward - fullStop;
        this.outputMethod = setter;
    }

    public void setFeedbackMethod(Supplier<Double> getter, double minReading, double maxReading) {
        this.sensorMin = minReading;
        this.sensorMax = maxReading;
        this.feedbackMethod = getter;
    }

    public void setFConstant (double f) {
        this.fType = FType.CONSTANT;
        this.f = f;
    }

    public void setFGravityArm(double f, double sensorAtTop, double countPerDegree) {
        this.sensorAtTop = sensorAtTop;
        this.sensorCountPerDegree = countPerDegree;
        this.fType = FType.GRAVITY_ARM;
        this.f = f;
    }

    public void setRampRateInMS (double motorStopSpeed, double motorFullSpeed, double timeForFullRunInMS) {
        this.rampRateMax = (motorFullSpeed - motorStopSpeed) / timeForFullRunInMS;
    }

    public void setIMax(double iMax) {
        this.iMax = iMax;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public void start() {
        accumulatedError = 
            changeInOutputRate =
            previousError = 
            0.0;
        timeLast = time();
        while (true) {
            // the magic!
            this.outputMethod.accept(loop(this.feedbackMethod.get()));
        }
    }

    private double time() {
        return RobotController.getFPGATime() / 1000.0;
    }

    /**
     * 
     * @param sensorData what you are measuring
     * @return motor output
     */
    public double loop(double sensorData) {
        timeNow = time();
        elapsedTime = timeNow - timeLast;
        timeLast = timeNow; // save for next time

        error = setpoint - sensorData;

        accumulatedError += error * elapsedTime;
        if (accumulatedError > iMax) accumulatedError = iMax;

        changeInError = (error - previousError) / elapsedTime;

        switch (this.fType) {
            case SPEED_DEPENDENT:
                fPart = (sensorMax - this.setpoint) / (sensorMax - sensorMin) * this.motorFullForward;
                break;
            case GRAVITY_ARM:
                // might need to divide by error
                fPart = Math.sin(this.sensorAtTop - this.setpoint);
                break;
            case CONSTANT:
                fPart = f;
            default:
                break;
        }

        output = 
            p * error +
            i * accumulatedError +
            d * changeInError +
            fPart;

        changeInOutputRate = (output - lastOutput) / elapsedTime;

        // we need to change output to only increate the amount it is allowed in this length of time
        // and in the direction of change
        if (Math.abs(changeInOutputRate) > rampRateMax) {
            lastOutput +=
                (rampRateMax * elapsedTime) * // amount of change in this timeframe
                (changeInOutputRate / changeInOutputRate); // in the correct direction of change
        }

        return output;
    }
}
