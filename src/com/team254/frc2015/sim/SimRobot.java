package com.team254.frc2015.sim;

import java.util.TimerTask;

import com.team254.fakewpilib.SimRobotBase;
import com.team254.frc2015.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInputSetter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.EncoderSetter;
import edu.wpi.first.wpilibj.PWMObserver;
import edu.wpi.first.wpilibj.SolenoidStore;

public class SimRobot extends SimRobotBase {
    final double METERS_PER_INCH = 1.0 / 254.0;
    final double KG_PER_LB = 1.0 / 2.2;
    final double ELEVATOR_RADS_PER_INCH = Math.PI
            / (2.0 * Constants.kElevatorPulleyRadiusInches);

    PWMObserver bottom_carriage_observer = new PWMObserver(
            Constants.kBottomCarriageMotor1PWM);
    PWMObserver top_carriage_observer = new PWMObserver(
            Constants.kTopCarriageMotor1PWM);
    EncoderSetter bottom_carriage_encoder = new EncoderSetter(
            Constants.kBottomCarriageEncoderDIOA,
            Constants.kBottomCarriageEncoderDIOB);
    EncoderSetter top_carriage_encoder = new EncoderSetter(
            Constants.kTopCarriageEncoderDIOA,
            Constants.kTopCarriageEncoderDIOB);
    DigitalInputSetter bottom_carriage_hall_effect = new DigitalInputSetter(
    		Constants.kBottomCarriageHomeDIO);
    DigitalInputSetter top_carriage_hall_effect = new DigitalInputSetter(
    		Constants.kTopCarriageHomeDIO);

    double elevator_rads_per_tick = .25 * 2.0 * Math.PI
            / Constants.kElevatorEncoderCountsPerRev;  // 4x decoding

    ServoMechanism bottom_carriage = new ServoMechanism(
            bottom_carriage_observer, bottom_carriage_encoder,
            Constants.kBottomCarriageMotor1PDP,
            elevator_rads_per_tick, DCMotor.makeTransmission(
                    DCMotor.makeRS775(), 2, 10.5, .8), 0.0,
            new ServoMechanism.Limits(
                    Constants.kBottomCarriageMinPositionInches
                            * ELEVATOR_RADS_PER_INCH,
                    Constants.kBottomCarriageMaxPositionInches
                            * ELEVATOR_RADS_PER_INCH));
    ServoMechanism top_carriage = new ServoMechanism(top_carriage_observer,
            top_carriage_encoder, Constants.kTopCarriageMotor1PDP,
            elevator_rads_per_tick, DCMotor.makeTransmission(
                    DCMotor.makeRS775(), 2, 10.5, .8), 0.0,
            new ServoMechanism.Limits(Constants.kTopCarriageMinPositionInches
                    * ELEVATOR_RADS_PER_INCH,
                    Constants.kTopCarriageMaxPositionInches
                            * ELEVATOR_RADS_PER_INCH));

    @Override
    public void initSimRobot() {
        System.out.println("sim robot init lol");

    }

    @Override
    public void startRobotSim() {
        System.out.println("starting robot sim!");

        double updateRate = 500.0; // Hz

        // Initialize.
        bottom_carriage.reset(Constants.kBottomCarriageHomePositionInches
                * ELEVATOR_RADS_PER_INCH);
        top_carriage.reset(Constants.kTopCarriageHomePositionInches
                * ELEVATOR_RADS_PER_INCH);

        class Run extends TimerTask {
            long loop_counter;

            @Override
            public void run() {
                if (SolenoidStore.getSolenoid(0).get()) {
                    bottom_carriage.setLoad(10.0 * KG_PER_LB
                            * Constants.kElevatorPulleyRadiusInches
                            * METERS_PER_INCH); // 10 lbs
                    bottom_carriage.step(12.0, -9.8
                            / Constants.kElevatorPulleyRadiusInches,
                            1.0 / updateRate);
                }
                if (bottom_carriage.m_output.getEncoder() != null) {
                	int enc_val = bottom_carriage.m_output.getEncoder().get();
                	bottom_carriage_hall_effect.set(enc_val > -50 && enc_val < 50); // about half an inch of travel, homing sensor
                }

                if (SolenoidStore.getSolenoid(1).get()) {
                    top_carriage.setLoad(10.0 * KG_PER_LB
                            * Constants.kElevatorPulleyRadiusInches
                            * METERS_PER_INCH); // 10 lbs
                    top_carriage.step(12.0, -9.8
                            / Constants.kElevatorPulleyRadiusInches,
                            1.0 / updateRate);
                }
                if (top_carriage.m_output.getEncoder() != null) {
                	int enc_val = top_carriage.m_output.getEncoder().get();
                	top_carriage_hall_effect.set(enc_val > -50 && enc_val < 50); // about half an inch of travel, homing sensor
                }

                loop_counter++;
                if (loop_counter % 100 == 0) {
                    if (!top_carriage.withinLowerLimits()) {
                        System.err.println("Top carriage hit lower limit!");
                    }
                    if (!top_carriage.withinUpperLimits()) {
                        System.err.println("Top carriage hit upper limit!");
                    }
                    if (!bottom_carriage.withinLowerLimits()) {
                        System.err.println("Bottom carriage hit lower limit!");
                    }
                    if (!bottom_carriage.withinUpperLimits()) {
                        System.err.println("Bottom carriage hit upper limit!");
                    }
                }
            }
        }
        java.util.Timer timer = new java.util.Timer();
        timer.scheduleAtFixedRate(new Run(), 0,
                (long) ((1.0 / updateRate) * 1000.0));
    }

}
