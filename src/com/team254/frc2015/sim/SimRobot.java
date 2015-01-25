package com.team254.frc2015.sim;

import com.team254.fakewpilib.SimRobotBase;
import com.team254.frc2015.sim.ServoMechanism.Limits;

import edu.wpi.first.wpilibj.EncoderSetter;
import edu.wpi.first.wpilibj.PWMObserver;
import edu.wpi.first.wpilibj.Timer;

public class SimRobot extends SimRobotBase {
	final double INCHES_TO_METERS = 1.0/254.0;

	PWMObserver bottom_carriage_observer = new PWMObserver(6);
	PWMObserver top_carriage_observer = new PWMObserver(1);
	EncoderSetter bottom_carriage_encoder = new EncoderSetter(4, 5);
	EncoderSetter top_carriage_encoder = new EncoderSetter(6, 7);
	double elevator_encoder_distance_per_tick = 2.0 * Math.PI / 360.0;  // 360 CPR

	ServoMechanism bottom_carriage = new ServoMechanism(bottom_carriage_observer,
			bottom_carriage_encoder,
			6,
			elevator_encoder_distance_per_tick,
			DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 10.5, .8),
			0.0,
			new ServoMechanism.Limits(0.0, 60.0 * INCHES_TO_METERS)
			);
	ServoMechanism top_carriage = new ServoMechanism(bottom_carriage_observer,
			bottom_carriage_encoder,
			4,
			elevator_encoder_distance_per_tick,
			DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 10.5, .8),
			0.0,
			new ServoMechanism.Limits(0.0, 60.0 * INCHES_TO_METERS)
			);

	@Override
	public void initSimRobot() {
		System.out.println("sim robot init lol");

	}

	@Override
	public void startRobotSim() {
		System.out.println("starting robot sim!");

		double updateRate = 300.0; // Hz

		while (true) {
			Timer.delay(1.0 / updateRate);
		}
	}

}
