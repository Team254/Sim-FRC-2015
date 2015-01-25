package com.team254.frc2015.sim;

import java.util.TimerTask;

import com.team254.fakewpilib.SimRobotBase;

import edu.wpi.first.wpilibj.EncoderSetter;
import edu.wpi.first.wpilibj.PWMObserver;
import edu.wpi.first.wpilibj.SolenoidStore;

public class SimRobot extends SimRobotBase {
	final double METERS_PER_INCH = 1.0 / 254.0;
	final double KG_PER_LB = 1.0 / 2.2;
	final double ELEVATOR_RADS_PER_INCH = Math.PI / (2.0 * .564);
	final double bottom_carriage_home_height = 20.0 * ELEVATOR_RADS_PER_INCH;
	final double top_carriage_home_height = 30.0 * ELEVATOR_RADS_PER_INCH;

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
			new ServoMechanism.Limits(0.0, 60.0 * ELEVATOR_RADS_PER_INCH)
			);
	ServoMechanism top_carriage = new ServoMechanism(top_carriage_observer,
			top_carriage_encoder,
			4,
			elevator_encoder_distance_per_tick,
			DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 10.5, .8),
			0.0,
			new ServoMechanism.Limits(10.0 * ELEVATOR_RADS_PER_INCH, 70.0 * ELEVATOR_RADS_PER_INCH)
			);

	@Override
	public void initSimRobot() {
		System.out.println("sim robot init lol");

	}

	@Override
	public void startRobotSim() {
		System.out.println("starting robot sim!");

		double updateRate = 500.0; // Hz
		
		// Initialize.
		bottom_carriage.reset(bottom_carriage_home_height);
		top_carriage.reset(top_carriage_home_height);

		class Run extends TimerTask {
			@Override
			public void run() {
				if (SolenoidStore.getSolenoid(0).get()) {
					bottom_carriage.setLoad(10.0 * KG_PER_LB * .564 * METERS_PER_INCH);  // 10 lbs
					bottom_carriage.step(12.0, -9.8, 1.0 / updateRate);
				} else {
					bottom_carriage.setLoad(100000.0);  // brake applied
					bottom_carriage.step(12.0, 0.0, 1.0 / updateRate);
				}

				if (SolenoidStore.getSolenoid(1).get()) {
					top_carriage.setLoad(10.0 * KG_PER_LB * .564 * METERS_PER_INCH);  // 10 lbs
					top_carriage.step(12.0, -9.8, 1.0 / updateRate);
				} else {
					top_carriage.setLoad(100000.0);  // brake applied
					top_carriage.step(12.0, 0.0, 1.0 / updateRate);
				}
			}
		}
		java.util.Timer timer = new java.util.Timer();
		timer.scheduleAtFixedRate(new Run(), 0, (long)((1.0 / updateRate) * 1000.0));
		while(true) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

}
