package com.team254.frc2015.sim;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.EncoderSetter;
import edu.wpi.first.wpilibj.PWMObserver;
import edu.wpi.first.wpilibj.Victor;

public class TestServoMechanism {
	@Test
	public void testServoMechanism() {
		// Make the motor and encoder.
		Victor motor = new Victor(1);
		Encoder encoder = new Encoder(1, 2);
		final double encoder_distance_per_pulse = 0.001;
		
		// Make a servo mechanism.
		DCMotor transmission = DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 10.0, 0.8);
		// Load is nominally 1kg*m^2
		ServoMechanism mechanism = new ServoMechanism(new PWMObserver(1), new EncoderSetter(1, 2),
				encoder_distance_per_pulse, transmission, 1.0, new ServoMechanism.Limits(0.0, 100.0));
	
		// Check against limits.
		assert(mechanism.withinLowerLimit());
		assert(mechanism.withinUpperLimits());
		
		// Drive the load down.
		motor.set(-1.0);
		mechanism.step(12.0, 0.0, 0.01);
		mechanism.step(12.0, 0.0, 0.01);
		mechanism.step(12.0, 0.0, 0.01);
		
		assertFalse(mechanism.withinLowerLimit());
		assert(mechanism.withinUpperLimits());
		
		// Check encoder went in reverse.
		assert(encoder.getRaw() < 0);
		
		// Reset the mechanism.
		mechanism.reset(0.0);
		assertEquals(encoder.getRaw(), 0);
		assert(mechanism.withinLowerLimit());
		
		// Simple 100Hz PD controller.
		double last_error = 50.0;
		for (int i = 0; i < 1000; ++i) {
			final double setpoint = 50.0;
			final double proportional_gain = 0.1;
			final double derivative_gain = 0.1;
			double error = setpoint - encoder.getRaw() * encoder_distance_per_pulse;
			double derivative = (error - last_error) / .01;
			last_error = error;
			motor.set(proportional_gain * error + derivative_gain * derivative);
			mechanism.step(12.0, 0.0, 0.01);
			if (i % 10 == 0) {
				System.out.println("Time: " + i * .01 + ", Error: " + error + ", Command: " + motor.get());
			}
		}
		assertEquals(encoder.getRaw() * encoder_distance_per_pulse, 50.0, 1E-3);
	}

}
