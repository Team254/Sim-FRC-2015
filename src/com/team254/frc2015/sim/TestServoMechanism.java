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
		final double cpr = 360.0;
		final double encoder_angular_distance_per_pulse = 2.0*Math.PI / cpr;
		final double pulley_radius = 0.0143256;  // .564 in (18T 5mm pitch)
		// arc length = 2*pi*r*central_angle/360
		final double encoder_linear_distance_per_pulse = pulley_radius * 2.0 * Math.PI / cpr;
		
		// Make a servo mechanism.
		DCMotor transmission = DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 10.5, 0.8);
		// Load is nominally 1kg*m^2
		ServoMechanism mechanism = new ServoMechanism(new PWMObserver(1), new EncoderSetter(1, 2),
				encoder_angular_distance_per_pulse, transmission, .007, new ServoMechanism.Limits(0.0, 1.8));
	
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
		final double period = 0.01;
		final double setpoint = 1.5;
		final double proportional_gain = 0.9;
		final double derivative_gain = 0.01;
		double last_error = 1.5;
		int last_encoder = 0;
		for (int i = 0; i < 1000; ++i) {
			double error = setpoint - encoder.getRaw() * encoder_linear_distance_per_pulse;
			double derivative = (error - last_error) / period;
			last_error = error;
			motor.set(proportional_gain * error + derivative_gain * derivative);
			last_encoder = encoder.getRaw();
			mechanism.step(12.0, 0.0, period);
			double velocity = (encoder.getRaw() - last_encoder) * encoder_linear_distance_per_pulse / period;
			if (i % 10 == 0) {
				System.out.println("Time: " + i * period + ", Error: " + error + ", Command: " + motor.get() + ", Velocity: " + velocity);
			}
		}
		assertEquals(encoder.getRaw() * encoder_linear_distance_per_pulse, 1.5, 1E-3);
	}

}
