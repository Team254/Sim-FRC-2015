package com.team254.frc2015.sim;

import static org.junit.Assert.*;

import org.junit.Test;

public class TestDCMotor {
	private final double EPS = 1E-9;

	@Test
	public void testGettersSetters() {
		// Make a new motor.
		DCMotor motor = new DCMotor(2.0, 10.0, 10.0);
		
		// Check initial conditions.
		assertEquals(motor.getPosition(), 0.0, EPS);
		assertEquals(motor.getVelocity(), 0.0, EPS);
		assertEquals(motor.getCurrent(), 0.0, EPS);
		
		// Reset initial conditions.
		motor.reset(1.0, 2.0, 3.0);
		assertEquals(motor.getPosition(), 1.0, EPS);
		assertEquals(motor.getVelocity(), 2.0, EPS);
		assertEquals(motor.getCurrent(), 3.0, EPS);
	}
	
	@Test
	public void testRS775() {
		DCMotor rs775 = DCMotor.makeTransmission(DCMotor.makeRS775(), 1, 10.0, 1.0);
		
		// Apply a positive voltage and small load.
		System.out.println("Voltage=6V, Load=.01 kg*m^2");
		for (int i = 0; i < 1000; ++i) {
			rs775.step(6.0, 0.01, 0.01);
			
			if (i % 100 == 0) {
				System.out.print("Time: " + 0.01*i);
				System.out.print(", Position: " + rs775.getPosition());
				System.out.print(", Velocity: " + rs775.getVelocity());
				System.out.println(", Current: " + rs775.getCurrent());
			}
		}
		// We expect negligible final current, and a final velocity of ~68.04 rad/sec.
		assertEquals(rs775.getCurrent(), 0.0, 1E-3);
		assertEquals(rs775.getVelocity(), 68.04, 1E-2);
	
		// Apply a larger voltage.
		System.out.println("Voltage=12V, Load=.01 kg*m^2");
		rs775.reset(0, 0, 0);
		for (int i = 0; i < 1000; ++i) {
			rs775.step(12.0, 0.01, 0.01);
			
			if (i % 100 == 0) {
				System.out.print("Time: " + 0.01*i);
				System.out.print(", Position: " + rs775.getPosition());
				System.out.print(", Velocity: " + rs775.getVelocity());
				System.out.println(", Current: " + rs775.getCurrent());
			}
		}
		// We expect negligible final current, and a final velocity of ~2 * 68.04 rad/sec.
		assertEquals(rs775.getCurrent(), 0.0, 1E-3);
		assertEquals(rs775.getVelocity(), 68.04*2, 1E-1);
		
		// Apply a larger load.
		System.out.println("Voltage=12V, Load=1.0 kg*m^2");
		rs775.reset(0, 0, 0);
		double start_current = 0;
		for (int i = 0; i < 1000; ++i) {
			rs775.step(12.0, 1.0, 0.01);
			if (i == 0) {
				start_current = rs775.getCurrent();
			}
			
			if (i % 100 == 0) {
				System.out.print("Time: " + 0.01*i);
				System.out.print(", Position: " + rs775.getPosition());
				System.out.print(", Velocity: " + rs775.getVelocity());
				System.out.println(", Current: " + rs775.getCurrent());
			}
		}
		double final_position = rs775.getPosition();
		double final_velocity = rs775.getVelocity();
		double final_current = rs775.getCurrent();
		
		// Go in reverse.
		System.out.println("Voltage=-12V, Load=1.0 kg*m^2");
		rs775.reset(0, 0, 0);
		for (int i = 0; i < 1000; ++i) {
			rs775.step(-12.0, 1.0, 0.01);
			
			if (i % 100 == 0) {
				System.out.print("Time: " + 0.01*i);
				System.out.print(", Position: " + rs775.getPosition());
				System.out.print(", Velocity: " + rs775.getVelocity());
				System.out.println(", Current: " + rs775.getCurrent());
			}
		}
		assertEquals(rs775.getCurrent(), final_current, EPS);
		
		
		// Add a second 775.
		rs775 = DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 10.0, 1.0);
		System.out.println("(2 motors) Voltage=12V, Load=1.0 kg*m^2");
		for (int i = 0; i < 1000; ++i) {
			rs775.step(12.0, 1.0, 0.01);
			if (i == 0) {
				// We expect to draw twice the starting current.
				assertEquals(rs775.getCurrent(), 2.0 * start_current, EPS);
			}
			
			if (i % 100 == 0) {
				System.out.print("Time: " + 0.01*i);
				System.out.print(", Position: " + rs775.getPosition());
				System.out.print(", Velocity: " + rs775.getVelocity());
				System.out.println(", Current: " + rs775.getCurrent());
			}
		}
		// We expect the two motor version to move faster than the single motor version.
		assert(rs775.getVelocity() - EPS > final_velocity);
		assert(rs775.getPosition() - EPS > final_position);
		final_position = rs775.getPosition();
		final_velocity = rs775.getVelocity();

		// Make it less efficient.
		rs775 = DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 10.0, 0.8);
		System.out.println("(2 motors, 80% efficient) Voltage=12V, Load=1.0 kg*m^2");
		for (int i = 0; i < 1000; ++i) {
			rs775.step(12.0, 1.0, 0.01);
			
			if (i % 100 == 0) {
				System.out.print("Time: " + 0.01*i);
				System.out.print(", Position: " + rs775.getPosition());
				System.out.print(", Velocity: " + rs775.getVelocity());
				System.out.println(", Current: " + rs775.getCurrent());
			}
		}
		// We expect the less efficient version to be slower.
		assert(rs775.getVelocity() + EPS < final_velocity);
		assert(rs775.getPosition() + EPS < final_position);
	}

}
