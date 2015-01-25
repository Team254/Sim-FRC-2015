package com.team254.frc2015.sim;

import edu.wpi.first.wpilibj.EncoderSetter;
import edu.wpi.first.wpilibj.PDP;
import edu.wpi.first.wpilibj.PWMObserver;

/**
 * Simulate a closed-loop position controlled mechanism.
 * @author jared
 */
public class ServoMechanism {
	protected PWMObserver m_input;
	protected EncoderSetter m_output;
	protected double m_encoder_distance_per_tick;
	protected DCMotor m_model;
	protected int m_pdp_channel;
	protected double m_load;
	protected Limits m_limits;
	
	public static class Limits {
		public Limits(double min, double max) {
			min_position = min;
			max_position = max;
		}
		public double min_position = -1E9;
		public double max_position = 1E9;
	}
	
	/**
	 * Create a new ServoMechanism.
	 * 
	 * @param input The source of the PWMs controlling the mechanism.
	 * @param output The position and rate of the mechanism will be written to this object.
	 * @param pdp_channel The channel # of the PDP slot for the motor.
	 * @param encoder_distance_per_tick How to convert between encoder units and real world distance.
	 * @param model The DCMotor representing the system dynamics.
	 * @param load The applied load to the mechanism, in kg*m^2
	 * @param limits Minimum and maximum position limits for the mechanism.
	 */
	public ServoMechanism(PWMObserver input,
			EncoderSetter output,
			int pdp_channel,
			double encoder_distance_per_tick,
			DCMotor model,
			double load,
			Limits limits) {
		m_input = input;
		m_output = output;
		m_pdp_channel = pdp_channel;
		m_encoder_distance_per_tick = encoder_distance_per_tick;
		m_model = model;
		m_load = load;
		m_limits = limits;
	}
	
	/**
	 * Set the load currently being moved by the mechanism.
	 * @param load The current mechanism load (in kg*m^2)
	 */
	public void setLoad(double load) {
		m_load = load;
	}
	
	/**
	 * Reset the position (and encoder value).
	 * @param position The new position of the mechanism.
	 */
	public void reset(double position) {
		m_model.reset(position, 0.0, 0.0);
		m_output.set(0);
		PDP.getInstance().setCurrent(m_pdp_channel, 0);
	}
	
	/**
	 * Simulate the mechanism's motion over a short time horizon.
	 * @param battery_voltage The current battery voltage (V).
	 * @param acceleration The external acceleration being experienced by the mechanism at
	 * the motor (ex. due to gravity) (rad/s^2).
	 * @param timestep The simulation period (s).
	 */
	public void step(double battery_voltage, double acceleration, double timestep) {
		double command = m_input.get() * battery_voltage;
		m_model.step(command, m_load, acceleration, timestep);
		m_output.set((int)(m_model.getPosition() / m_encoder_distance_per_tick));
		PDP.getInstance().setCurrent(m_pdp_channel, m_model.getCurrent());
	}
	
	/**
	 * Check the lower position limit.
	 * @return True if the current position is greater than or equal to the minimum.
	 */
	public boolean withinLowerLimits() {
		return m_model.getPosition() >= m_limits.min_position;
	}
	
	/**
	 * Check the upper position limit.
	 * @return True if the current position is less than or equal to the maximum.
	 */
	public boolean withinUpperLimits() {
		return m_model.getPosition() <= m_limits.max_position;
	}
}
