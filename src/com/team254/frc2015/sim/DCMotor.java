package com.team254.frc2015.sim;

/**
 * Simulates a DC motor using a simplified model appropriate for systems with
 * slow time constants.
 * 
 * @author jared
 */
public class DCMotor {
    // Motor constants
    protected final double m_kt;
    protected final double m_kv;
    protected final double m_resistance;
    protected final double m_motor_inertia;

    // Current motor state
    protected double m_position;
    protected double m_velocity;
    protected double m_current;

    // Convenience methods for constructing common motors.
    static DCMotor makeRS775() {
        final double KT = 0.009; // 9 mNm / A
        final double KV = 1083.0 * (Math.PI * 2.0) / 60.0; // 1083 rpm/V in
                                                           // rad/sec/V
        final double RESISTANCE = (18.0 / 130.0); // Rated for 130A stall @ 18V
        final double INERTIA = 1.20348237e-5; // 127g cylinder @ 1.084" diameter
        return new DCMotor(KT, KV, RESISTANCE, INERTIA);
    }

    static DCMotor makeRS550() {
        final double KT = 0.004862;
        final double KV = 1608.0 * (Math.PI * 2.0) / 60.0;
        final double RESISTANCE = (12.0 / 85.0);
        final double INERTIA = 0; // TODO(jared): Measure this
        return new DCMotor(KT, KV, RESISTANCE, INERTIA);
    }

    /*
     * Make a transmission.
     * 
     * @param motors The motor type attached to the transmission.
     * @param num_motors The number of motors in this transmission.
     * @param gear_reduction The reduction of the transmission.
     * @param efficiency The efficiency of the transmission.
     * @return A DCMotor representing the combined transmission.
     */
    static DCMotor makeTransmission(DCMotor motor, int num_motors,
            double gear_reduction, double efficiency) {
        return new DCMotor(num_motors * gear_reduction * efficiency
                * motor.m_kt, motor.m_kv / gear_reduction, motor.m_resistance
                / num_motors, motor.m_motor_inertia * num_motors
                * gear_reduction * gear_reduction);
    }

    /**
     * Simulate a simple DC motor.
     * 
     * @param kt
     *            Torque constant (N*m / amp)
     * @param kv
     *            Voltage constant (rad/sec / V)
     * @param resistance
     *            (ohms)
     * @param inertia
     *            (kg*m^2)
     */
    public DCMotor(double kt, double kv, double resistance, double inertia) {
        m_kt = kt;
        m_kv = kv;
        m_resistance = resistance;
        m_motor_inertia = inertia;

        reset(0, 0, 0);
    }

    /**
     * Simulate a simple DC motor.
     * 
     * @param kt
     *            Torque constant (N*m / amp)
     * @param kv
     *            Voltage constant (rad/sec / V)
     * @param resistance
     *            (ohms)
     */
    public DCMotor(double kt, double kv, double resistance) {
        this(kt, kv, resistance, 0);
    }

    /**
     * Reset the motor to a specified state.
     * 
     * @param position
     * @param velocity
     * @param current
     */
    public void reset(double position, double velocity, double current) {
        m_position = position;
        m_velocity = velocity;
        m_current = current;
    }

    /**
     * Simulate applying a given voltage and load for a specified period of
     * time.
     * 
     * @param applied_voltage
     *            Voltage applied to the motor (V)
     * @param load
     *            Load applied to the motor (kg*m^2)
     * @param acceleration
     *            The external torque applied (ex. due to gravity) (N*m)
     * @param timestep
     *            How long the input is applied (s)
     */
    public void step(double applied_voltage, double load,
            double external_torque, double timestep) {
        /*
         * Using the 971-style first order system model. V = I * R + Kv * w
         * torque = Kt * I
         * 
         * V = torque / Kt * R + Kv * w torque = J * dw/dt + external_torque
         * 
         * dw/dt = (V - Kv * w) * Kt / (R * J) - external_torque / J
         */
        load += m_motor_inertia;
        double acceleration = (applied_voltage - m_velocity / m_kv) * m_kt
                / (m_resistance * load) + external_torque / load;
        m_velocity += acceleration * timestep;
        m_position += m_velocity * timestep + .5 * acceleration * timestep
                * timestep;
        m_current = load * acceleration * Math.signum(applied_voltage) / m_kt;
    }

    public double getPosition() {
        return m_position;
    }

    public double getVelocity() {
        return m_velocity;
    }

    public double getCurrent() {
        return m_current;
    }
}
