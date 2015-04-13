package com.team254.frc2015.sim;

import static org.junit.Assert.*;

import org.junit.Test;

public class CanGrabber {

    @Test
    public void test() {
        // The transmission to test.
        DCMotor motor = DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 77.0,
                .8);
        
        // INPUTS
        final double battery_voltage = 12.8;
        final double arm_com = .82;  // m
        final double arm_mass = .2;  // kg
        final double battery_r = 0.018;  // ohms
        // torque provided constantly over the entire range of motion
        final double constant_spring_assist_torque = 0.0;  // N*m
        // torque provided varying with sin of angle (same effect as gravity)
        // due to over-center.
        final double over_center_spring_assist_torque = 0.0;  // N*m
        
        double total_time = 0.0;
        double arm_inertia = arm_mass * arm_com * arm_com;
        // lower
        while (motor.getPosition() < Math.PI / 2 && total_time < 1.0) {
            double gravity_torque = 9.8 * arm_com * arm_mass
                    * Math.sin(motor.getPosition());
            double current = motor.getCurrent() * 2; // Two peacocks
            motor.step(
                    battery_voltage - current * battery_r,
                    arm_inertia,
                    gravity_torque + constant_spring_assist_torque
                            + over_center_spring_assist_torque
                            * Math.sin(motor.getPosition()), 0.0001);
            total_time += .0001;
        }
        System.out.println("Time to lower mechanism 90 degrees: " + total_time);

        // raise
        total_time = 0.0;
        while (motor.getPosition() > 0 && total_time < 1.0) {
            double gravity_torque = 9.8 * arm_com * arm_mass
                    * Math.sin(motor.getPosition());
            double current = motor.getCurrent() * 2; // Two peacocks
            motor.step(
                    -battery_voltage + current * battery_r,
                    arm_inertia,
                    gravity_torque + constant_spring_assist_torque
                            + over_center_spring_assist_torque
                            * Math.sin(motor.getPosition()), 0.0001);
            total_time += .0001;
        }
        if (total_time >= 1.0) {
            System.out.println("CAN NOT RAISE ARM!");
        } else {
            System.out.println("Time to raise mechanism back to start angle: " + total_time);
        }
    }
}
