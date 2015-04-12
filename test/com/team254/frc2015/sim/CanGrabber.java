package com.team254.frc2015.sim;

import static org.junit.Assert.*;

import org.junit.Test;

public class CanGrabber {

    @Test
    public void test() {
        DCMotor motor = DCMotor.makeTransmission(DCMotor.makeRS775(), 2, 80.0,
                .8);
        double total_time = 0.0;
        double arm_com = 1.0;
        double arm_mass = .3;
        double battery_r = 0.018;
        double constant_spring_assist_torque = 0.0; // torque provided through
                                                     // the whole range of
                                                     // motion
        double over_center_spring_assist_torque = 0.0; // torque provided
                                                         // proportional to sin
                                                         // of angle (direction
                                                         // of gravity)
        // lower
        while (motor.getPosition() < Math.PI / 2 && total_time < 1.0) {
            double gravity_torque = 9.8 * arm_com * arm_mass
                    * Math.sin(motor.getPosition());
            double current = motor.getCurrent() * 2; // Two peacocks
            motor.step(
                    12.0 - current * battery_r,
                    .4,
                    gravity_torque + constant_spring_assist_torque
                            + over_center_spring_assist_torque
                            * Math.sin(motor.getPosition()), 0.0001);
            total_time += .0001;
        }
        System.out.println("Time: " + total_time + ", Angle: "
                + motor.getPosition());

        // raise
        total_time = 0.0;
        while (motor.getPosition() > 0 && total_time < 1.0) {
            double gravity_torque = 9.8 * arm_com * arm_mass
                    * Math.sin(motor.getPosition());
            double current = motor.getCurrent() * 2; // Two peacocks
            motor.step(
                    -12.0 + current * battery_r,
                    .4,
                    gravity_torque + constant_spring_assist_torque
                            + over_center_spring_assist_torque
                            * Math.sin(motor.getPosition()), 0.0001);
            total_time += .0001;
        }
        if (total_time >= 1.0) {
            System.out.println("CAN NOT RAISE ARM!");
        }
        System.out.println("Time: " + total_time + ", Angle: "
                + motor.getPosition());
    }
}
