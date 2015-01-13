package com.team254.frc2015.sim;

import com.team254.fakewpilib.SimRobotBase;

import edu.wpi.first.wpilibj.EncoderSetter;
import edu.wpi.first.wpilibj.PWMObserver;
import edu.wpi.first.wpilibj.Timer;


public class SimRobot extends SimRobotBase {

	EncoderSetter es = new EncoderSetter(0,1);
	PWMObserver p1 = new PWMObserver(0);
	
  @Override
  public void initSimRobot() {
    System.out.println("sim robot init lol");
    
  }

  @Override
  public void startRobotSim() {
	  System.out.println("starting robot sim!");
	  
	  double updateRate = 100.0; //Hz
	  double currentPosition = 0;
	  double speedOfActuator = 4.0; // Inches per second
	  
	  double scaleFactorToEncoder = 1250;
	 
	  while (true) {
		  currentPosition += ((p1.get() * speedOfActuator) / updateRate);
		  
		  int newEnc = (int)(currentPosition * scaleFactorToEncoder);
		  
		  es.set(newEnc);
		  
		  Timer.delay(1.0 / updateRate);
	  }
  }

}
