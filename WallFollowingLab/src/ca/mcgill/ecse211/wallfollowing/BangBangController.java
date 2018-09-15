package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;


public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    
    
    
    int dist_ERR = bandCenter - distance;
    
    if (Math.abs(dist_ERR) <= bandwidth)
    {
    	WallFollowingLab.leftMotor.setSpeed(185);
    	WallFollowingLab.rightMotor.setSpeed(185);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if (dist_ERR > 0)
    {
    	
    	if (distance < 7)
    	{
    		WallFollowingLab.leftMotor.setSpeed(115);
    		WallFollowingLab.rightMotor.setSpeed(140);
    		WallFollowingLab.leftMotor.backward();
    		WallFollowingLab.rightMotor.backward();
    	}
    	//right turn
    	else 
    	{
    		WallFollowingLab.leftMotor.setSpeed(210);
    		WallFollowingLab.rightMotor.setSpeed(20);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.backward();
    	}
    }
    
    else
    {
    	//left turn
    	WallFollowingLab.leftMotor.setSpeed(120);
    	WallFollowingLab.rightMotor.setSpeed(210);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
