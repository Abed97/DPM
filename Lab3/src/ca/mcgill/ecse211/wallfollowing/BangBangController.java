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


/*** This class implements the bang bang controller for Lab1 on the EV3 platform.
 * 
 * @authorAbedAtassi&HyunSuAn
 */
public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  /*** This method starts the bang bang-controller based 
   * on user button input.
   * 
   * @param bandwidth, bandCenter, motorLow, motorHigh.
   * */
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
    
    // The robot goes forward
    if (Math.abs(dist_ERR) <= bandwidth)
    {
    	WallFollowingLab.leftMotor.setSpeed(200);
    	WallFollowingLab.rightMotor.setSpeed(200);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if (dist_ERR > 0)
    {
    	// If the distance is closer than 8 cm the robot goes backward to the right
    	if (distance < 8)
    	{
    		WallFollowingLab.leftMotor.setSpeed(115);
    		WallFollowingLab.rightMotor.setSpeed(160);
    		WallFollowingLab.leftMotor.backward();
    		WallFollowingLab.rightMotor.backward();
    	}
    	//right turn
    	else if (dist_ERR > 0)
    	{
    		WallFollowingLab.leftMotor.setSpeed(210);
    		WallFollowingLab.rightMotor.setSpeed(100);
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
