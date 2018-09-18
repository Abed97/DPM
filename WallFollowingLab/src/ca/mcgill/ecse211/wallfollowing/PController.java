package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    
    final int GAIN = 10;
    if (distance > 50) {
    	distance = 50;
    } 
    int dist_ERR = 30 - distance;
    
    int constant = Math.abs(GAIN * dist_ERR);
    
    
    
    if (Math.abs(dist_ERR) <= 1)
    {
    	WallFollowingLab.leftMotor.setSpeed(200);
    	WallFollowingLab.rightMotor.setSpeed(200);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if (dist_ERR > 0)
    {
    	
    	if (distance <= 7)
    	{
    		WallFollowingLab.leftMotor.setSpeed(115);
    		WallFollowingLab.rightMotor.setSpeed(160);
    		WallFollowingLab.leftMotor.backward();
    		WallFollowingLab.rightMotor.backward();
    	}
    	//right turn
    	else 
    	{
    		WallFollowingLab.leftMotor.setSpeed(270);
    		WallFollowingLab.rightMotor.setSpeed(120 - constant);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    	}
    }
    
    else if (dist_ERR < 0)
    {
    	//left turn
    	WallFollowingLab.leftMotor.setSpeed(100 - constant);
    	WallFollowingLab.rightMotor.setSpeed(150);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
