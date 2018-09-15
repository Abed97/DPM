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
    
    final int GAIN = 2;
    int dist_ERR = bandCenter - distance;
    int cons = GAIN * dist_ERR;
    
    //int new_LSPD = (GAIN * dist_ERR) + WallFollowingLab.leftMotor.getRotationSpeed();
    //int new_RSPD = (GAIN * dist_ERR) + WallFollowingLab.rightMotor.getRotationSpeed();
    
    if (Math.abs(dist_ERR) <= bandWidth ) 
    {
    	WallFollowingLab.leftMotor.setSpeed(185);
    	WallFollowingLab.rightMotor.setSpeed(185);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if (dist_ERR > 0)
    {
    	// turning right
    	WallFollowingLab.leftMotor.setSpeed(185 + cons);
    	WallFollowingLab.rightMotor.setSpeed(185 - cons);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    	
    }
    
    else
    {
    	// turning left
    	WallFollowingLab.leftMotor.setSpeed(185 + cons);
    	WallFollowingLab.rightMotor.setSpeed(185 - cons);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    
    /*
    else if (dist_ERR > 0)
    {
    	if (distance < 7)
    	{
    		WallFollowingLab.leftMotor.setSpeed(120);
    		WallFollowingLab.rightMotor.setSpeed(175);
    		WallFollowingLab.leftMotor.backward();
    		WallFollowingLab.rightMotor.backward();
    	}
    	//right turn
    	else 
    	{
    		WallFollowingLab.leftMotor.setSpeed(240);
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
    
    
    
    */
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
