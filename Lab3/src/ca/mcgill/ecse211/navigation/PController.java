package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.wallfollowing.UltrasonicController;
import ca.mcgill.ecse211.navigation.lab3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/*** This class implements the P controller for Lab1 on the EV3 platform.
 * 
 * @authorAbedAtassi&HyunSuAn
 */
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
    
    /*
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    */
    
  }
  
  /*** This method starts the p-controller based 
   * on user button input.
   * 
   * @param distance.
   * */
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

    
    // Gain constant
    final int GAIN = 10;
    
    // When distance is too high, cap it to prevent overflow in diff
    if (distance > 50) {
    	distance = 50;
    } 
    int dist_ERR = 30 - distance;
    
    int diff = Math.abs(GAIN * dist_ERR);
    
    
    // The robot goes forward
    if (Math.abs(dist_ERR) <= 1)
    {
    	lab3.leftMotor.setSpeed(200);
    	lab3.rightMotor.setSpeed(200);
    	lab3.leftMotor.forward();
    	lab3.rightMotor.forward();
    }
    
    else if (dist_ERR > 0)
    {
    	//Goes backwards to the right if distance is less or equal than 7 cm 
    	if (distance <= 7)
    	{
    		lab3.leftMotor.setSpeed(115);
    		lab3.rightMotor.setSpeed(160);
    		lab3.leftMotor.backward();
    		lab3.rightMotor.backward();
    	}
    	//right turn
    	else 
    	{
    		lab3.leftMotor.setSpeed(270);
    		lab3.rightMotor.setSpeed(120 - diff);
    		lab3.leftMotor.forward();
    		lab3.rightMotor.forward();
    	}
    	
    }
    
    else if (dist_ERR < 0)
    {
    	//left turn
    	lab3.leftMotor.setSpeed(100 - diff);
    	lab3.rightMotor.setSpeed(150);
    	lab3.leftMotor.forward();
    	lab3.rightMotor.forward();
    }
    
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
