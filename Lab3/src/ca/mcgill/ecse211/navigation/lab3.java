// Lab2.java
package ca.mcgill.ecse211.navigation;


import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


/***
 * This class implements Lab3 on the EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class lab3 {

  // Motor Objects, and Robot related parameters
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 9.85; //9.9;

  public static final int bandCenter = 15; //30; // Offset from the wall (cm)
  public static final int bandWidth = 1; //1; // Width of dead band (cm)
  public static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
  public static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/seec)


  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    //ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance(); 
    Display odometryDisplay = new Display(lcd); 
    navigation navigation = new navigation(leftMotor,rightMotor,TRACK,WHEEL_RAD);
    ObstacleAvoidance obstacleavoidance = new ObstacleAvoidance(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    do {
        // clear the display
        lcd.clear();

        // ask the user whether the motors should drive in a square or float
        lcd.drawString("< Left |  Right >", 0, 0);
        lcd.drawString("       |         ", 0, 1);
        lcd.drawString(" Float | Navigate", 0, 2);
        lcd.drawString("motors | 	     ", 0, 3);
        lcd.drawString("       | 		 ", 0, 4);

        buttonChoice = Button.waitForAnyPress();
      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

      if (buttonChoice == Button.ID_LEFT) {

        leftMotor.forward();
        leftMotor.flt();
        rightMotor.forward();
        rightMotor.flt();

     // Display changes in position as wheels are (manually) moved
        
        Thread odoThread = new Thread(odometer);
        odoThread.start();
        Thread odoDisplayThread = new Thread(odometryDisplay);
        odoDisplayThread.start();

      }
      else {
          // clear the display
          lcd.clear();

          // ask the user whether the motors should drive in a square or float
          lcd.drawString("< Left  | Right >", 0, 0);
          lcd.drawString("  No    | with   ", 0, 1);
          lcd.drawString("  obs - | obs-   ", 0, 2);
          lcd.drawString("  tacle | tacle  ", 0, 3);
          lcd.drawString("        |        ", 0, 4);
          
          buttonChoice = Button.waitForAnyPress();
          
       // Start odometer and display threads
          Thread odoThread = new Thread(odometer);
          odoThread.start();
          Thread odoDisplayThread = new Thread(odometryDisplay);
          odoDisplayThread.start();
          
          if(buttonChoice == Button.ID_LEFT){
        	 navigation.run();
          }
          
          if(buttonChoice == Button.ID_RIGHT){
        	  obstacleavoidance.run();
        	// run the obstacleAvoidance
          }
        }

        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
      }
    }