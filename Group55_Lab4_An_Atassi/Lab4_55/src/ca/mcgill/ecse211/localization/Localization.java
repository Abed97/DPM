package ca.mcgill.ecse211.localization;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

/***
 * This class allows the user to choose between rising and falling edge localization and to continue to light localization on the EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class Localization {

	// Motor Objects and parameters initialization
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	
	public static final double TRACK = 10.5;
	
	public static boolean edge = false;

	public static void main(String[] args) throws OdometerExceptions {
		
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		LightLocalizer lightLocalizer = new LightLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		int buttonChoice;
		Display odometryDisplay = new Display(lcd);
		do {
			// clear the display
			lcd.clear();

			// ask the user whether to choose between rising edge and falling edge
			lcd.drawString("< Left |  Right >", 0, 0);
			lcd.drawString("       |         ", 0, 1);
			lcd.drawString("Rising |  Falling", 0, 2);
			lcd.drawString("edge   |  edge   ", 0, 3);
			lcd.drawString("       | 		 ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) { // Go to the rising edge localization
			Thread odoThread = new Thread(odometer); 
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			usLocalizer.run();
			buttonChoice = Button.waitForAnyPress();
			lightLocalizer.run();

		} else { // Go to falling edge localization
			edge = true;
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			usLocalizer.run();
			buttonChoice = Button.waitForAnyPress();
			lightLocalizer.run();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}
