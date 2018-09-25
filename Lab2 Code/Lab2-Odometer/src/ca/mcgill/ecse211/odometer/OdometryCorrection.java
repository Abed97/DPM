/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.sensor.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odoData;
	private static final Port csPort = LocalEV3.get().getPort("S1");
	private float[] csData;
	private SampleProvider ColorID;
	private int xCount = 0;
	private int yCount = 0;
	private static final double size = 30.48;

	private double bufferY = 0;
	private double bufferTheta = 0;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odoData = Odometer.getOdometer();

		SensorModes ColorSensor = new EV3ColorSensor(csPort);

		ColorID = ColorSensor.getMode("Red");

		this.csData = new float[ColorID.sampleSize()];

	}

	/**
	 * This method corrects the X and Y distances depending on where the robot is.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		double offset = 3.5; //distance of the sensor from the middle of the robot
		while (true) {
			correctionStart = System.currentTimeMillis();
			ColorID.fetchSample(csData, 0);
			float intensity = csData[0]; // last value sent by sensor
			if (intensity < 0.21) { // Checks if a black line is detected
				Sound.playTone(3, 100); //Plays a sound when the black line is detected
				double angle = odoData.getXYT()[2];
				if (angle >= 0.00 && angle <= 40) { // Keep increasing y-position of the robot until before the 90 degree turn
					odoData.setY(yCount * size - offset); // keep updating Y depending on how many tiles are being traversed
					yCount++; // Increase tile count
					LCD.drawString("case 1 " + yCount + " " + intensity, 0, 5); //draw the case number and number of tiles traversed
				} else if (angle >= 75 && angle <= 105) { // Keep increasing x-position until 90 degree turn
					odoData.setX(xCount * size - offset); // keep updating X depending on how many tiles are being traversed
					xCount++;
					LCD.drawString("case 2 " + xCount, 0, 5); 

				} 
				else if (angle >= 165 && angle <= 195) { // Keep decreasing y-position until 90 degree turn
					yCount--;
					odoData.setY(yCount * size - offset);
					LCD.drawString("case 3 " + yCount, 0, 5);

				} else if (angle >= 260 && angle <= 285) { // Keep decreasing x-position until 90 degree turn
					xCount--;
					odoData.setX(xCount * size - offset);
					LCD.drawString("case 4 " + xCount, 0, 5);

				}
			}

			// last_intensity = intensity;

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
