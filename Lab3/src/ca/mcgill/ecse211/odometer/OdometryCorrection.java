/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.sensor.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private static final Port csPort = LocalEV3.get().getPort("S1");
	private float[] csData;
	private SampleProvider ColorID;
	private int countx=0;
	private int county=0;
	private static final double size = 30.48;
	private double bufferY=0;
	private double bufferTheta=0;


	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {
		// There are 4 steps involved:
		// 1. Create a port object attached to a physical port (done already above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data

		this.odometer = Odometer.getOdometer();
		SensorModes ColorSensor = new EV3ColorSensor(csPort);
		ColorID = ColorSensor.getMode("Red");
		this.csData = new float[ColorID.sampleSize()];
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		while (true) {
			correctionStart = System.currentTimeMillis();
			ColorID.fetchSample(csData, 0);
			float intensity = csData[0]; //last value sent by sensor
			// encounters a black line
			// all numbers within if-condition correspond to range of angle (in degrees) on square corners
			if(intensity<0.3) {
				Sound.beep();
				if((odometer.getXYT()[2] >= 350 && odometer.getXYT()[2] <= 360) || (odometer.getXYT()[2] >= 0.00 && odometer.getXYT()[2] <= 10)){ // case 1: keep increasing y-position until wheels turn 90 degrees
					odometer.setY(county*size); // robot is horizontally in line with origin
					county++;
				}
				else if(odometer.getXYT()[2] >= 80 && odometer.getXYT()[2] <= 100){ // case 2: keep increasing x-position until 90 degree turn
					odometer.setX(countx*size);
					countx++;
				}// robot is vertically in line with origin
				else if(odometer.getXYT()[2] >= 170 && odometer.getXYT()[2] <= 190){ // case 3: keep decreasing y-position until 90 degree turn
					county--;
					odometer.setY(county*size);
					bufferY=odometer.getXYT()[1];
				}	
				else if(odometer.getXYT()[2] >= 260 && odometer.getXYT()[2] <= 284){ // case 4: keep decreasing x-position until 90 degree turn
					countx--;
					odometer.setX(countx*size);
					/*if(countx==3) {
						bufferTheta=odometer.getXYT()[2];
					}
					if(countx==0) {
						bufferY=bufferY+91.44*Math.sin(270-bufferTheta);
						odometer.setY(bufferY);
					}*/
				}	 
			}
			// TODO Trigger correction (When do I have information to correct?)
			// TODO Calculate new (accurate) robot position

			// TODO Update odometer with new calculated (and more accurate) vales

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


