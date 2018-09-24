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

		this.odoData = Odometer.getOdometer();
		
		//2.Sensor instance
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
		double offset = 3.5;
		while (true) {
			correctionStart = System.currentTimeMillis();
			ColorID.fetchSample(csData, 0);
			float intensity = csData[0]; //last value sent by sensor
			// encounters a black line
			// all numbers within if-condition correspond to range of angle (in degrees) on square corners
			if(intensity<0.21) {
				Sound.playTone(3, 100);
				double angle =odoData.getXYT()[2];
				if(angle >= 0.00 && angle <= 40){ // case 1: keep increasing y-position until wheels turn 90 degrees
					odoData.setY(county*size -offset); // robot is horizontally in line with origin
					county++;
					LCD.drawString("case 1 " + county +" " + intensity, 0, 5);
				}
				else if(angle >= 78 && angle <= 105){ // case 2: keep increasing x-position until 90 degree turn
					odoData.setX(countx*size -offset);
					countx++;
					LCD.drawString("case 2 "+ countx, 0, 5);

				}// robot is vertically in line with origin
				else if(angle >= 165 && angle <= 193){ // case 3: keep decreasing y-position until 90 degree turn
					county--;
					odoData.setY(county*size -offset);
					LCD.drawString("case 3 " + county, 0, 5);

				}	
				else if(angle >= 260 && angle <= 284){ // case 4: keep decreasing x-position until 90 degree turn
					countx--;
					odoData.setX(countx*size - offset);
					LCD.drawString("case 4 " + countx, 0, 5);

				}	 
			}
		
			//last_intensity = intensity;
			
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


