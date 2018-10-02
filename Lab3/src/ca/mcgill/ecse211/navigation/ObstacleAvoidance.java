package ca.mcgill.ecse211.navigation;
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;


import lejos.hardware.motor.NXTRegulatedMotor;





public class ObstacleAvoidance implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final NXTRegulatedMotor sensorMotor = new NXTRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	//HSA
	
	private static final int bandCenter = 30; // Offset from the wall (cm)
	private static final int bandWidth = 1; //1; // Width of dead band (cm)
	private static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/seec)
	
	PController pController = new PController(bandCenter, bandWidth);
	
	int runTime = 15000; //runtime in milliseconds
	boolean obstacle_encountered = false;
	
	
	
	
	
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	int iterator = 0;
	private Odometer odometer;
	private OdometerData odoData;
	private double[][]  wayPoints = new double[][]{{2*30.48,1*30.48}, // change values for different maps
		{1*30.48,1*30.48},
		{1*30.48,2*30.48},
		{2*30.48,0*30.48},
		};
	private int var;
		//array list for points
		public ObstacleAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
				final double TRACK, final double WHEEL_RAD) throws OdometerExceptions { // constructor
			this.odometer = Odometer.getOdometer();
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			odoData = OdometerData.getOdometerData();
			odoData.setXYT(0 , 0 , 0);
			this.TRACK = TRACK;
			this.WHEEL_RAD = WHEEL_RAD;
			sensorMotor.resetTachoCount();
			var = 50;
			sensorMotor.setSpeed(30);

			SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
			usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
			// this instance
			this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
			// returned
		}

		// run method (required for Thread)
		public void run() {
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
				motor.stop();
				motor.setAcceleration(300);  // reduced the acceleration to make it smooth
			}
			// wait 5 seconds
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not expected that
				// the odometer will be interrupted by another thread
			}
			
			
			
			
			// implemented this for loop so that navigation will work for any number of points
			while(iterator < wayPoints.length) { //iterate through all the points 
				travelTo(wayPoints[iterator][0], wayPoints[iterator][1]);
				iterator++;
			}
		}

		void travelTo(double x, double y) {
			
			
			currentX = odometer.getXYT()[0];// get the position on the board
			currentY = odometer.getXYT()[1];
			currentT = odometer.getXYT()[2];

			dx = x- currentX;
			dy = y - currentY;
			distanceToTravel = Math.sqrt(dx*dx+dy*dy);
			if(dy>=0) {
				dt=Math.atan(dx/dy);
			}
			else if(dy<=0&&dx>=0) {
				dt=Math.atan(dx/dy)+Math.PI;
			}
			else {
				dt=Math.atan(dx/dy)-Math.PI;
			}//Mathematical convention

			// initial angle is 0||2pi, same direction as y-axis, going clockwise
			double differenceInTheta = (dt*180/Math.PI-currentT); // robot has to turn "differenceInTheta",
			//turn the robot to the desired direction
			turnTo(differenceInTheta); 

			// drive forward required distance
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);

			
			
			while(isNavigating()) { //avoiding the obstacles
				
				//HSA
				
				
				
				
				
				
			// rotate sensor
			
				/*
				sensorMotor.rotateTo(20,false);
				sensorMotor.rotateTo(-20,false);
				*/
				
				usDistance.fetchSample(usData,0);
				float distance = usData[0]*100;
				
				
				if (distance <= 10 && obstacle_encountered == false)
				{
					
				
					//leftMotor.stop(true);
					//rightMotor.stop(false);
					sensorMotor.rotateTo(var, true);
					
					long startTime = System.currentTimeMillis();
					
					
					while (System.currentTimeMillis() - startTime <= runTime)
					{
						
						usDistance.fetchSample(usData,0);
						distance = usData[0]*100;
						pController.processUSData((int) distance);
						
					}
					
					leftMotor.stop(true);
					rightMotor.stop(false);
					sensorMotor.rotateTo(var - 50, false);
					
					obstacle_encountered = true;
					iterator--;
					
				}
				
				
				
				
				/*
				//usDistance.fetchSample(usData,0);
				//float distance = usData[0]*100;
				if(distance<= 15) {
					if(odometer.getXYT()[0]<2.4*30.48&&odometer.getXYT()[0]>1.3*30.48&&odometer.getXYT()[1]<2.5*30.48&&odometer.getXYT()[1]>1.6*30.48){
						leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
						rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
						leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
						leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
						rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
					}
					else {
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
					leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
					}
					iterator--;
				}
				
				*/
				
				
				
			}
		}

		void turnTo(double theta) {
			if(theta>180) { // angle convention. the robot should turn in direction
				theta=360-theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
			else if(theta<-180) {
				theta=360+theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
			else {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);	
			}
		}


		boolean isNavigating() {
			if((leftMotor.isMoving() || rightMotor.isMoving()))
				return true;
			else 
				return false;

		}

		private static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}
		private static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
}
