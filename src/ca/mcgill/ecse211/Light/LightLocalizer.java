package ca.mcgill.ecse211.Light;


import ca.mcgill.ecse211.Main.Navigation;
import ca.mcgill.ecse211.Odometer.OdometryCorrector;
import ca.mcgill.ecse211.Odometer.Odometer;

import lejos.hardware.Sound;

/**
 * 
 * Localizes the robot to the given origins by the user using the light sensor. Rotates 360° sweeping all
 * black lines on the platform and using trigonometry makes adjustments to the x,y and theta accordingly
 *
 */

public class LightLocalizer implements LightController{

	private Odometer odo;
	private Navigation nav;

	private int lineCount;

	public enum LocalizationState{INIT, FORWARD, BACKTRACK, SEEK, CORRECTION, DONE};


	private double thetaX, thetaY;
	private double correctedX, correctedY;
	private double deltaThetaX, deltaThetaY, deltaTheta;
	public static float firstReading = -1;
	private double lightThreshold = 35.0;
	public float lightSensorIntensity;
	private double sensorDistance = 11.3; //in cm, 4.5inches
	private final double WHEEL_RAD = 2.2;
	private double[] lineAngles, linePos;
	private static final int ROTATE_SPEED = 150 ;
	
	public static Object lock;
	public boolean running;


	public LocalizationState state = LocalizationState.INIT;


	public LightLocalizer(Odometer odo, Navigation nav) {
		this.odo = odo;
		this.nav = nav;
		this.lineCount = 0;
		this.lineAngles = new double[4];
		this.linePos = new double[3];
		this.running = true;
	}

	public void process(int value) {
		switch(state) {
		case INIT:

			if (firstReading == -1) { //Set the first reading value
				firstReading = value;
				OdometryCorrector.firstReading = value;
			}

			//At this point, the robot will (ideally) be at 0-degrees
			//Turning it 45 degrees will (ideally) turn it in the direction of (0,0)
			//***Might want to change this to just 45???
			nav.turnTo(odo.getXYT()[2], 45);


			odo.leftMotor.setSpeed(ROTATE_SPEED);
			odo.rightMotor.setSpeed(ROTATE_SPEED);
			odo.leftMotor.forward();
			odo.rightMotor.forward();

			this.state = LocalizationState.FORWARD;
			break;
		case FORWARD:
			//Have the robot move forward until the light sensor detects a line (aka the (0, 0) point)

			/*If the current reading is significantly less than the first reading 
			 * (aka 30% less), a line is being passed. 
			 * Has to be a significant enough change, since the panels are not a uniform color. */
			if ((100*Math.abs(value - firstReading)/firstReading) > lightThreshold) {
				if (value < firstReading) {
					odo.leftMotor.stop(true);
					odo.rightMotor.stop(false);
					this.state = LocalizationState.BACKTRACK;
				}
			} 
			break;
		case BACKTRACK:

			//Have the robot move backwards (by the sensorDistance amount)
			//until the wheel center is at the (0, 0) point instead of the sensor
			odo.leftMotor.rotate(-Navigation.convertDistance(WHEEL_RAD , sensorDistance), true);
			odo.rightMotor.rotate(-Navigation.convertDistance(WHEEL_RAD, sensorDistance), false);
			this.state = LocalizationState.SEEK;
			break;

		case SEEK:
			linePos = odo.getXYT();
			
			// start rotating to find the lines
			odo.leftMotor.backward();
			odo.rightMotor.forward();

			//Continue rotating until all 4 lines are crossed

			if ((100*Math.abs(value - firstReading)/firstReading) > lightThreshold) {

				//This is included because previously, the motors would stop later than expected
				//leading to errors in calculations
				if(lineCount == 3) {
					odo.leftMotor.stop(true);
					odo.rightMotor.stop(false);
					this.state = LocalizationState.CORRECTION;
				}
				
				lineAngles[lineCount] = linePos[2];

				Sound.beep();

				lineCount++;

			}
			break;

		case CORRECTION:
			//Trigonometry calculations from tutorial
			//assuming that we start in the bottom left square
			thetaY = Math.abs(lineAngles[2] - lineAngles[0]);
			thetaX =  Math.abs(lineAngles[3] - lineAngles[1]);

			correctedX = -sensorDistance*Math.cos(Math.toRadians(thetaY/2)) + 30.48;
			correctedY = -sensorDistance*Math.cos(Math.toRadians(thetaX/2)) + 30.48;

			//current theta value
			//540 was used for deltaThetaY instead of 270 as an offset for more accurate calculations
			deltaThetaX = (270 + (thetaX/2) - lineAngles[3]) % 360;
			deltaThetaY = (540 + (thetaY/2) - lineAngles[0]) % 360;

			//Getting the average of the two values gave a more accurate result
			deltaTheta = (deltaThetaX + deltaThetaY)/2.0;

			odo.setXYT(correctedX, correctedY, -deltaTheta);	

			nav.syncTravelTo(1, 1);
			nav.turnTo(odo.getXYT()[2], 2);

			this.state = LocalizationState.DONE;
			break;
		case DONE:
			this.running = false;
		default:
			break;
		}
		
	}
	
	public boolean isRunning() {
		return this.running;
	}

	public Object getLock() {
		return lock;
	}
	
	
}

