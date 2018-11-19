package ca.mcgill.ecse211.Odometer;

import ca.mcgill.ecse211.Light.TwoLightController;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Main.Main;
import ca.mcgill.ecse211.Main.Navigation;

/**
 * 
 * Class that corrects odometer readings when it crosses a black line. Filters light sensor data instead of 
 * directly reading the intensity
 *
 */

public class OdometryCorrector implements TwoLightController {

	private Odometer odometer;
	private Navigation nav;
	public boolean running = true;

	/**
	 * If lineDetected is 1, it means that the right sensor detected the line
	 * If lineDetected is -1, it means that the left sensor detected the line
	 * If lineDetected is 0, no sensor detected any line
	 */
	private int lineDetected;
	private int correctionFilter;

	public static float firstReading = -1;

	private static final float lightThreshold = 20.0f;
	private static final double sensorDistance = 10.0; //in cm, 4.5inches
	private static final float ERROR_THRESHOLD = 5.0f;

	private static final int MIN_CORRECTION_FILTER = 0;
	private static final int MAX_CORRECTION_FILTER = 40;

	public static final double TILE_SIZE = Main.TILE_SIZE;
	public static final double HALF_TILE_SIZE = Main.TILE_SIZE/2;

	public enum CorrectionState{DETECTING, CORRECTING, END};
	public CorrectionState state = CorrectionState.DETECTING;


	public OdometryCorrector(Navigation nav) {
		this.lineDetected = 0;
		this.correctionFilter = 0;
		this.nav = nav;
		try {
			this.odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public boolean isRunning() {

		return this.running;
	}

	@Override
	public Object getLock() {

		return null;
	}

	public boolean detectLine(int value) {
		return (100*Math.abs(value - firstReading)/firstReading) > lightThreshold;
	}

	@Override
	public void process(int leftLS, int rightLS) {

		switch(this.state)  {
		case DETECTING:
			//Set the first reading value
			if (firstReading == -1) { 
				firstReading = (leftLS+rightLS)/2;
			}
			
			// make sure the navigation is navigating and not turning
			// if turning, we dont want the robot to detect the line and try to correct
			if (!nav.isNavigating()) {
				return;
			}
			
			// if a line is detected, we increase filter of the detection
			// if the filter is more than the max allowed filter value, we reset the lineDetected value (false positive)
			if (lineDetected != 0) {
				if (correctionFilter > MAX_CORRECTION_FILTER) {
					correctionFilter = 0;
					lineDetected = 0;
				} else {
					correctionFilter++; 
				}
			}

			// check if right sensor detected a line
			if (detectLine(rightLS)) {
				if (lineDetected == 0) {
					lineDetected = 1;
				} else if (lineDetected == -1) {
					// if correction filter is smaller than the MIN_CORRECTION_FILTER
					// we do not correct
					if (correctionFilter < MIN_CORRECTION_FILTER) {
						lineDetected = 0;
						correctionFilter = 0;
						return;
					}
					Object lock = new Object();
					Navigation.lock = lock;
					while(odometer.leftMotor.isMoving() || odometer.rightMotor.isMoving());
					odometer.leftMotor.flt(false);
					odometer.rightMotor.backward();
					this.state = CorrectionState.CORRECTING;
				}
			}

			if (detectLine(leftLS)) {
				if (lineDetected == 0) {
					lineDetected = -1;
				} else if (lineDetected == 1) {
					// if correction filter is smaller than the MIN_CORRECTION_FILTER
					// we do not correct
					if (correctionFilter < MIN_CORRECTION_FILTER) {
						lineDetected = 0;
						correctionFilter = 0;
						return;
					}
					Object lock = new Object();
					Navigation.lock = lock;
					while(odometer.leftMotor.isMoving() || odometer.rightMotor.isMoving());
					odometer.rightMotor.flt(false);
					odometer.leftMotor.backward();
					this.state = CorrectionState.CORRECTING;
				}
			}
			break;
		case CORRECTING:
			if (lineDetected == 1) {
				if (detectLine(rightLS)) {
					odometer.leftMotor.stop(true);
					odometer.rightMotor.stop(false);
					this.state = CorrectionState.END;
				}
			} else if (lineDetected == -1) {
				if (detectLine(leftLS)) {
					odometer.leftMotor.stop(true);
					odometer.rightMotor.stop(false);
					this.state = CorrectionState.END;
				}
			}
			break;
		case END:
			// reset the filter
			this.correctionFilter = 0;
			// reset state of the detection
			this.lineDetected = 0;


			// do odometry correction
			double[] xyt = odometer.getXYT();


			// set the angle to the closest one to a multiple of 90
			float theta = Math.round(xyt[2]/90.0) * 90;
			odometer.setTheta(theta);


			double lineX, lineY, errorX, errorY, deltaX, deltaY;

			deltaX = Math.sin(Math.toRadians(theta)) * sensorDistance;
			deltaY = Math.cos(Math.toRadians(theta)) * sensorDistance;
			lineX = xyt[0] - deltaX;
			lineY = xyt[1] - deltaY;

			errorX = Math.round(lineX/TILE_SIZE) - lineX ;

			errorY = Math.round(lineY/TILE_SIZE) - lineY;


			if (Math.abs(errorX) <= ERROR_THRESHOLD && errorX <= errorY) {
				// probably x line
				odometer.update(errorX, 0, 0);

			} else if (Math.abs(errorY) <= ERROR_THRESHOLD) {
				// probably y line
				odometer.update(0, errorY, 0);
			}

			Object lock = Navigation.lock;
			Navigation.lock = null;
			synchronized(lock) {
				lock.notifyAll();
			}

			this.state = CorrectionState.DETECTING;

			try {
				// arbitrary delay so that we don't detect 
				// the line that we just correct for
				Thread.sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}

	}

}
