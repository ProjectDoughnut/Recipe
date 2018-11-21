 package ca.mcgill.ecse211.Odometer;

import ca.mcgill.ecse211.Gyro.AngleSampler;
import ca.mcgill.ecse211.Light.LightController;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Main.Main;

/**
 * 
 * Class that corrects odometer readings when it crosses a black line. Filters light sensor data instead of 
 * directly reading the intensity
 *
 */

public class OdometryCorrector implements LightController {

	private Odometer odometer;
	private AngleSampler gyro;
	public boolean running = true;
	
	private boolean lineDetected = false;

	public static float firstReading = LightLocalizer.firstReading;

	private static final float lightThreshold = 20.0f;
	private static final double sensorDistance = 10.0; //in cm, 4.5inches

	private double corrX;
	private double corrY;

	public static final double TILE_SIZE = Main.TILE_SIZE;
	public static final double HALF_TILE_SIZE = Main.TILE_SIZE/2;
	private static final float ERROR_THRESHOLD = 5.0f;

	public OdometryCorrector(AngleSampler gyro) {
		corrX = 0;
		corrY = 0;
		this.gyro = gyro;
		try {
			odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
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


	@Override
	public void process(int value) {
		double xyt[] = odometer.getXYT();
		
		
		// correct the angle using value gyro
//		odometer.update(0, 0, gyro.getTheta()-xyt[2]);

		if (firstReading == -1) { //Set the first reading value
			firstReading = value;
		}

		if ((100*Math.abs(value - firstReading)/firstReading) > lightThreshold ) {
			if (lineDetected) {
				return;
			}

			double lineX, lineY, errorX, errorY, deltaX, deltaY;

			deltaX = Math.sin(Math.toRadians(xyt[2])) * sensorDistance;
			deltaY = Math.cos(Math.toRadians(xyt[2])) * sensorDistance;
			lineX = xyt[0] - deltaX;
			lineY = xyt[1] - deltaY;

			errorX = lineX % TILE_SIZE;
			if (errorX >= HALF_TILE_SIZE) {
				errorX -= TILE_SIZE;
			} else if (errorX <= -HALF_TILE_SIZE) {
				errorX += TILE_SIZE;
			}

			errorY = lineY % TILE_SIZE;
			if (errorY >= HALF_TILE_SIZE) {
				errorY -= TILE_SIZE;
			} else if (errorY <= -HALF_TILE_SIZE) {
				errorY += TILE_SIZE;
			}

			if (Math.abs(errorX) <= ERROR_THRESHOLD && errorX <= errorY) {
				corrX = -errorX;
				odometer.update(corrX, 0, 0);

			} else if (Math.abs(errorY) <= ERROR_THRESHOLD) {
				// probably y line
				corrY = -errorY;
				odometer.update(0, corrY, 0);
			}
			lineDetected = true;

		} else {
			lineDetected = false;
		}

	}

}
