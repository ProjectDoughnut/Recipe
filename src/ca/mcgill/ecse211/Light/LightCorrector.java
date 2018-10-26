package ca.mcgill.ecse211.Light;

import ca.mcgill.ecse211.Lab5.Main;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;


public class LightCorrector implements LightController {

	private Odometer odometer;
	public boolean running = true;

	public static float firstReading = -1;

	private static final float lightThreshold = 20.0f;
	private static final double sensorDistance = 10.0; //in cm, 4.5inches

	private double corrX;
	private double corrY;

	public static final double TILE_SIZE = Main.TILE_SIZE;
	public static final double HALF_TILE_SIZE = Main.TILE_SIZE/2;
	private static final float ERROR_THRESHOLD = 5.0f;

	public LightCorrector() {
		corrX = 0;
		corrY = 0;
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

		if (firstReading == -1) { //Set the first reading value
			firstReading = value;
		}

		if ((100*Math.abs(value - firstReading)/firstReading) > lightThreshold) {

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

				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {

					e.printStackTrace();
				}

			}

		}

	}

}
