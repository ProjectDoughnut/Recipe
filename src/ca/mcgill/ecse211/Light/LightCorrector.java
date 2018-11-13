package ca.mcgill.ecse211.Light;

import ca.mcgill.ecse211.Main.Main;
import ca.mcgill.ecse211.Main.Navigation;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;



/**
 * 
 * Class that corrects odometer readings when it crosses a black line. Filters light sensor data instead of 
 * directly reading the intensity
 *
 */

public class LightCorrector implements LightController {

	private Odometer odometer;
	private Navigation nav;
	public boolean running = true;

	public static float firstReading = -1;

	private static final float lightThreshold = 20.0f;
	private static final double sensorDistance = 10.0; //in cm, 4.5inches

	private double corrX;
	private double corrY;

	public static final double TILE_SIZE = Main.TILE_SIZE;
	public static final double HALF_TILE_SIZE = Main.TILE_SIZE/2;
	private static final float ERROR_THRESHOLD = 5.0f;

	public LightCorrector(Navigation nav) {
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

		if (firstReading == -1) { //Set the first reading value
			firstReading = value;
		}

		if ((100*Math.abs(value - firstReading)/firstReading) > lightThreshold && this.nav.betweenLines()) {

			this.nav.adjustOdometer();
		}
		
		  try {Thread.sleep(500);
			} catch (InterruptedException e) {}
	}

}
