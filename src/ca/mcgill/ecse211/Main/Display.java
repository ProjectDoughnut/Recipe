package ca.mcgill.ecse211.Main;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * 
 * Organizes and shows the readings/instructions on the brick's display to the user
 *
 */

public class Display extends Thread implements Runnable {

	private Odometer odo;
	private TextLCD lcd;
	private double[] position;
	private final long DISPLAY_PERIOD = 25;
	private long timeout = Long.MAX_VALUE;

	/**
	 * This is the class constructor
	 * 
	 * @param odoData
	 * @throws OdometerExceptions 
	 */
	public Display(TextLCD lcd) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.lcd = lcd;
	}

	/**
	 * This is the overloaded class constructor
	 * 
	 * @param odoData
	 * @throws OdometerExceptions 
	 */
	public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.timeout = timeout;
		this.lcd = lcd;
	}

	public void run() {
		lcd.clear();

		long updateStart, updateEnd;

		long tStart = System.currentTimeMillis();
		do {
			updateStart = System.currentTimeMillis();

			// Retrieve x, y and Theta information
			position = odo.getXYT();


			// Print x,y, and theta information
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);

			if (ColorClassifier.detectedRing != null) {
				lcd.clear();
				lcd.drawString("Object Detected!", 0, 0);
				lcd.drawString(ColorClassifier.detectedRing.toString(), 0, 1);
				try {
					Thread.sleep(1000);
					lcd.clear();
				} catch (InterruptedException e) {

					e.printStackTrace();
				} 
			}



			// this ensures that the data is updated only once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} while ((updateEnd - tStart) <= timeout);

	}

}


