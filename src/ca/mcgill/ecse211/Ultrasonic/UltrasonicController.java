package ca.mcgill.ecse211.Ultrasonic;


/**
 * 
 * Interface that is used by USLocalizer to process ultrasonic sensor data (in cm)
 *
 */

public interface UltrasonicController {
	/**
	 * Check if controller is running.
	 * 
	 * @return boolean
	 */
	public boolean isRunning();
	
	/**
	 * Return lock object used for thread synchronization.
	 * 
	 * @return Object
	 */
	public Object getLock();

	/**
	 * Method that will use the data read by the poller.
	 * 
	 * @param value
	 */
	public void process(int value);
	
}
