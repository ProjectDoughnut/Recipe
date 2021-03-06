package ca.mcgill.ecse211.Light;

/**
 * 
 * Interface that is used by LightPoller to process light sensor data
 *
 */

public interface LightController {
	
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
