package ca.mcgill.ecse211.Color;

public interface ColorController {
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
	public void process(float[] value);
	
}