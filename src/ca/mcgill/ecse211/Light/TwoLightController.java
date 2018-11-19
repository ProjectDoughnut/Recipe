package ca.mcgill.ecse211.Light;

public interface TwoLightController {
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
	 * Method that will use the data read by the two pollers.
	 * 
	 * @param value 
	 * @param value2
	 */
	public void process(int value, int value2);
}
