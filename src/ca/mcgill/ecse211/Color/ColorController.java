package ca.mcgill.ecse211.Color;

public interface ColorController {
	
	public boolean isRunning();
	
	public Object getLock();
	
	public void process(float[] value);
	
}