package ca.mcgill.ecse211.Light;

public interface LightController {
	
	public boolean isRunning();
	
	public Object getLock();
	
	public void process(int value);
	
}
