package ca.mcgill.ecse211.Ultrasonic;

public interface UltrasonicController {
	
	public boolean isRunning();
	
	public Object getLock();

	public void process(int value);
	
}
