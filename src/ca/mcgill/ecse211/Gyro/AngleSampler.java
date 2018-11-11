package ca.mcgill.ecse211.Gyro;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.robotics.SampleProvider;

/**
 * 
 * Class that polls gyro sensor readings in its own thread, contains getters and setters for theta
 * which is the heading of the robot
 *
 */

public class AngleSampler {

	public boolean running;
	private float offset;


	public static Lock lock = new ReentrantLock(true); // Fair lock for
	// concurrent writing
	private volatile boolean isReseting = false; // Indicates if a thread is
	// trying to reset any
	// position parameters
	private Condition doneReseting = lock.newCondition(); // Let other threads
	// know that a reset
	// operation is
	// over.
	
	private SampleProvider gyro;
	private float[] gyroData;

	public AngleSampler(SampleProvider gyro) {
		this.running = true;
		this.gyro = gyro;
		this.gyroData = new float[gyro.sampleSize()];
		this.offset = 0;
	}

	/**
	 * Get angle detected by gyroscope
	 * @return float
	 */
	public float getTheta() {
		gyro.fetchSample(gyroData, 0); // acquire data

		return (-(gyroData[0])+offset)%360; 
	}
	
	public void resetOffset() {
		this.offset = 0;
	}

	public float getOffset() {
		return offset;
	}

	public void setOffset(float offset) {
		this.offset = offset;
	}

	public boolean isRunning() {
		return this.running;
	}

}
