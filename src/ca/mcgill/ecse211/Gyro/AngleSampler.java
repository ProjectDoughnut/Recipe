package ca.mcgill.ecse211.Gyro;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.robotics.SampleProvider;

public class AngleSampler {

	public boolean running;
	private float theta;

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
	}

	/**
	 * Get angle detected by gyroscope
	 * @return float
	 */
	public float getTheta() {
		gyro.fetchSample(gyroData, 0); // acquire data
		this.theta = -(gyroData[0]); 

		return theta;
	}

	public void setTheta(float theta) {

		lock.lock();
		isReseting = true;
		try {
			this.theta = theta;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
		} finally {
			lock.unlock();
		}
	}

	public boolean isRunning() {
		
		return this.running;
	}

}
