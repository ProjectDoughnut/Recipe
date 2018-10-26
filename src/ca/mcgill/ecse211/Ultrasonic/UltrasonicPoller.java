package ca.mcgill.ecse211.Ultrasonic;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private UltrasonicController cont;
	private float[] usData;
	public int distance;

	public volatile boolean running = true;


	public UltrasonicPoller(SampleProvider us, UltrasonicController cont) {
		this.us = us;
		this.cont = cont;
		this.usData = new float[us.sampleSize()];
		this.running = true;
	}


	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		while (cont.isRunning()) {
			
			if (cont.getLock() != null) {
				Object lock = cont.getLock();
				synchronized(lock) {
					try {
						lock.wait();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
			
			us.fetchSample(usData, 0); // acquire data
			distance = (int)(usData[0] * 100.0); // extract from buffer, cast to int
			cont.process(distance); // now take action depending on value
			try {
				Thread.sleep(15);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

}