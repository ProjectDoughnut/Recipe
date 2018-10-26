package ca.mcgill.ecse211.Light;

import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
	private SampleProvider ls;
	private LightController cont;
	private float[] lsData;
	public int distance;

	public volatile boolean running;

	public LightPoller(SampleProvider ls, LightController cont) {
		this.ls = ls;
		this.lsData = new float[ls.sampleSize()];
		this.cont = cont;
		this.running = true;
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert LS result to an integer
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
			
			ls.fetchSample(lsData, 0); // acquire data
			distance = (int)(lsData[0] * 100.0); // extract from buffer, cast to int
			cont.process(distance); // now take action depending on value
			try {
				Thread.sleep(20);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
}