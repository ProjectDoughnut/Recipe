package ca.mcgill.ecse211.Light;

import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
	private SampleProvider ls;
	private LightController cont;
	private float[] lsData;
	public int distance;
	
	public int SLEEP_TIME = 20;

	public volatile boolean running;

	public LightPoller(SampleProvider ls, LightController cont) {
		this.ls = ls;
		this.lsData = new float[ls.sampleSize()];
		this.cont = cont;
		this.running = true;
	}

	public LightPoller(SampleProvider ls, LightController cont, int SLEEP_TIME) {
		this(ls, cont);
		this.SLEEP_TIME = SLEEP_TIME;
	}
	/**
	 * Poller which pass the sensor information to its corresponding controller.
	 * Has a lock mechanism to sleep the thread.
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
				Thread.sleep(this.SLEEP_TIME);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
}