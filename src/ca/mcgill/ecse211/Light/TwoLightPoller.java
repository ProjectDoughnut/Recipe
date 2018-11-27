package ca.mcgill.ecse211.Light;

import lejos.robotics.SampleProvider;

public class TwoLightPoller extends Thread{

	private SampleProvider ls;
	private SampleProvider ls2;
	private TwoLightController cont;
	private float[] lsData;
	private float[] lsData2;
	public int distance;
	public int distance2;
	
	public int SLEEP_TIME = 25;

	public volatile boolean running;

	public TwoLightPoller(SampleProvider ls, SampleProvider ls2, TwoLightController cont) {
		this.ls = ls;
		this.ls2 = ls2;
		this.lsData = new float[ls.sampleSize()];
		this.lsData2 = new float[ls2.sampleSize()];
		this.cont = cont;
		this.running = true;
	}

	public TwoLightPoller(SampleProvider ls, SampleProvider ls2, TwoLightController cont, int SLEEP_TIME) {
		this(ls, ls2, cont);
		this.SLEEP_TIME = SLEEP_TIME;
	}
	/**
	 * Poller which pass the sensor information to its corresponding controller.
	 * Has a lock mechanism to sleep the thread.
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		long updateStart, updateEnd;

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
			updateStart = System.currentTimeMillis();
			
			ls.fetchSample(lsData, 0); // acquire data
			distance = (int)(lsData[0] * 100.0); // extract from buffer, cast to int
			
			ls2.fetchSample(lsData2, 0); // acquire data
			distance2 = (int)(lsData2[0] * 100.0); // extract from buffer, cast to int
			
			cont.process(distance, distance2); // now take action depending on value
			
			updateEnd = System.currentTimeMillis();
			try {
				Thread.sleep(this.SLEEP_TIME - (updateEnd - updateStart));
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
	
}
