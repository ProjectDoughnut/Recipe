package ca.mcgill.ecse211.Color;

import lejos.robotics.SampleProvider;

public class ColorPoller extends Thread {
	private SampleProvider cs;
	private ColorController cont;
	private float[] csData;
	public float[] rgbValues;

	public volatile boolean running;

	public ColorPoller(SampleProvider cs, ColorController cont) {
		this.cs = cs;
		this.csData = new float[cs.sampleSize()];
		this.cont = cont;
		this.running = true;
		this.rgbValues = new float[3];
	}

	/*
	 * Sensors now return floats using a uniform protocol.
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
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
			
			cs.fetchSample(csData, 0); // acquire data
			rgbValues[0] = csData[0]; // extract from buffer, cast to int
			rgbValues[1] = csData[1]; // extract from buffer, cast to int
			rgbValues[2] = csData[2]; // extract from buffer, cast to int
			
			float unitRGB = (float)Math.sqrt(
					Math.pow(rgbValues[0], 2) +
					Math.pow(rgbValues[1], 2) + 
					Math.pow(rgbValues[2], 2));
			
			rgbValues[0] /= unitRGB; // extract from buffer, cast to int
			rgbValues[1] /= unitRGB; // extract from buffer, cast to int
			rgbValues[2] /= unitRGB; // extract from buffer, cast to int
			
			cont.process(rgbValues); // now take action depending on value
			try {
				Thread.sleep(25);
			} catch (Exception e) 
			{
			} // Poor man's timed sampling
		}
	}
}