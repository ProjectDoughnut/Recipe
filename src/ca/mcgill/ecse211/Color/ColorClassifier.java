package ca.mcgill.ecse211.Color;

import java.util.ArrayList;

import ca.mcgill.ecse211.Main.Navigation;
import ca.mcgill.ecse211.Odometer.Odometer;
import lejos.hardware.Sound;

/**
 * 
 * ColorClassifier compares the data received by the color sensor to the RGB values 
 * of the different coloured rings using Gaussian distribution and beeps accordingly, e.g. one beep for for blue
 * two beeps for green, etc
 *
 */

public class ColorClassifier implements ColorController{

	private Odometer odo;
	private Navigation nav;

	public enum ClassificationState{INIT, CLASSIFYING, DONE};

	public enum RingColors{BLUE, GREEN, YELLOW, ORANGE};

	public float lightSensorIntensity;
	
	private ArrayList<RingColors> detectedRings;
	private ArrayList<float[]> detectedRingValues;
	
	public static RingColors targetRing;
	public static boolean targetDetected;
	public static RingColors detectedRing;
	public boolean classifyingDemo;
	public static double[] targetLocation = new double[3];
	public static float[] targetRGBValues = new float[3];

	public static double[] blueRValues = {0.191671, 0.026218};
	public static double[] blueGValues = {0.593177, 0.043177};
	public static double[] blueBValues = {0.7793101, 0.0390857};

	public static double[] greenRValues = {0.504023, 0.007849};
	public static double[] greenGValues = {0.845422, 0.003544};
	public static double[] greenBValues = {0.1757721, 0.0158844};

	public static double[] orangeRValues = {0.97871, 0.007473};
	public static double[] orangeGValues = {0.181839, 0.036419};
	public static double[] orangeBValues = {0.0865479, 0.0137534};

	public static double[] yellowRValues = {0.883695, 0.016992};
	public static double[] yellowGValues = {0.446074, 0.033467};
	public static double[] yellowBValues = {0.1366481, 0.0045489};

	public static Object lock;
	public boolean running;

	

	public ClassificationState state = ClassificationState.INIT;


	public ColorClassifier(Odometer odo, Navigation nav, RingColors targetRing, boolean classifying) {
		this.odo = odo;
		this.nav = nav;
		this.running = true;
		this.detectedRings = new ArrayList<RingColors>();
		this.detectedRingValues = new ArrayList<float[]>();
		ColorClassifier.targetRing = targetRing;
		this.classifyingDemo = classifying;
		
	}

	public void process(float[] values) {
		detectedRing = null;
		RingColors ring = detectColor(values);
		
		
		
		if (ring != null && detectedRings.indexOf(ring) == -1 && !this.classifyingDemo) {
			detectedRings.add(ring);
			detectedRingValues.add(values);
			ColorClassifier.detectedRing = ring;
			
			if (ring == targetRing) {
				Sound.beep();

				// stopping navigation
				targetDetected = true;
				targetLocation = odo.getXYT();
				targetRGBValues = values;
				Object lock = new Object();
				Navigation.lock = lock;
				nav.setRunning(false);
				nav.clearCoordList();

				Navigation.lock = null;
				synchronized(lock) {
					lock.notifyAll();
				}
				// stopping this thread
					this.running = false;
			} else {
				Sound.twoBeeps();
			}
			
		} else if (ring != null && classifyingDemo) {
			Sound.buzz();
			ColorClassifier.detectedRing = ring;
		}	
	}


	public RingColors detectColor(float[] values) {
		float R, G, B;
		R = values[0];
		G = values[1];
		B = values[2];

		if (withinGaussDist(R, blueRValues, 2) &&
				withinGaussDist(G, blueGValues, 2) &&
				withinGaussDist(B, blueBValues, 2)) {
			return RingColors.BLUE;
		} else if (withinGaussDist(R, greenRValues, 4) &&
				withinGaussDist(G, greenGValues, 4) &&
				withinGaussDist(B, greenBValues, 4)) {
			return RingColors.GREEN;
		} else if (withinGaussDist(R, orangeRValues, 2) &&
				withinGaussDist(G, orangeGValues, 2) &&
				withinGaussDist(B, orangeBValues, 2)) {
			return RingColors.ORANGE;
		} else if (withinGaussDist(R, yellowRValues, 2) &&
				withinGaussDist(G, yellowGValues, 2) &&
				withinGaussDist(B, yellowBValues, 2)) {
			return RingColors.YELLOW;
		}
		return null;
	}


	public boolean withinGaussDist(double value, double[] target, int sigma) {
		return (Math.abs(value - target[0]) <= sigma * target[1]);
	}

	public boolean isRunning() {
		return this.running;
	}

	public Object getLock() {
		return lock;
	}


	public void setClassifyingDemo(boolean set) {
		this.classifyingDemo = set;
	}
}
