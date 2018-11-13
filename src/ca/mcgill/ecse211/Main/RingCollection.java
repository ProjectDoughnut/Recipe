package ca.mcgill.ecse211.Main;

import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * 
 * The RingCollection class drives the robot around the tree, stopping at each branch and attempting to collect rings. Contains a main run method
 * and a getRing() method that directs the robot once it is in line with a tree branch. This thread will run at the same time as navigation, odometry, 
 * odometry correction, and color classification. However, the color classification thread is only started once this thread begins. The methodology for this class is
 * quite simple, using a series of if-statements to determine which points around the tree to travel to and to ensure that hitting the wall or water is avoided.
 * The getRing() method also follows a fairly simple methodology. 
 *
 */

public class RingCollection extends Thread{

	private Odometer odo;
	private Navigation nav;
	private EV3MediumRegulatedMotor servo;
	private float[][] tunnel;
	private float[][] island;
	private float[] tree;
	private double WHEEL_RADIUS;
	private double sensorDistance;
	private double toBranch = 5;
	public static Object done = new Object();
	
	public boolean running;
	
	public RingCollection(Odometer odo, Navigation nav, EV3MediumRegulatedMotor servo, float[][] tunnel, float[] tree, float[][] island, double WHEEL_RADIUS, double sensorDistance) {
		this.odo = odo;
		this.nav = nav;
		this.servo = servo;
		this.tree = tree;
		this.tunnel = tunnel;
		this.island = island;
		this.WHEEL_RADIUS = WHEEL_RADIUS;
		this.sensorDistance = sensorDistance;
		this.running = true;
	}
	
	@SuppressWarnings("static-access")
	public void run() {
		
		//Lock the color thread until this thread begins

		
		/*
		 * Check that the robot is done navigating to tree first?????
		 * Then, determine which orientation it is in
		 */
		
		if (nav.tunnelPointingX && nav.tunnelUp) {
			
			//This is not taking into account island or wall parameters
			//SHOULD THESE BE SYNCTRAVELTO????
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] - 1);
			nav.syncTravelTo(tree[0] - 1, tree[1]);
			nav.turnTo(odo.getXYT()[2], 90);
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] + 1);
			nav.syncTravelTo(tree[0], tree[1] + 1);
			//Not sure about this turn
			nav.turnTo(odo.getXYT()[2], 180);
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] + 1);
			nav.syncTravelTo(tree[0] + 1, tree[1]);
			nav.turnTo(odo.getXYT()[2], -90);
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] - 1);
			nav.syncTravelTo(tree[0], tree[1] - 1);
			
		}
		else if (nav.tunnelPointingX && !nav.tunnelUp) {
			
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] + 1);
			nav.syncTravelTo(tree[0] + 1, tree[1]);
			//Not sure about this angle:
			nav.turnTo(odo.getXYT()[2], -90);
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] - 1);
			nav.syncTravelTo(tree[0], tree[1] - 1);
			//Not sure about this angle:
			nav.turnTo(odo.getXYT()[2], 0);
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] - 1);
			nav.syncTravelTo(tree[0] - 1, tree[1]);
			//Not sure:
			nav.turnTo(odo.getXYT()[2], 90);
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] + 1);
			nav.syncTravelTo(tree[0], tree[1] + 1);
			
		}
		else if (!nav.tunnelPointingX && nav.tunnelToLeft) {
			
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] - 1);
			nav.syncTravelTo(tree[0], tree[1] - 1);
			nav.turnTo(odo.getXYT()[2], 0);
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] - 1);
			nav.syncTravelTo(tree[0] - 1, tree[1]);
			//Not sure about this turn
			nav.turnTo(odo.getXYT()[2], 90);
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] + 1);
			nav.syncTravelTo(tree[0], tree[1] + 1);
			nav.turnTo(odo.getXYT()[2], 180);
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] + 1);
			nav.syncTravelTo(tree[0] + 1, tree[1]);
			
		}
		else if (!nav.tunnelPointingX && !nav.tunnelToLeft) {
			
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] + 1);
			nav.syncTravelTo(tree[0], tree[1] + 1);
			nav.turnTo(odo.getXYT()[2], 180);
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] + 1);
			nav.syncTravelTo(tree[0] + 1, tree[1]);
			//Not sure about this turn
			nav.turnTo(odo.getXYT()[2], -90);
			getRings();
			nav.syncTravelTo(tree[0] + 1, tree[1] - 1);
			nav.syncTravelTo(tree[0], tree[1] - 1);
			nav.turnTo(odo.getXYT()[2], 0);
			getRings();
			nav.syncTravelTo(tree[0] - 1, tree[1] - 1);
			nav.syncTravelTo(tree[0] - 1, tree[1]);
		}
	}

	/*
	 * This method collects the rings (if any) on the branch of the tree that the robot is currently facing. 
	 * This also runs under the assumption that the ring is already at the correct angle, facing the branch
	 */
	public void getRings() {
		
		//Open claw
		servo.rotate(90);
		
		//Move the robot forward a set amount: This should be tested multiple times to determine the ideal amount
		odo.leftMotor.rotate(Navigation.convertDistance(WHEEL_RADIUS, toBranch), true);
		odo.rightMotor.rotate(Navigation.convertDistance(WHEEL_RADIUS, toBranch), false);
		
		//Close claw
		servo.rotate(-90);
		
		//Move the robot backward a set amount: This should be tested multiple times to determine the ideal amount
		odo.leftMotor.rotate(-Navigation.convertDistance(WHEEL_RADIUS, toBranch), true);
		odo.rightMotor.rotate(-Navigation.convertDistance(WHEEL_RADIUS, toBranch), false);
	}
	

}
