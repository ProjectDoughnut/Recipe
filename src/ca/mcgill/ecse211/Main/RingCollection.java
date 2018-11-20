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
	private float[] origin;
	private float[][] tunnel;
	private float[][] island;
	private float[] tree;
	private double WHEEL_RADIUS;
	private double toBranch = 10;
	public static Object done = new Object();
	
	public boolean running;
	
	public RingCollection(Odometer odo, Navigation nav, EV3MediumRegulatedMotor servo, float[] origin, float[][] tunnel, float[] tree, float[][] island, double WHEEL_RADIUS) {
		this.odo = odo;
		this.nav = nav;
		this.servo = servo;
		this.origin = origin;
		this.tree = tree;
		this.tunnel = tunnel;
		this.island = island;
		this.WHEEL_RADIUS = WHEEL_RADIUS;
		this.running = true;
	}
	
	public void run() {
		
		//Lock the color thread until this thread begins

		/*
		 * Check that the robot is done navigating to tree first?????
		 * Then, determine which orientation it is in
		 */
  		int yOnRight = 0;
  		int xOnRight = 0;
  		float[] tunnelVector = new float []{tunnel[1][0] -tunnel[0][0],tunnel[1][1] -tunnel[0][1]};
  		
  		if (tunnelVector[0] > 0 && tunnelVector[1] > 0  || tunnelVector[0] < 0 && tunnelVector[1] < 0) {
  		// y tunnel
  			if (tree[0] > (tunnel[0][0] + tunnel[1][0])/2 ) {
  				yOnRight = -1;
  			} else {
  				yOnRight = 1;
  			}
  		} else if (tunnelVector[0] > 0 && tunnelVector[1] < 0 || tunnelVector[0] < 0 && tunnelVector[1] > 0) {
  			// x tunnel
  			if (tree[1] > (tunnel[0][1] + tunnel[1][1])/2 ) {
  				xOnRight = 1;
  			} else {
  				xOnRight = -1;
  			}
  		}
  		
  		for (int i = 0; i< 4; i++) {
  			// four sides of the square
  			//    -----x-----
  			//    |    |    |
  			//    x----O----x
  			//    |    |    |
  			//    -----x-----
  			
  			getRings();
  			// note that since only xOnRight or yOnRight
  			// one of the tmp values will actually have a change
  			float tmpX = origin[0] + xOnRight;
  			float tmpY = origin[1] + yOnRight;
  			nav.syncTravelTo(tmpX, tmpY);
  			// try to understand logic here
  			// basically, if x is some value, y will because the negative of x
  			// if y is some value, x will become y
  			int tmpXOnRight = xOnRight;
  			xOnRight = yOnRight;
  			yOnRight = -tmpXOnRight;

  			nav.syncTravelTo(tmpX-xOnRight, tmpY-yOnRight);
  			
  			// i skipped an intermediate step here
  			xOnRight = -xOnRight;
			yOnRight = -yOnRight;  			
  			
  			float angle = 0;
  			if (xOnRight == 1) {
  				angle = 0;
  			} else if (xOnRight == -1) {
  				angle = 180;
  			} else if (yOnRight == 1) {
  				angle = 270;
  			} else if (yOnRight == -1) {
  				angle = 90;
  			}
  			
  			nav.turnTo(odo.getXYT()[2], angle);
  			
  		}

	}

	/*
	 * This method collects the rings (if any) on the branch of the tree that the robot is currently facing. 
	 * This also runs under the assumption that the ring is already at the correct angle, facing the branch
	 */
	public void getRings() {
		servo.setSpeed(50);
		odo.leftMotor.setSpeed(80);
		odo.rightMotor.setSpeed(80);
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
