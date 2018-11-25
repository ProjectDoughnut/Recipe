package ca.mcgill.ecse211.Main;

import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import lejos.hardware.Sound;
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
	private double toBranch = 7.0;
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
  		
  		float[][] coordinates = new float[9][];
  		float[] angles = new float[4];
  		coordinates[0] = origin.clone();
  		for (int i = 0; i< 4; i++) {
  			// four sides of the square
  			//    -----x-----
  			//    |    |    |
  			//    x----O----x
  			//    |    |    |
  			//    -----x-----
  			
  			
  			// turn to the right heading
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
  			
  			angles[i] = angle;
  			
//  			getRings();
  			// note that since only xOnRight or yOnRight
  			// one of the tmp values will actually have a change
  			float tmpX = origin[0] + xOnRight;
  			float tmpY = origin[1] + yOnRight;
  			coordinates[2*i+1] = new float[] {tmpX, tmpY};
  			// try to understand logic here
  			// basically, if x is some value, y will because the negative of x
  			// if y is some value, x will become y
  			int tmpXOnRight = xOnRight;
  			xOnRight = yOnRight;
  			yOnRight = -tmpXOnRight;

  			coordinates[2*i+2] = new float[] {tmpX-xOnRight, tmpY-yOnRight};
  			origin = new float[]{tmpX-xOnRight, tmpY-yOnRight};
  			// i skipped an intermediate step here
  			xOnRight = -xOnRight;
			yOnRight = -yOnRight;
  		}
  		
  		
  		// figure which points are inside the island/wall
  		
  		for (int i = 0; i < 8; i++) {
  			if (!isIn(island, coordinates[i])) {
  				coordinates[i] = null;
  			}
  		}
  		
  		// go in clockwise direction until encountering the point
  		// that can get the maximum of places by only going counter clockwise
  		int offset = 0;
  		for (int i = 0; i < 8; i++) {
  			if (coordinates[i] == null) {
  				offset = (i+1)%8; //mod 8 means it is the first element
  			}
  		}

  	
  		if (offset !=0) {
  		// go clockwise to reach the initial point if offset is not 0
  	  		for (int i = 7; i >= offset; i--) {
  	  			nav.syncTravelTo(coordinates[i][0], coordinates[i][1]);
  	  		}
  		}
  		
  		// realCoordinates represents the list of actual coordinates to go to
  		// keep track of where it ends relative to the start
  		int endOffset = 0;
  		for (int i = 0; i < 8; i++) {
  			if (i %2 == 0) {
  	  			nav.turnTo(odo.getXYT()[2], angles[((i+offset)%8)/2]);
  	  			getRings();
  			}
  			if (coordinates[(i+offset+1)%8] != null) {
  				nav.syncTravelTo(coordinates[(i+offset+1)%8][0], coordinates[(i+offset+1)%8][1]);
  			} else {
  				endOffset = (i+offset)%8;
  				break;
  			}
  		}
  		
  		// go back to the original place so we can travel back using 
  		// the exact same algorithm used to navigate here
  		
  		if (endOffset != 0) {
  			for (int i = endOffset -1; i> 0; i--) {
  				nav.syncTravelTo(coordinates[i][0], coordinates[i][1]);
  			}
  		}
	}
	
	public static boolean isIn(float[][] area, float[] point) {
		boolean overlapX = area[1][0] - area[0][0] >= point[0] - area[0][0];
		boolean overlapY = area[1][1] - area[0][1] >= point[1] - area[0][1];
		
		return overlapX && overlapY;
	}

	/*
	 * This method collects the rings (if any) on the branch of the tree that the robot is currently facing. 
	 * This also runs under the assumption that the ring is already at the correct angle, facing the branch
	 */
	public void getRings() {
		servo.setSpeed(30);
		odo.leftMotor.setSpeed(80);
		odo.rightMotor.setSpeed(80);
		//Open claw
		servo.rotate(90);
		
		//Move the robot forward a set amount: This should be tested multiple times to determine the ideal amount
		odo.leftMotor.rotate(Navigation.convertDistance(WHEEL_RADIUS, toBranch), true);
		odo.rightMotor.rotate(Navigation.convertDistance(WHEEL_RADIUS, toBranch), false);
		
		//Close claw
		servo.rotate(-85);
		
		//Move the robot backward a set amount: This should be tested multiple times to determine the ideal amount
		odo.leftMotor.rotate(-Navigation.convertDistance(WHEEL_RADIUS, toBranch), true);
		odo.rightMotor.rotate(-Navigation.convertDistance(WHEEL_RADIUS, toBranch), false);
	}
	

}
