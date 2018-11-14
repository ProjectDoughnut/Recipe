package ca.mcgill.ecse211.Main;

import java.util.ArrayList;

import ca.mcgill.ecse211.Gyro.AngleSampler;
import ca.mcgill.ecse211.Odometer.OdometryCorrector;
import ca.mcgill.ecse211.Odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * 
 * Navigation class drives the robot to given coordinates [x,y]. Contains methods for moving in a straight line
 * and change heading 
 *
 */

public class Navigation extends Thread{

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 130;

	private final double WHEEL_RAD;
	private final double WHEEL_BASE;
	private final double TILE_SIZE;

	private Odometer odometer;

	private double x, y, theta;

	public static double destX, destY, destT;

	public static Object lock;
	public static boolean tunnelPointingX;
	public static boolean tunnelToLeft;
	public static boolean tunnelUp;
	public static int yOnRight;
	public static int xOnRight;

	private ArrayList<double[]> _coordsList;
	private boolean isNavigating;
	private boolean selfCorrecting = false;


	private volatile boolean running;
	private double sensorDistance = 	11.3;
	private double corrX;
	private double ERROR_THRESHOLD = 5.0;
	private double corrY;


	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			final double WHEEL_RAD, final double WHEEL_BASE, double tileSize) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.WHEEL_BASE = WHEEL_BASE;
		this.TILE_SIZE = tileSize;
		this.isNavigating = false;
		this._coordsList = new ArrayList<double[]>();
		this.running = true;

	}


	/**
	 * Adds the navigation points to the _coordList
	 * 
	 * @param navX coordinate of position
	 * @param navY coordinate of position
	 */
	public void travelTo(double navX, double navY) {
		this._coordsList.add(new double[] {navX*TILE_SIZE, navY*TILE_SIZE});
	}

	/**
	 * Navigates to all coordinates sotred in _coordsList
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		this.running = true;
		while (!this._coordsList.isEmpty()) {
			//while(Button.waitForAnyPress() != Button.ID_ENTER);
			if (!this.isRunning()) {
				break;
			}
			double[] coords = this._coordsList.remove(0);

			this._travelTo(coords[0], coords[1]);

			//			if (selfCorrecting) {
			//				double xyt[] = odometer.getXYT();
			//				double deltaX = coords[0] - xyt[0];
			//				double deltaY = coords[1] - xyt[1];
			//
			//				double magnitudeSqr = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);
			//				if (magnitudeSqr > 8) {
			//
			//				}
			//			}

		}
		this.running = false;
	}

	/**
	 * Synchronized travel to navX, navY.
	 * 
	 * @param navX
	 * @param navY
	 * @return boolean True if navigation complete successfully
	 */
	boolean _travelTo(double navX, double navY) {

		// get current coordinates
		double xyt[] = odometer.getXYT();

		theta = xyt[2];
		x = xyt[0];
		y = xyt[1];	


		double deltaX = navX - x;
		double deltaY = navY - y;


		double magnitudeSqr = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);
		//need to convert theta from degrees to radians
		double deltaTheta = Math.atan2(deltaX, deltaY) / Math.PI * 180;

		destX = navX;
		destY = navY;
		destT = deltaTheta;

		// turn to the correct direction
		this._turnTo(theta, deltaTheta);
		Sound.beep();
		// move until destination is reached
		// while loop is used in case of collision override
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		this.isNavigating = true;
		leftMotor.forward();
		rightMotor.forward();
		TextLCD lcd = LocalEV3.get().getTextLCD();

		Sound.twoBeeps();
		// @todo might consider using rotate function for fix amount of distance using the .rotate instead
		while(true) {


			double newTheta, newX, newY;

			double newXyt[] = odometer.getXYT();
			//need to convert theta from degrees to radians
			newTheta = newXyt[2];
			newX = newXyt[0];
			newY = newXyt[1];	

			//If the difference between the current x/y and the x/y we started from is similar to the deltaX/deltaY, 
			//Stop the motors because the point has been reached
			if (Math.pow(newX - x, 2) + Math.pow(newY - y, 2) >= magnitudeSqr) {
				break;
			}

			// This is only relevant if the ultrasonic poller thread is being used
			if (lock != null) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				synchronized(lock) {
					try {
						lock.wait();
						this._coordsList.add(0, new double[] {navX, navY});
						return false;
					} catch(InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			// stop the navigation entirely
			if (!running) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				this._coordsList.add(0, new double[] {navX, navY});
				return false;
			}

			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {

				e.printStackTrace();
			}

		}

		leftMotor.stop(true);
		rightMotor.stop(false);
		this.isNavigating = false;
		Sound.beepSequence();
		return true;
	}



	/**
	 * Call synchonized _travelTo function when running ouside a seperate thread instance.
	 * 
	 * @param navX
	 * @param navY
	 */
	public void syncTravelTo(double navX, double navY) {
		this.running = true;
		_travelTo(navX*TILE_SIZE, navY*TILE_SIZE);
		this.running = false;
	}


	/**
	 * 
	 * This method causes the robot to turn (on point) to the absolute (minima) heading theta. This method
	 * should turn a MINIMAL angle to its target.
	 * 
	 * @param currTheta current theta
	 * @param destTheta to turn robot by
	 */

	void _turnTo(double currTheta, double destTheta) {
		// get theta difference
		double deltaTheta = destTheta - currTheta;
		// normalize theta (get minimum value)
		deltaTheta = normalizeAngle(deltaTheta);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		this.isNavigating = true;
		leftMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, deltaTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, deltaTheta), false);
		this.isNavigating = false;
	}



	/**
	 * Synchronized public turn method
	 * 
	 * @param currTheta
	 * @param destTheta
	 */
	public void turnTo(double currTheta, double destTheta) {
		_turnTo(currTheta, destTheta);
	}


	/**
	 * Normalized the angle so that the turn is always < 180 degrees
	 * 
	 * @param theta
	 * @return
	 */
	double normalizeAngle(double theta) {
		return (((theta-180)%360+360)%360)-180;
	}



	boolean isNavigating() {
		return isNavigating;
	}

	public boolean isRunning() {
		return running;
	}

	public void setRunning(boolean running) {
		this.running = running;
	}

	public void clearCoordList() {
		this._coordsList.clear();
	}


	/**
	 * @param startIsland
	 * @param endIsland
	 * @param tunnel
	 * @param tree
	 * @return
	 */
	public static float[][] pathing(int[] corner, float tunnel[][], float[] tree) {
		ArrayList<Object> paths = new ArrayList<Object>();
		float[] tunnelVector = new float []{tunnel[1][0] -tunnel[0][0],tunnel[1][1] -tunnel[0][1]};
		yOnRight = 0;
		xOnRight = 0;
		if (tunnelVector[0] > 0 && tunnelVector[1] > 0 ) {
			yOnRight = 1;

		} else if (tunnelVector[0] < 0 && tunnelVector[1] < 0) {
			yOnRight = -1;
		} else if (tunnelVector[0] > 0 && tunnelVector[1] < 0) {
			xOnRight = 1;
		} else if(tunnelVector[0] < 0 && tunnelVector[1] > 0) {
			xOnRight = -1;
		}

		if (xOnRight !=0 && yOnRight == 0 ) {

			tunnelPointingX = true;
			paths.add(new float[]{corner[0], (tunnel[0][1] + tunnel[1][1])/2});

			// if the position of the tree is less far than the one of the tunnel (tunnel overlap with terrain)
			// we will get around it instead
			if (Math.abs(tree[0] - tunnel[0][0]) < Math.abs(tunnel[1][0]-tunnel[0][0])) {
				// go one step farther than the tunnel
				float tunnelXpp = tunnel[1][0]+xOnRight*1;

				paths.add(new float[]{tunnelXpp, (tunnel[0][1] + tunnel[1][1])/2});

				yOnRight = -xOnRight;
				xOnRight = 0;

				if (!(tree[1] > (tunnel[0][1] + tunnel[1][1])/2 ^ yOnRight == 1)) {
					float tmp = tunnel[1][1]+1*yOnRight;
					paths.add(new float[]{tunnelXpp, tmp});
					xOnRight = yOnRight;
					yOnRight = 0;
					paths.add(new float[]{tree[0], tmp});
					yOnRight = -xOnRight;
					xOnRight = 0;
				} else {
					float tmp = tunnel[0][1]-1*yOnRight;
					paths.add(new float[]{tunnelXpp, tmp});
					xOnRight = -yOnRight;
					yOnRight = 0;
					paths.add(new float[]{tree[0], tmp});
					yOnRight = xOnRight;
					xOnRight = 0;
				}

			} else if (tree[1] == tunnel[0][1] || tree[0] == tunnel[1][1]) {
				// go one step farther than the tunnel
				float tunnelXpp = tunnel[1][0]+xOnRight*1;

				paths.add(new float[]{tunnelXpp, (tunnel[0][1] + tunnel[1][1])/2});

				yOnRight = -xOnRight;
				xOnRight = 0;

				if (tree[1] == tunnel[0][1]) {
					float tmp = tunnel[1][1]+1*yOnRight;
					paths.add(new float[]{tunnelXpp, tmp});
					xOnRight = yOnRight;
					yOnRight = 0;
					paths.add(new float[]{tree[0], tmp});
					yOnRight = -xOnRight;
					xOnRight = 0;
				} else {
					float tmp = tunnel[0][1]-1*yOnRight;
					paths.add(new float[]{tunnelXpp, tmp});
					xOnRight = -yOnRight;
					yOnRight = 0;
					paths.add(new float[]{tree[0], tmp});
					yOnRight = xOnRight;
					xOnRight = 0;
				}
			} else {
				paths.add(new float[]{tree[0], (tunnel[0][1] + tunnel[1][1])/2});
				yOnRight = -xOnRight;
				xOnRight = 0;
			}
			// go to before the y coordinate
			if (tree[1] > (tunnel[0][1] + tunnel[1][1])/2) {
				paths.add(new float[]{tree[0] + 0.5f, tree[1] - 1.0f});
				tunnelUp = true;
			} else {
				paths.add(new float[]{tree[0] + 0.5f, tree[1] + 1.5f});
				tunnelUp = false;
			}


		} else {
			tunnelPointingX = false;
			paths.add(new float[]{((tunnel[0][0] + tunnel[1][0])/2), corner[1]});
			if (Math.abs(tree[1] - tunnel[0][1]) < Math.abs(tunnel[1][1]-tunnel[0][1])) {
				float tunnelYpp = tunnel[1][1] +yOnRight*1;
				paths.add(new float[]{(tunnel[0][0] + tunnel[1][0])/2, tunnelYpp});
				xOnRight = -yOnRight;
				yOnRight = 0;

				if (!(tree[0] > (tunnel[0][0] + tunnel[1][0])/2 ^ xOnRight == 1)) {
					float tmp = tunnel[0][0]+1*xOnRight;
					paths.add(new float[]{tmp, tunnelYpp});
					yOnRight = -xOnRight;
					xOnRight = 0;
					paths.add(new float[]{tmp, tree[1]});
					xOnRight = yOnRight;
					yOnRight = 0;

				} else {
					float tmp = tunnel[1][0]-1*xOnRight;
					paths.add(new float[]{tmp, tunnelYpp});
					yOnRight = xOnRight;
					xOnRight = 0;
					paths.add(new float[]{tmp, tree[1]});
					xOnRight = -yOnRight;
					yOnRight = 0;
				}
			} else if (tree[0] == tunnel[0][0] || tree[0] == tunnel[1][0]) {
				// if the tree is exactly on the 
				float tunnelYpp = tunnel[1][1] +yOnRight*1;
				paths.add(new float[]{(tunnel[0][0] + tunnel[1][0])/2, tunnelYpp});
				xOnRight = -yOnRight;
				yOnRight = 0;
				if (tree[0] == tunnel[1][0]) {
					float tmp = tunnel[0][0]+1*xOnRight;
					paths.add(new float[]{tmp, tunnelYpp});
					yOnRight = -xOnRight;
					xOnRight = 0;
					paths.add(new float[]{tmp, tree[1]});
					xOnRight = yOnRight;
					yOnRight = 0;

				} else {
					float tmp = tunnel[1][0]-1*xOnRight;
					paths.add(new float[]{tmp, tunnelYpp});
					yOnRight = xOnRight;
					xOnRight = 0;
					paths.add(new float[]{tmp, tree[1]});
					xOnRight = -yOnRight;
					yOnRight = 0;
				}

			} else {
				paths.add(new float[]{(tunnel[0][0] + tunnel[1][0])/2, tree[1]});

			}
			// go to before the x coordinate
			if (tree[0] > (tunnel[0][0] + tunnel[1][0])/2) {
				paths.add(new float[]{tree[0]-1.5f, tree[1] + 0.5f});
				tunnelToLeft = false;
			} else {
				paths.add(new float[]{tree[0] + 1.0f, tree[1] +0.5f});
				tunnelToLeft = true;
			}

		}
		float[][] pathsArr = new float[paths.size()][];
		int i = 0;
		for (Object o: paths) {
			pathsArr[i++] = (float[]) o;
		}
		return pathsArr;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}


	public boolean betweenLines() {

		double[] position = odometer.getXYT();

		if(position[0] % TILE_SIZE < (TILE_SIZE/8) && position[1]%TILE_SIZE < (TILE_SIZE/8)) {
			return false;
		}
		return true;

	}

	public void adjustOdometer() {
		double[] xyt = odometer.getXYT();

		double lineX, lineY, errorX, errorY, deltaX, deltaY;

		deltaX = Math.sin(Math.toRadians(xyt[2])) * sensorDistance;
		deltaY = Math.cos(Math.toRadians(xyt[2])) * sensorDistance;
		lineX = xyt[0] - deltaX;
		lineY = xyt[1] - deltaY;

		errorX = lineX % TILE_SIZE;
		if (errorX >= TILE_SIZE/2) {
			errorX -= TILE_SIZE;
		} else if (errorX <= -TILE_SIZE/2) {
			errorX += TILE_SIZE;
		}

		errorY = lineY % TILE_SIZE;
		if (errorY >= TILE_SIZE/2) {
			errorY -= TILE_SIZE;
		} else if (errorY <= -TILE_SIZE/2) {
			errorY += TILE_SIZE;
		}

		if (Math.abs(errorX) <= ERROR_THRESHOLD && errorX <= errorY) {
			corrX = -errorX;

			odometer.update(corrX, 0, 0);

		} else if (Math.abs(errorY) <= ERROR_THRESHOLD) {
			// probably y line

			corrY = -errorY;
			odometer.update(0, corrY, 0);


		}

	}


	public void adjustTheta(float a) {
		odometer.setTheta(a);
	}
}

