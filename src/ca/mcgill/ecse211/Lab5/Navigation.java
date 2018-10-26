package ca.mcgill.ecse211.Lab5;

import java.text.DecimalFormat;
import java.util.ArrayList;

import ca.mcgill.ecse211.Gyro.AngleSampler;
import ca.mcgill.ecse211.Light.LightCorrector;
import ca.mcgill.ecse211.Odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread{

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 130;
	private final double WHEEL_RAD;
	private final double WHEEL_BASE;
	private final double TILE_SIZE;

	private Odometer odometer;
	private AngleSampler gyro;
	private LightCorrector corrector;

	private double x, y, theta;

	public static double destX, destY, destT;

	public static Object lock;

	private ArrayList<double[]> _coordsList;
	private boolean isNavigating;

	private volatile boolean running;

	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			final double WHEEL_RAD, final double WHEEL_BASE, double tileSize) {

		this(odo, null, null, leftMotor, rightMotor, 
				WHEEL_RAD,WHEEL_BASE, tileSize);
	}

	public Navigation(Odometer odo, AngleSampler gyro, LightCorrector corrector, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			final double WHEEL_RAD, final double WHEEL_BASE, double tileSize) {
		this.odometer = odo;
		this.gyro = gyro;
		this.corrector = corrector;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.WHEEL_BASE = WHEEL_BASE;
		this.TILE_SIZE = tileSize;
		this.isNavigating = false;
		this._coordsList = new ArrayList<double[]>();

	}


	/**
	 * Adds the navigation points to the coordList, setting up the thread
	 * 
	 * @param navX coordinate of position
	 * @param navY coordinate of position
	 */
	public void travelTo(double navX, double navY) {
		this._coordsList.add(new double[] {navX*TILE_SIZE, navY*TILE_SIZE});
	}

	//The run method runs through the _coordsList and travels through the points
	public void run() {
		this.running = true;
		while (!this._coordsList.isEmpty()) {
			if (!this.isRunning()) {
				break;
			}
			double[] coords = this._coordsList.remove(0);
			this._travelTo(coords[0], coords[1]);
		}
		this.running = false;
	}

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
		// correct angle with gyro
		if (this.gyro != null) {
			float theta = this.gyro.getTheta();
			this.odometer.setTheta(theta);
			this._turnTo(theta, deltaTheta);			
		}
		// move until destination is reached
		// while loop is used in case of collision override
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		this.isNavigating = true;
		leftMotor.forward();
		rightMotor.forward();

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
		
		return true;
	}

	public void syncTravelTo(double navX, double navY) {
		_travelTo(navX*TILE_SIZE, navY*TILE_SIZE);
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

	public void turnTo(double currTheta, double destTheta) {
		_turnTo(currTheta, destTheta);
	}

	//Getting the minimum angle to turn:
	//It is easier to turn +90 than -270
	//Also, it is easier to turn -90 than +270
	double normalizeAngle(double theta) {
		if (theta <= -180) {
			theta += 360;
		}
		else if (theta > 180) {
			theta -= 360;
		}
		return theta;
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

	public void setGyro(AngleSampler gyro) {
		this.gyro = gyro;
	}

	public void setCorrector(LightCorrector corrector) {
		this.corrector = corrector;
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
}

