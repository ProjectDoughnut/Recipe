package ca.mcgill.ecse211.Ultrasonic;


import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class USLocalizer extends Thread implements UltrasonicController{

	public enum LocalizationType{FALLING_EDGE, RISING_EDGE};

	/**
	 * 
	 * RESET state ensure robot isn't within the raising or falling edge limit
	 * SEEK state seeks thetaA
	 * SEEK_2 state seeks thetaB
	 * CORRECTION state corrects robot direction
	 *
	 */
	public enum LocalizationState{RESET, SEEK, SEEK_2, CORRECTION, DONE};


	public static final double ROTATION_SPEED = 30.0;

	
	public static Object done = new Object();
	
	
	public boolean running;
	
	private Odometer odo;
	private LocalizationType type;
	private LocalizationState state;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private int prevDistance;
	public static double thetaA;
	public static double thetaB;
	public static double thetaAv;


	private double thetaZero;
	private int filterControl;
	
	
	public int FILTER_OUT = 20;
	public static int FILTER_OUT_DIST = 50;
	public static int WALL_DIST = 35;
	public static int WALL_ERROR = 5;
	public static int TURN_ERROR = 0;

	private static final int ROTATE_SPEED = 120;
	public static int ACCEL = 300;

	public USLocalizer(Odometer odo) {
		this.odo = odo;

		this.leftMotor = odo.leftMotor;
		this.rightMotor = odo.rightMotor;
		prevDistance = 50;
		this.state = LocalizationState.RESET;
		
		this.filterControl = 0;
		this.prevDistance = Integer.MAX_VALUE;
		
		
		// initialize motor speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		this.running = true;
	}

	public void setType(LocalizationType type) {
		this.type = type;

	}	

	@Override
	public boolean isRunning() {
		return this.running;
	}

	public void process(int distance) {

		
		// filter for the crease of the wall
		if (distance >= 255 && filterControl < FILTER_OUT && prevDistance < distance) {
			filterControl++;
			distance = prevDistance;
		} else if (distance >= 255) {
			// do nothing
		} else {
			filterControl = 0;
		}
		
		if (type == LocalizationType.FALLING_EDGE) {
			// Robot will rotate clockwise until there is no wall in front of it
			// and further more to ensure it goes to falling edge

			switch(state) {
				case RESET:
					//If it is far from the wall, stop the motors, set theta=0 and start seeking thetaA (SEEK state)
					if (distance >= FILTER_OUT_DIST) {
						leftMotor.stop(true);
						rightMotor.stop(false);
						odo.setTheta(0);
						Sound.twoBeeps();
						this.state = LocalizationState.SEEK;
					} 
					//If it is not too far from the wall, start moving counterclockwise until it is
					else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case SEEK:
					//Here, it is seeing the first falling edge:
					//Stop the motors, get thetaA, rotate clockwise (by thetaA amount), start looking for thetaB (SEEK_2 case)
					if ((distance <= WALL_DIST + WALL_ERROR) && (prevDistance > WALL_DIST + WALL_ERROR))  {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaA = odo.getXYT()[2];
						Sound.beep();
						leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK,-thetaA), true);
						rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, -thetaA), false);
						this.state = LocalizationState.SEEK_2;
					} 
					//If no falling edge detected, just keep moving clockwise
					else {
						leftMotor.forward();
						rightMotor.backward();
					}
					break;
				case SEEK_2:
					//Detects second falling edge: stop motors, get thetaB, start correcting (CORRECTION case)
					if ((distance <= WALL_DIST + WALL_ERROR) && (prevDistance > WALL_DIST + WALL_ERROR)) {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaB = odo.getXYT()[2] - 360;
						Sound.beep();
						this.state = LocalizationState.CORRECTION;
					} 
					//If no falling edge, keep moving counterclockwise
					else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case CORRECTION:			
					
					//thetaAv will be 45 degrees away from zero degrees
					this.thetaAv = (this.thetaA - this.thetaB)/2.0;
					this.thetaZero = thetaAv - 45.0;
					

					//Rotate in the direction of thetaZero
					leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), true);
					rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), false);

					leftMotor.stop();
					rightMotor.stop();
					odo.setTheta(0);
					odo.setTheta(0);
					
					this.state = LocalizationState.DONE;
					break;	
				case DONE:
					synchronized(USLocalizer.done) {
						LightLocalizer.lock = null;
						USLocalizer.done.notifyAll();
					}
					this.running = false;
					break;
				default:
					break;
			}
		} else if (type == LocalizationType.RISING_EDGE) {
			// Robot will rotate counterclockwise until there is no wall in front of it
			// and further more to ensure it goes to rising edge
			switch(state) {
				case RESET:
					//If it is close to the wall, stop the motors, set theta=0 and start seeking thetaA (SEEK case)
					if (distance < WALL_DIST - WALL_ERROR) {
						leftMotor.stop(true);
						rightMotor.stop(false);
						odo.setTheta(0);
						Sound.twoBeeps();
						this.state = LocalizationState.SEEK;
					}
					//Otherwise, just keep going counterclockwise until it sees something
					else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case SEEK:
					//Here, it is seeing the first rising edge:
					//Stop the motors, get thetaA, rotate clockwise (by thetaA amount), start looking for thetaB (SEEK_2 case)
					if ((distance >= WALL_DIST - WALL_ERROR) && (prevDistance < WALL_DIST + WALL_ERROR))  {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaA = odo.getXYT()[2];
						Sound.beep();
						leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK,-thetaA), true);
						rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, -thetaA), false);
						this.state = LocalizationState.SEEK_2;
					}
					//If no rising edge detected, just keep moving clockwise
					else {
						leftMotor.forward();
						rightMotor.backward();
					}
					break;
				case SEEK_2:
					//Detects second rising edge: stop motors, get thetaB, start correcting (CORRECTION case)
					if ((distance >= WALL_DIST - WALL_ERROR) && (prevDistance < WALL_DIST + WALL_ERROR))  {
						leftMotor.stop(true);
						rightMotor.stop(false);
						thetaB = odo.getXYT()[2] - 360;
						Sound.beep();
						this.state = LocalizationState.CORRECTION;
					} 
					//Otherwise, just keep going counterclockwise until it sees something
					else {
						leftMotor.backward();
						rightMotor.forward();
					}
					break;
				case CORRECTION:
					//thetaAv will be 45 degrees away from zero degrees
					this.thetaAv = (this.thetaA - this.thetaB)/2.0;
					this.thetaZero = this.thetaAv + 135;

	
					//Rotate in the direction of thetaZero
					leftMotor.rotate(convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), true);
					rightMotor.rotate(-convertAngle(odo.WHEEL_RAD, odo.TRACK, thetaZero + TURN_ERROR), false);
	
					//Set the odometer theta to 0
					leftMotor.stop();
					rightMotor.stop();
					odo.setTheta(0);
					odo.setTheta(0);
					//odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {false, false, true});
					this.state = LocalizationState.DONE;
					break;
				case DONE:
					synchronized(USLocalizer.done) {
						LightLocalizer.lock = null;
						USLocalizer.done.notifyAll();
					}
					this.running = false;
					break;
				default:
					break;
				
			}
		}
		
		// set previous distance
		// this is called before any seeks in the reset state to ensure
		// that this prevDistance will be initialized
		this.prevDistance = distance;

	}

	@Override
	public Object getLock() {
		return null;
	}

	private  int convertDistance(double radius, double distance)
	{ 															 
		return (int) ((180.0 * distance) / (Math.PI * radius)); 
	} 

	private  int convertAngle(double radius, double width, double angle) 
	{ 
		return convertDistance(radius, Math.PI * width * angle / 360.0); 
	}


}
