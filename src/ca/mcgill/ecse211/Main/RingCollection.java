package ca.mcgill.ecse211.Main;

import ca.mcgill.ecse211.Odometer.Odometer;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class RingCollection {

	private Odometer odo;
	private Navigation nav;
	private EV3MediumRegulatedMotor servo;
	
	public RingCollection(Odometer odo, Navigation nav, EV3MediumRegulatedMotor servo) {
		this.odo = odo;
		this.nav = nav;
		this.servo = servo;
	}
	
	public void run() {
		
		
	}
}
