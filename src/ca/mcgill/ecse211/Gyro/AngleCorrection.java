package ca.mcgill.ecse211.Gyro;

import ca.mcgill.ecse211.Gyro.AngleSampler;
import ca.mcgill.ecse211.Light.LightPoller;
import ca.mcgill.ecse211.Main.Navigation;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import lejos.hardware.lcd.LCD;

/**
 * This class corrects the odometer value
 * @author Lucy Coyle
 */

public class AngleCorrection implements Runnable {

	private LightPoller lsPoller;
	private Navigation nav;
	private AngleSampler gyro;
	private Odometer odo;
	
  /**
   * This is the default constructor of this class. It initiates all motors and variables once.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param TRACK
   * @param WHEEL_RAD
 * @throws OdometerExceptions 
   */
  public AngleCorrection(Navigation nav, AngleSampler gp) throws OdometerExceptions{


    this.nav = nav;
    this.gyro = gp;
    this.odo = Odometer.getOdometer();

  }


  /**
   * This method is where the logic for the odometry correction is implemented
   */

  public void run() {
	  
	  while(true) {

		  float angle = gyro.getTheta();
		  odo.setTheta(angle);
//		  LCD.drawString("Angle: " + angle, 0, 7);
		  
		  try {Thread.sleep(500);
			} catch (InterruptedException e) {}
	  }
    
  }

}


