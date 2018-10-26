package ca.mcgill.ecse211.Lab5;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;


public class Display extends Thread implements Runnable {

	  private Odometer odo;
	  private TextLCD lcd;
	  private double[] position;
	  private final long DISPLAY_PERIOD = 25;
	  private long timeout = Long.MAX_VALUE;

	  /**
	   * This is the class constructor
	   * 
	   * @param odoData
	   * @throws OdometerExceptions 
	   */
	  public Display(TextLCD lcd) throws OdometerExceptions {
	    odo = Odometer.getOdometer();
	    this.lcd = lcd;
	  }

	  /**
	   * This is the overloaded class constructor
	   * 
	   * @param odoData
	   * @throws OdometerExceptions 
	   */
	  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
	    odo = Odometer.getOdometer();
	    this.timeout = timeout;
	    this.lcd = lcd;
	  }

	public void run() {
	    lcd.clear();
	    
	    long updateStart, updateEnd;

	    long tStart = System.currentTimeMillis();
	    do {
	      updateStart = System.currentTimeMillis();

	      // Retrieve x, y and Theta information
	      position = odo.getXYT();

	  	  if (ColorClassifier.detectedRing != null) {
	  		  lcd.clear();
	    	  	  lcd.drawString("Object Detected!", 0, 0);
		      lcd.drawString(ColorClassifier.detectedRing.toString(), 0, 1);

		      	try {
		      		Thread.sleep(1000);
		      		lcd.clear();
		      	} catch (InterruptedException e) {
				
		      		e.printStackTrace();
			} 

	      }
	  	  
	      if (odo != null) {
	    	  
		      // Print x,y, and theta information
		      DecimalFormat numberFormat = new DecimalFormat("######0.00");
		      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
		      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
		      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
		      
		  	  if (ColorClassifier.detectedRing != null) {
		  		  lcd.clear();
		    	  	  lcd.drawString("Object Detected!", 0, 0);
			      lcd.drawString(ColorClassifier.detectedRing.toString(), 0, 1);
			      	try {
			      		Thread.sleep(1000);
			      		lcd.clear();
			      	} catch (InterruptedException e) {
					
			      		e.printStackTrace();
				} 
			     
			    if (ColorClassifier.targetDetected) {
			    		lcd.clear();
			    		lcd.drawString("Target Information", 0, 0);
			    		lcd.drawString("X: " + ColorClassifier.targetLocation[0], 0, 1);
			    		lcd.drawString("Y: " + ColorClassifier.targetLocation[1], 0, 2);
			    		lcd.drawString("T: " + ColorClassifier.targetLocation[2], 0, 3);
			    		lcd.drawString("Red Sample: " + ColorClassifier.targetRGBValues[0], 0, 4);
			    		lcd.drawString("Green Sample: " + ColorClassifier.targetRGBValues[1], 0, 5);
			    		lcd.drawString("Blue Sample: " + ColorClassifier.targetRGBValues[2], 0, 6);
			    }

		      }
	      }
	      
	  
	      
	      // this ensures that the data is updated only once every period
	      updateEnd = System.currentTimeMillis();
	      if (updateEnd - updateStart < DISPLAY_PERIOD) {
	        try {
	          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
	        } catch (InterruptedException e) {
	          e.printStackTrace();
	        }
	      }
	    } while ((updateEnd - tStart) <= timeout);

	  }

	}


