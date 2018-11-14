package ca.mcgill.ecse211.Main;

import ca.mcgill.ecse211.Ultrasonic.*;


import ca.mcgill.ecse211.Ultrasonic.USLocalizer.LocalizationType;
import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Color.ColorClassifier.RingColors;
import ca.mcgill.ecse211.Gyro.AngleCorrection;
import ca.mcgill.ecse211.Gyro.AngleSampler;
import ca.mcgill.ecse211.Color.ColorPoller;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Light.LightPoller;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.UARTSensor;
import lejos.robotics.SampleProvider;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
* 
* The starting point of the robot's program. All motors, sensors and other constants required are defined here.
* Upon launch of the program the user will be prompted and based on the user's action a set of instructions will
* be carried out by the robot
*
*/

public class Main {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3MediumRegulatedMotor clawServo = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

	private static final Port usPort = LocalEV3.get().getPort("S3");
	private static final Port gyroPort = LocalEV3.get().getPort("S4");
	private static final Port lsPort = LocalEV3.get().getPort("S2");
	private static final Port csPort = LocalEV3.get().getPort("S1");

	//Setting up ultrasonic sensor
	public static UARTSensor usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usValue = usSensor.getMode("Distance");

	//Setting up gyro sensor 
	public static EV3GyroSensor gyroSensor = new EV3GyroSensor(gyroPort);
	public static SampleProvider gyroValue = gyroSensor.getMode("Angle");
	
	//Setting up light sensor

	public static UARTSensor lsSensor = new EV3ColorSensor(lsPort);
	public static SampleProvider lsValue = lsSensor.getMode("Red");
	
	public static EV3ColorSensor csSensor = new EV3ColorSensor(csPort);
	public static SampleProvider csValue = csSensor.getRGBMode();
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	public static final double WHEEL_BASE = 12.2;
	public static final double TILE_SIZE = 30.48;
	public static final double sensorDistance = 11.3;
	
	
	private static boolean red = false;
	private static boolean green = false;
	private static int corner;
	private static int[] cornerXY = new int[2];
	private static float[][] island = new float[2][2];
	private static float[][] home = new float[2][2];
	private static float[][] tunnel = new float[2][2];
	private static float[] tree = new float[2];
	public static float[][] pathToTree;
	private static RingColors targetRing;
	private static final String SERVER_IP = "192.168.2.67";
	private static final int TEAM_NUMBER = 2;
	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;


	public static void main(String[] args) throws OdometerExceptions, UnknownHostException, IOException, ParseException {
		// init thread to exit application
		Thread exitThread = new Thread() {
			public void run() {
				while (Button.waitForAnyPress() != Button.ID_ESCAPE);
				System.exit(0);
			}
		};
		exitThread.start();
		
	    
		getWiFiParameters();
		lcd.clear();
	    
		int buttonChoice;

		//Setting up the odometer and display
		
		Odometer odo = Odometer.getOdometer(leftMotor, rightMotor, WHEEL_BASE, WHEEL_RAD);
		Navigation nav = new Navigation(odo, leftMotor, rightMotor, WHEEL_RAD, WHEEL_BASE, TILE_SIZE);
		Display display = new Display(lcd);
		
		//define ultrasound localizer
		
		USLocalizer USLocal = new USLocalizer(odo);
		UltrasonicPoller usPoller = new UltrasonicPoller(usValue, USLocal);
		
		// define light localizer
		
		LightLocalizer LSLocal = new LightLocalizer(odo, nav);
		LightLocalizer.lock = USLocalizer.done;
		LightPoller lsPoller = new LightPoller(lsValue, LSLocal);

		
		// define gyro corrector
		AngleSampler gyro = new AngleSampler(gyroValue);
		
		
		// define light corrector

		ColorClassifier CSLocal = new ColorClassifier(odo, nav, targetRing, false);
		
		double[] xyt;

		ColorPoller csPoller = new ColorPoller(csValue, CSLocal);


		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Test  |  Full  ", 0, 2);
			lcd.drawString("       |  demo  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) { //US Localization has been selected
			// clear the display
			lcd.clear();
		      // ask the user whether odometery correction should be run or not
		      lcd.drawString("< Left | Right >", 0, 0);
		      lcd.drawString("       |Localize", 0, 1);
		      lcd.drawString("Get    | and    ", 0, 2);
		      lcd.drawString("Rings  | Nav-   ", 0, 3);
		      lcd.drawString("       | igate  ", 0, 4);

		    buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
//			USLocal.setType(LocalizationType.FALLING_EDGE);
		    Thread odoThread = new Thread(odo);
		    odoThread.start();
			Thread displayThread = new Thread(display);
			displayThread.start();

			if (buttonChoice == Button.ID_LEFT) {
				
				RingCollection ringCollector = new RingCollection(odo, nav, clawServo, tunnel, tree, island, WHEEL_RAD, sensorDistance);
				//Set which position its gonna be in: depending on where the tree is located
//				clawServo.setSpeed(45);
//				leftMotor.setSpeed(80);
//				rightMotor.setSpeed(80);
//				ringCollector.getRings();

				//Pointing y or x:
				Navigation.tunnelPointingX = true;
				
				//If x, is the tree further up or down:
				Navigation.tunnelUp = true;
				
				//If y, is the tree to the left or right:
				//Navigation.tunnelToLeft = false;
				//odo.setXYT(3*TILE_SIZE, 3*TILE_SIZE, 90);
				
				//Thread ringCollectThread = new Thread(ringCollector);
				//ringCollectThread.start();
				Thread colorThread = new Thread(csPoller);
				colorThread.start();
	
				

			}
			else if (buttonChoice == Button.ID_RIGHT) {
				USLocal.setType(LocalizationType.FALLING_EDGE);
				Thread usPollerThread = new Thread(usPoller);
				usPollerThread.start();
				Thread lsPollerThread = new Thread(lsPoller);
				lsPollerThread.start();
				//Navigate using wifi class and only along x, y lines

				try {
					usPollerThread.join();
					lsPollerThread.join();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				if (corner == 1) {
					odo.setXYT(7*TILE_SIZE, 1*TILE_SIZE, 270);
					cornerXY[0] = 7;
					cornerXY[1] = 1;
				}
				else if (corner == 2) {
					odo.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 90);
					cornerXY[0] = 7;
					cornerXY[1] = 7;
				}
				else if (corner == 3) {
					odo.setXYT(1*TILE_SIZE, 7*TILE_SIZE, 180);
					cornerXY[0] = 1;
					cornerXY[1] = 7;
				}
				else {
					cornerXY[0] = 1;
					cornerXY[1] = 1;
				}
				Sound.beep();
				//pathToTree = Navigation.pathing(cornerCoord, home, island, tunnel, tree);
				
				
				// add coordinates of tunnel and tree here

				float[][] paths = Navigation.pathing(cornerXY, tunnel, tree);

				for (float[] path: paths) {
					if (path[0] == cornerXY[0]) {
						if (corner == 0 || corner == 1)
							nav.travelTo(path[0], path[1] + 0.23f);
						else if (corner == 2 || corner == 3)
							nav.travelTo(path[0], path[1] - 0.23f);
					}
					else if (path[1] == cornerXY[1]) {
						if (corner == 0 || corner == 3)
							nav.travelTo(path[0] + 0.23f, path[1]);
						else if (corner == 1 || corner == 2)
							nav.travelTo(path[0] - 0.26f, path[1]);
					}
					else
						nav.travelTo(path[0], path[1]);
				}
				   
				Thread navThread = new Thread(nav);
				navThread.start();
			}
			
		
		} else { 
		    Thread odoThread = new Thread(odo);
		    odoThread.start();
			Thread displayThread = new Thread(display);
			displayThread.start();
			USLocal.setType(LocalizationType.FALLING_EDGE);
			Thread usPollerThread = new Thread(usPoller);
			usPollerThread.start();
			Thread lsPollerThread = new Thread(lsPoller);
			lsPollerThread.start();
			//Navigate using wifi class and only along x, y lines

			//pathToTree = Navigation.pathing(cornerCoord, home, island, tunnel, tree);

			try {
				usPollerThread.join();
				lsPollerThread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			

			if (corner == 0) {
				cornerXY[0] = 1;
				cornerXY[1] = 1;
			}
			else if (corner == 1) {
				odo.setXYT(7*TILE_SIZE, 1*TILE_SIZE, 270);
				cornerXY[0] = 7;
				cornerXY[1] = 1;
			}
			else if (corner == 2) {
				odo.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 90);
				cornerXY[0] = 7;
				cornerXY[1] = 7;
			}
			else if (corner == 3) {
				odo.setXYT(1*TILE_SIZE, 7*TILE_SIZE, 180);
				cornerXY[0] = 1;
				cornerXY[1] = 7;
			}
			Sound.beep();
			
			// add coordinates of tunnel and tree here
			float[][] paths = Navigation.pathing(cornerXY, tunnel, tree);
			for (float[] path: paths) {
				if (path[0] == cornerXY[0]) {
					if (corner == 0 || corner == 1)
						nav.travelTo(path[0], path[1] + 0.23f);
					else if (corner == 2 || corner == 3)
						nav.travelTo(path[0], path[1] - 0.23f);
				}
				else if (path[1] == cornerXY[1]) {
					if (corner == 0 || corner == 3)
						nav.travelTo(path[0] + 0.23f, path[1]);
					else if (corner == 1 || corner == 2)
						nav.travelTo(path[0] - 0.23f, path[1]);
				}
				else
					nav.travelTo(path[0], path[1]); 
			}
			   
			Thread navThread = new Thread(nav);
			navThread.start();
			
			try {
				navThread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			
			// start ring collection thread
			
			RingCollection ringCollector = new RingCollection(odo, nav, clawServo, tunnel, tree, island, WHEEL_RAD, sensorDistance);
			
			while(Button.waitForAnyPress() != Button.ID_ENTER);

			Thread ringCollectThread = new Thread(ringCollector);
			ringCollectThread.start();
			Thread colorThread = new Thread(csPoller);
			colorThread.start();
			
			
			// wait for color thread to join (ided one ring)
			try {
				colorThread.join();
				// EMERGENCY EXIT :)
				System.exit(0);
			
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
//			AngleCorrection angleCorrect = new AngleCorrection(nav, gyro);
//			Thread angleCorrectThread = new Thread(angleCorrect);
//			angleCorrectThread.start();
//			
//			
//			Thread lsCorrectThread = new Thread(lsCorrectorPoller);
//			lsCorrectThread.start();
			
			
		}


}
	
	public static void getWiFiParameters() {
		 // Initialize WifiConnection class
	    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

	    // Connect to server and get the data, catching any errors that might occur
	    try {
	      /*
	       * getData() will connect to the server and wait until the user/TA presses the "Start" button
	       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
	       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
	       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
	       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
	       * but receives corrupted data or a message from the server saying something went wrong. For
	       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
	       * will receive a message saying an invalid team number was specified and getData() will throw
	       * an exception letting you know.
	       */
	      Map data = conn.getData();

	      int greenTeam = ((Long) data.get("GreenTeam")).intValue();
	      if (greenTeam == TEAM_NUMBER) {
	    	  	green = true;
	    	  	corner = ((Long) data.get("GreenCorner")).intValue();
			float[] startHome = {((Long) data.get("Green_LL_x")).intValue(), ((Long) data.get("Green_LL_y")).intValue()};
			float[] endHome = {((Long) data.get("Green_UR_x")).intValue(), ((Long) data.get("Green_UR_y")).intValue()};
			home[0] = startHome;
			home[1] = endHome;
			float[] startIsland = {((Long) data.get("Island_LL_x")).intValue(), ((Long) data.get("Island_LL_y")).intValue()};
			float[] endIsland = {((Long) data.get("Island_UR_x")).intValue(), ((Long) data.get("Island_UR_y")).intValue()};
			island[0] = startIsland;
			island[1] = endIsland;
			float[] tunnel_LL = {((Long) data.get("TNG_LL_x")).intValue(), ((Long) data.get("TNG_LL_y")).intValue()};
			float[] tunnel_UR = {((Long) data.get("TNG_UR_x")).intValue(), ((Long) data.get("TNG_UR_y")).intValue()};
			tunnel[0] = tunnel_LL;
			tunnel[1] = tunnel_UR;
			tree[0] = ((Long) data.get("TG_x")).intValue(); 
			tree[1] = ((Long) data.get("TG_y")).intValue();
	    	  	
	      }


	    } catch (Exception e) {
	      System.err.println("Error: " + e.getMessage());
	    }
	    
	}
}
