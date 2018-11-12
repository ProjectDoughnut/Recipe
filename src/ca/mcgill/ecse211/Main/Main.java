package ca.mcgill.ecse211.Main;

import ca.mcgill.ecse211.Ultrasonic.*;

import ca.mcgill.ecse211.Ultrasonic.USLocalizer.LocalizationType;
import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Color.ColorClassifier.RingColors;
import ca.mcgill.ecse211.Gyro.AngleSampler;
import ca.mcgill.ecse211.Color.ColorPoller;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Light.LightPoller;
import ca.mcgill.ecse211.Light.LightCorrector;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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


public class Main {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final Port usPort = LocalEV3.get().getPort("S3");
	private static final Port gyroPort = LocalEV3.get().getPort("S4");
	private static final Port lsPort = LocalEV3.get().getPort("S1");
	private static final Port csPort = LocalEV3.get().getPort("S2");

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
	public static final double WHEEL_BASE = 12.70;
	public static final double TILE_SIZE = 30.48;
	
	
	private static boolean red = false;
	private static boolean green = false;
	private static int corner;
	private static int[] cornerCoord;
	private static float[][] island = new float[2][2];
	private static float[][] home = new float[2][2];
	private static float[][] tunnel = new float[2][2];
	private static float[] tree = new float[2];
	public static float[][] pathToTree;
	private static RingColors targetRing;
	private static final String SERVER_IP = "192.168.2.25";
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
		//AngleSampler gyro = new AngleSampler(gyroValue);
		
		
		// define light corrector
		LightCorrector LSCorrector = new LightCorrector();
		LightPoller lsCorrectorPoller = new LightPoller(lsValue, LSCorrector);
		
		ColorClassifier CSLocal = new ColorClassifier(odo, nav, targetRing, false);

		//ColorPoller csPoller = new ColorPoller(csValue, CSLocal);
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
		      lcd.drawString("Local- | and    ", 0, 2);
		      lcd.drawString(" ize   | Nav-   ", 0, 3);
		      lcd.drawString("       | igate  ", 0, 4);

		    buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
//			USLocal.setType(LocalizationType.FALLING_EDGE);
		    Thread odoThread = new Thread(odo);
		    odoThread.start();
			Thread displayThread = new Thread(display);
			displayThread.start();

			if (buttonChoice == Button.ID_LEFT) {
				USLocal.setType(LocalizationType.FALLING_EDGE);
				Thread usPollerThread = new Thread(usPoller);
				usPollerThread.start();
				Thread lsPollerThread = new Thread(lsPoller);
				lsPollerThread.start();
			}
			else if (buttonChoice == Button.ID_RIGHT) {
				USLocal.setType(LocalizationType.FALLING_EDGE);
				Thread usPollerThread = new Thread(usPoller);
				usPollerThread.start();
				Thread lsPollerThread = new Thread(lsPoller);
				lsPollerThread.start();
				//Navigate using wifi class and only along x, y lines

				if (corner == 1) {
					odo.setXYT(7*TILE_SIZE, 1*TILE_SIZE, 270);
				}
				if (corner == 2) {
					odo.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 90);
				}
				if (corner == 3) {
					odo.setXYT(1*TILE_SIZE, 7*TILE_SIZE, 180);
				}
				Sound.beep();
				//pathToTree = Navigation.pathing(cornerCoord, home, island, tunnel, tree);
				
				nav._turnTo(odo.getXYT()[2], 0);
				nav._travelTo(2, 2);
				Sound.beep();
//				nav.moveThroughTunnel(tunnel, tree);
//				nav.moveToTree(tunnel, tree);
				
				try {
					usPollerThread.join();
					lsPollerThread.join();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				// add coordinates of tunnel and tree here
				float[][] paths = Navigation.pathing(new float[][] {{1,2},{2,3}}, new float[] {4,5});
				for (float[] path: paths) {
					nav.travelTo(path[0], path[1]);
				}
				
				Thread navThread = new Thread(nav);
				navThread.start();
			}
			
		} else { 


			
			
//			USLocal.setType(LocalizationType.RISING_EDGE);
//			Thread usPollerThread = new Thread(usPoller);
//			usPollerThread.start();
//			Thread lsPollerThread = new Thread(lsPoller);
//			lsPollerThread.start();
//			try {
//				usPollerThread.join();
//				lsPollerThread.join();
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
			
			
			// initiate *other stuffs*


			
			
			//Perform full demo
			
			
			
			
//			
//			// clear the display
//			lcd.clear();
//			USLocal.setType(LocalizationType.RISING_EDGE);
//			
//			// Start odometer and display threads
//			Thread odoThread = new Thread(odo);
//			odoThread.start();
//			Thread displayThread = new Thread(display);
//			displayThread.start();
//			Thread usPollerThread = new Thread(usPoller);
//			usPollerThread.start();
//			Thread lsPollerThread = new Thread(lsPoller);
//			lsPollerThread.start();
//			
//			try {
//				usPollerThread.join();
//				lsPollerThread.join();
//				usSensor.close();
//			} catch (InterruptedException e) {
//			
//				e.printStackTrace();
//			}
//			
//			// set position according to startOption
//			switch(startOption) {
//			case 1:
//				odo.setXYT(7*TILE_SIZE, 1*TILE_SIZE, 270);
//				break;
//			case 3:
//				odo.setXYT(1*TILE_SIZE, 7*TILE_SIZE, 90);
//				break;
//			case 2:
//				odo.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 180);
//				break;
//			}
//			
//			
//			// start gyro corrector thread
//			nav.turnTo(odo.getXYT()[2], 0);
//			gyroSensor.reset();
//			nav.setGyro(gyro);
//			
//			
//			// start light corrector thread
//			Thread lsCorrectorThread = new Thread(lsCorrectorPoller);
//			lsCorrectorThread.start();
//			
//			nav.setCorrector(LSCorrector);
//			
//			// travels to the left corner
//			xyt = odo.getXYT();			
//			nav.travelTo(startCorner[0], startCorner[1]);
//			Sound.beep();
//			Thread navThread  = new Thread(nav);
//			navThread.start();
//			
//			try {
//				navThread.join();
//			} catch (InterruptedException e) {
//				
//				e.printStackTrace();
//			}
//			
//			Sound.beep();
//						
//			// find ring part
//			// Makes the robot go in a square spiral 
//			//initSpiral(nav, startCorner, endCorner);
//			
//			Thread csPollerThread = new Thread(csPoller);
//			csPollerThread.start();
//			navThread = new Thread(nav);
//			navThread.start();
//		
//			try {
//				navThread.join();
//				CSLocal.running = false;
//				csSensor.close();
//
//			} catch (InterruptedException e) {
//				
//				e.printStackTrace();
//			}
//			
//			// Ring found
//			// navigates to the end corner
//			nav.setRunning(true);
//			xyt =  odo.getXYT();
//			if (xyt[2] >= 46 && xyt[2] <= 135) {
//				nav.syncTravelTo(((int)xyt[0]/TILE_SIZE+0.5), xyt[1]/TILE_SIZE );	
//			} else if (xyt[2] >= 136 && xyt[2] <= 225) {
//				nav.syncTravelTo(xyt[0]/TILE_SIZE, ((int)xyt[1]/TILE_SIZE-0.5));	
//			} else if (xyt[2] >= 226 && xyt[2] <= 315) {
//				nav.syncTravelTo(((int)xyt[0]/TILE_SIZE-0.5), xyt[1]/TILE_SIZE );	
//			} else {
//				nav.syncTravelTo(xyt[0]/TILE_SIZE, ((int)xyt[1]/TILE_SIZE+0.5));
//			}
//			
//			nav.syncTravelTo(endCorner[0]+0.5, odo.getXYT()[1]/TILE_SIZE);	
//			nav.syncTravelTo(endCorner[0]+0.5, endCorner[1]);	
//			nav.syncTravelTo(endCorner[0],  endCorner[1]);
//			Sound.beep();
//		}
	}
	
	/**
	 * Makes the robot go in a square spiral 
	 * 
	 * @param navigation, startCorner, endCorner
	 */
	
//	public static void initSpiral(Navigation nav, double[] Ll, double[] Rr) {
//		double x, X, y, Y;
//		
//		x = Ll[0] - 0.5;
//		X = Rr[0] + 0.5;
//		y = Ll[1] + 0.5;
//		Y = Rr[1] + 0.5;
//		
//		nav.travelTo(x, y);
//		while(X>x && Y>y) {
//			nav.travelTo(x, Y);
//			x++;
//			nav.travelTo(X, Y);
//			Y--;
//			nav.travelTo(X, y);
//			X--;
//			nav.travelTo(x, y);
//			y++;	
//		}
//	}
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
