package ca.mcgill.ecse211.Main;

import ca.mcgill.ecse211.Ultrasonic.*;


import ca.mcgill.ecse211.Ultrasonic.USLocalizer.LocalizationType;
import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Color.ColorClassifier.RingColors;
import ca.mcgill.ecse211.Color.ColorPoller;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Light.LightPoller;
import ca.mcgill.ecse211.Light.TwoLightPoller;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import ca.mcgill.ecse211.Odometer.OdometryCorrector;
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
import java.util.Arrays;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;


/**
 * 
 * @author Binyuan Sun and Alexandra Livadas
 *
 * The starting point of the robot's program. All motors, sensors and other constants required are defined here.
 * Upon launch of the program the user will be prompted and based on the user's action a set of instructions will
 * be carried out by the robot. 
 * 
 * Description of code flow: 1. Robot takes in parameters from the server 2. Sets up all the 
 * constructors, ultrasonic, light, odometer, navigation, etc 3. Ask for user input 4. Robot 
 * sets USLocal to FALLING_EDGE method 5. UltrasonicPoller thread is atarted as well as the LightPoller
 * thread 6. Odometer and OdometryCorrector threads are started in parallel 7. Robot starts ultrasonic
 * localization in the USLocalizer class where method process() is called, Once ultrasonic localization ends,
 * LightLocalizer class's process() method is called and the robot uses "lsSensor" on "lsPort" value to read 
 * the black lines. The robot sweeps the lines by rotating 360 degrees and trigonometry is used to localize 
 * the robot to the starting point 8. The code then checks which coordinates the robot is starting from. 
 * The odometer reading is set accordingly 9. The pathing method in the Navigation class takes in the starting
 * corner, tunnel and tree coordinates from what the server provided and calculates the path the robot needs
 * to take and returns it in a two-dimentional float array. For further details on how the pathing method 
 * works look at Software Documentation Version x section xyz 10. The navigation thread is started and then 
 * joined with other threads, the robot starts travelling towards the tree 11. The odometryCorrector fixes
 * the heading if the x and y errors are greater than (Bin help). The navigation thread is paused while
 * the heading is corrected. For more details look at OdometryCorrector class. The nav thread resumes
 * once the angle correction is finished 12. All of the coordinates except for the tree's are saved in an 
 * array called originCoordinate 13. The RingCollection class takes in the originCoordinate which makes the robot 
 * stop one tile away from the tree 14. The ColorPoller thread is started and then joined with the other threads. 
 * The OdometryCorrector is paused when the ColorPoller thread is running. It resumes once 4 sides of the trees 
 * are visited.
 * 
 * TO BE CONTINUED
 * 
 * Threads in use - We have a total of x (waiting for everything to be done)
 * threads. 1 color poller thread for the color sensor, 1 light poller thread for the two
 * threads that detect lines, 1 odometer thread, 1 odometry display thread,
 * 1 ultrasonic poller thread and 1 odometry correction thread.
 * We use 'lock's to pause our threads, then unlock it for it to continue running
 * For efficiency we join threads after to. Threads also sleep for a period of time 
 * in some classes for (BIN Help). 
 * 
 * Current number of lines of code: (update at the end), Code estimation: (update
 * at the end) (number of semicolons).
 */

public class Main {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3MediumRegulatedMotor clawServo = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

	private static final Port usPort = LocalEV3.get().getPort("S3");
	private static final Port lsPort = LocalEV3.get().getPort("S2");
	private static final Port lsPort2 = LocalEV3.get().getPort("S4");
	private static final Port csPort = LocalEV3.get().getPort("S1");

	//Setting up ultrasonic sensor
	public static UARTSensor usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usValue = usSensor.getMode("Distance");

	//Setting up light sensor

	public static UARTSensor lsSensor = new EV3ColorSensor(lsPort);
	public static SampleProvider lsValue = lsSensor.getMode("Red");

	public static UARTSensor lsSensor2 = new EV3ColorSensor(lsPort2);
	public static SampleProvider lsValue2 = lsSensor2.getMode("Red");

	public static EV3ColorSensor csSensor = new EV3ColorSensor(csPort);
	public static SampleProvider csValue = csSensor.getRGBMode();

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	public static final double WHEEL_BASE = 12.2;
	public static final double TILE_SIZE = 30.48;
	public static final double sensorDistance = 11.3;

	public static final float[][] gameArea = {{1,1}, {14,8}};

	private static int corner;
	private static int[] cornerXY = new int[2];
	private static float[][] island = new float[2][2];
	private static float[][] home = new float[2][2];
	private static float[][] tunnel = new float[2][2];
	private static float[] tree = new float[2];
	public static float[][] pathToTree;
	private static RingColors targetRing;
	private static final String SERVER_IP = "192.168.43.254";
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
		LightPoller lsPoller = new LightPoller(lsValue2, LSLocal);


		// define light corrector

		ColorClassifier CSLocal = new ColorClassifier(odo, nav, targetRing, false);

		double[] xyt;

		ColorPoller csPoller = new ColorPoller(csValue, CSLocal);


		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float

			lcd.drawString("Press enter to start", 0, 3);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		} while (buttonChoice != Button.ID_ENTER);

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

			Sound.beep();
			Sound.beep();
			Sound.beep();

			if (corner == 1) {
				odo.setXYT(14*TILE_SIZE, 1*TILE_SIZE, 270);
				cornerXY[0] = 14;
				cornerXY[1] = 1;
			}
			else if (corner == 2) {
				odo.setXYT(14*TILE_SIZE, 8*TILE_SIZE, 180);
				cornerXY[0] = 14;
				cornerXY[1] = 8;
			}
			else if (corner == 3) {
				odo.setXYT(1*TILE_SIZE, 8*TILE_SIZE, 90);
				cornerXY[0] = 1;
				cornerXY[1] = 8;
			}
			else {
				cornerXY[0] = 1;
				cornerXY[1] = 1;
			}


			// start the odo correction thread
			OdometryCorrector odoCorrector = new OdometryCorrector(nav);
			TwoLightPoller odoCorrectorPoller = new TwoLightPoller(lsValue, lsValue2, odoCorrector);
			Thread odoCorrectorThread = new Thread(odoCorrectorPoller);
			odoCorrectorThread.start();

			// add coordinates of tunnel and tree here
			float[][] paths = Navigation.pathing(cornerXY, tunnel, tree);
			for (float[] path: paths) {
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
			// beeps 3 times

			Sound.beep();
			Sound.beep();
			Sound.beep();

			float[] originCoordinate = paths[paths.length -1];

			// start ring collection thread



			RingCollection ringCollector = new RingCollection(odo, nav, clawServo, originCoordinate, tunnel, tree, island, WHEEL_RAD);

			Thread ringCollectThread = new Thread(ringCollector);

			ringCollectThread.start();
			Thread colorThread = new Thread(csPoller);
			colorThread.start();


			// wait for color thread to join (ided one ring)
			try {
				//colorThread.join();
				ringCollectThread.join();
				CSLocal.running = false;
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			Sound.beep();

			for (int i = paths.length-1; i>=0; i--) {
				nav.travelTo(paths[i][0], paths[i][1]);
			}

			// unneeded
			nav.travelTo(cornerXY[0], cornerXY[1]);  

			Thread navBackThread = new Thread(nav);
			navBackThread.start();

			try {
				navBackThread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();

			//			Sound.playNote(Sound.XYLOPHONE, 500, 500);



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
			int redTeam = ((Long) data.get("RedTeam")).intValue();

			if (greenTeam != TEAM_NUMBER && redTeam != TEAM_NUMBER) return;
			
			float[] startHome = new float[2];
			float[] endHome = new float[2];
			float[] tunnel_LL = new float[2];
			float[] tunnel_UR = new float[2];

			if (greenTeam == TEAM_NUMBER) {
				corner = ((Long) data.get("GreenCorner")).intValue();
				startHome = new float[] {((Long) data.get("Green_LL_x")).intValue(), ((Long) data.get("Green_LL_y")).intValue()};
				endHome = new float[]{((Long) data.get("Green_UR_x")).intValue(), ((Long) data.get("Green_UR_y")).intValue()};
				home[0] = startHome;
				home[1] = endHome;

				tunnel_LL = new float[] {((Long) data.get("TNG_LL_x")).intValue(), ((Long) data.get("TNG_LL_y")).intValue()};
				tunnel_UR = new float[] {((Long) data.get("TNG_UR_x")).intValue(), ((Long) data.get("TNG_UR_y")).intValue()};
				tree[0] = ((Long) data.get("TG_x")).intValue(); 
				tree[1] = ((Long) data.get("TG_y")).intValue();

			} else if (redTeam == TEAM_NUMBER) {
				corner = ((Long) data.get("RedCorner")).intValue();
				startHome = new float[] {((Long) data.get("Red_LL_x")).intValue(), ((Long) data.get("Red_LL_y")).intValue()};
				endHome = new float[]{((Long) data.get("Red_UR_x")).intValue(), ((Long) data.get("Red_UR_y")).intValue()};
				home[0] = startHome;
				home[1] = endHome;

				tunnel_LL = new float[] {((Long) data.get("TNR_LL_x")).intValue(), ((Long) data.get("TNR_LL_y")).intValue()};
				tunnel_UR = new float[] {((Long) data.get("TNR_UR_x")).intValue(), ((Long) data.get("TNR_UR_y")).intValue()};
				tree[0] = ((Long) data.get("TR_x")).intValue(); 
				tree[1] = ((Long) data.get("TR_y")).intValue();
			}

			float[] startIsland = {((Long) data.get("Island_LL_x")).intValue() , ((Long) data.get("Island_LL_y")).intValue()};
			float[] endIsland = {((Long) data.get("Island_UR_x")).intValue(), ((Long) data.get("Island_UR_y")).intValue()};
			if (startIsland[0] > gameArea[1][0]) {
				startIsland[0] = gameArea[1][0];
			} else if (startIsland[0] < gameArea[0][0]) {
				startIsland[0] = gameArea[0][0];
			}

			if (startIsland[1] > gameArea[1][1]) {
				startIsland[1] = gameArea[1][1];
			} else if (startIsland[1] < gameArea[0][1]) {
				startIsland[1] = gameArea[0][1];
			}				

			if (endIsland[0] > gameArea[1][0]) {
				endIsland[0] = gameArea[1][0];
			} else if (endIsland[0] < gameArea[0][0]) {
				endIsland[0] = gameArea[0][0];
			}

			if (endIsland[1] > gameArea[1][1]) {
				endIsland[1] = gameArea[1][1];
			} else if (endIsland[1] < gameArea[0][1]) {
				endIsland[1] = gameArea[0][1];
			}

			island[0] = startIsland;
			island[1] = endIsland;

			if (startIsland[1] - endHome[1] > 0) {
				// basic case
				tunnel[0] = tunnel_LL;
				tunnel[1] = tunnel_UR;
			} else if(startHome[1] - endIsland[1] > 0) {
				tunnel[0] = tunnel_UR;
				tunnel[1] = tunnel_LL;
			} else if (endIsland[0] - startHome[0]  > 0 ) {
				tunnel[0] = new float[] {tunnel_LL[0], tunnel_UR[1]};
				tunnel[1] = new float[] {tunnel_UR[0], tunnel_LL[1]};
			} else if (endHome[0] - startIsland[0]  > 0) {
				tunnel[1] = new float[] {tunnel_LL[0], tunnel_UR[1]};
				tunnel[0] = new float[] {tunnel_UR[0], tunnel_LL[1]};
			}


		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

	}
}
