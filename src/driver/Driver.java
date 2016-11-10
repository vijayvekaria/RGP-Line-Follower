package driver;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;


public class Driver {
	
	//Create sensor ports
	Port colourSensorPort = LocalEV3.get().getPort("S3");
	Port ultrasonicSensorPort = LocalEV3.get().getPort("S2");
	
	//Create color sensor and sample objects
	EV3ColorSensor colourSensor;
	SampleProvider colourSensorProvider;
	float[] bwSample;
	
	//Create ultrasonic sensor and sample objects
	EV3UltrasonicSensor ultrasonicSensor;
	SampleProvider ultrasonicSensorProvider;
	float[] distanceSample; 
	
	//Create brick pilot
	DifferentialPilot ev3Pilot;
	
	boolean curtainInFront = true;
	
	//Initialise SENSORS
	private void createSensor() {
		//Colour sensor initialisation
		colourSensor = new EV3ColorSensor(colourSensorPort);
		colourSensorProvider = colourSensor.getRGBMode();
		bwSample = new float[colourSensorProvider.sampleSize()];
		
		//Ultrasonic sensor initialisation
		ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicSensorPort);
		ultrasonicSensorProvider = ultrasonicSensor.getDistanceMode();
		distanceSample = new float[colourSensorProvider.sampleSize()];
	}
	
	//Initialise EV3 Pilot
	private void createPilot() {
		ev3Pilot = new DifferentialPilot(5.5, 12, Motor.B, Motor.C);
	}
	
	//Calculates average RGB value from colour sensor sample
	private double calculateColorReadingAverage(){
		colourSensorProvider.fetchSample(bwSample, 0);
		double avgSample = (bwSample[0] + bwSample[1] + bwSample[1]) / 3;
		return avgSample;
	}
	
	//Creates and runs a new thread to check if a curtain is in front of the brick
	private void runCurtainCheck() {
		new Thread(new Runnable() {
			@Override
			public void run() {
				while(true){
					ultrasonicSensorProvider.fetchSample(distanceSample, 0);
					if (distanceSample[0] > 0.1){
						curtainInFront = false;
					} else {
						curtainInFront = true;
					}
				}
			}
		}).start();
	}
	
	//Runs the track 
	private void runTrack() {
		double bValue = 10;
		double wValue = 90;
		double bwMidPoint = (bValue + wValue) / 2;
		
		double propConstant = 10;
		double intgConstant = 1;
		double diffConstant = 100;
		

		new Thread(new Runnable() {		
			@Override
			/*		Tuning Constants
			 * 				: http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
			 */
			public void run() {
				double currentReading = 0;
				double currentError = 0;
				double integral = 0;
				double derivative = 0;
				double lastError = 0;
				while(!curtainInFront){	
					currentReading = calculateColorReadingAverage();
					currentError = currentReading - bwMidPoint;
					integral = currentError + integral;
					derivative = currentError - lastError;
					double correction = (propConstant * currentError) + (intgConstant * integral) + (diffConstant * derivative);
					ev3Pilot.steer(correction);
					lastError = currentError;
				}
			}			
		}).start();
	}
	
	//Constructor that call initialisation methods and run methods
	public Driver() {
		createSensor();
		createPilot();
		runCurtainCheck();
		runTrack();
	}
	
	public static void main(String[] args) {
		Driver testDriver = new Driver();
	}
}
