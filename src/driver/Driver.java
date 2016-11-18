package driver;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;


public class Driver {
	TextLCD TEST = LocalEV3.get().getTextLCD();
	RegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.B);
	RegulatedMotor mC = new EV3LargeRegulatedMotor(MotorPort.C);
	DifferentialPilot ev3Pilot = new DifferentialPilot(5.5, 12, mB, mC);
	
	Port colourSensorPort = LocalEV3.get().getPort("S3");
	EV3ColorSensor colourSensor;
	SampleProvider colourSensorProvider;
	float[] bwSample;
	float correction = 0;
	
	
//	Port ultrasonicSensorPort = LocalEV3.get().getPort("S2");
//	EV3UltrasonicSensor ultrasonicSensor;
//	SampleProvider ultrasonicSensorProvider;
//	float[] distanceSample; 

//	boolean curtainInFront = true;
	
	public static void main(String[] args) {
		Driver testDriver = new Driver();
	}
	
	public Driver() {
		TEST.drawString("START" , 0, 0);
		createSensor();
		//runCurtainCheck();
		runTrack();
	}
	
	private void createSensor() {
		colourSensor = new EV3ColorSensor(colourSensorPort);
		colourSensorProvider = colourSensor.getRedMode();
		bwSample = new float[colourSensorProvider.sampleSize()];
		
		/*ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicSensorPort);
		ultrasonicSensorProvider = ultrasonicSensor.getDistanceMode();
		distanceSample = new float[colourSensorProvider.sampleSize()]; */
	}
	

/*Creates and runs a new thread to check if a curtain is in front of the brick
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
	}*/
	
	//Runs the track 
	private void runTrack() {
		double bValue = 0.1;
		double wValue = 0.9;
		final double bwMidPoint = (bValue + wValue) / 2;
		final double propConstant = 100;
		final double diffConstant = 6000;
		
		new Thread(new Runnable() {		
			public void run() {
				double currentReading = 0;
				double currentError = 0;
				double derivative = 0;
				double lastError = 0;
	
				mB.forward();
				mC.forward();
				
				while(true){
					colourSensorProvider.fetchSample(bwSample, 0);
					currentReading = bwSample[0];

					currentError = currentReading - bwMidPoint; //Calculates the difference between the reading and the expected midPoint
					derivative = currentError - lastError; //Calculates the gain in error

					correction = (float) ((float) 1.5 * ((propConstant * currentError) + (diffConstant * derivative))); //Calculates the correction value
					
					TEST.drawString("Reading: " + currentReading, 0, 1);
					TEST.drawString("Error: " + currentError, 0, 2);
					TEST.drawString("Correction: " + correction, 0, 3);
		
					mB.setSpeed((int) (75 - correction));
					mC.setSpeed((int) (75 + correction));
					
					lastError = currentError;
				}
			}			
		}).start();
	}
}