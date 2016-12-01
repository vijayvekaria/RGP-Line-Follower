package driver;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Driver {
	TextLCD ev3Screen = LocalEV3.get().getTextLCD();
	
	RegulatedMotor headMotor = new EV3MediumRegulatedMotor(MotorPort.A);
	RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
	RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
	
	Port ultrasonicSensorPort = LocalEV3.get().getPort("S2");
	Port colourSensorPort = LocalEV3.get().getPort("S3");
	
	EV3ColorSensor colourSensor = new EV3ColorSensor(colourSensorPort);;
	SampleProvider colourSensorProvider = colourSensor.getRedMode();;
	float[] colourSample = new float[colourSensorProvider.sampleSize()];;
	
	EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicSensorPort);;
	SampleProvider ultrasonicSensorProvider = ultrasonicSensor.getDistanceMode();;
	float[] distanceSample = new float[ultrasonicSensorProvider.sampleSize()];; 

	boolean obstacleInFront = false;
	
	double bwMidPoint = 0.5;
	double motorPower = 50;
	double propConstant = 100;
	double diffConstant = 6000;
	
	
	public static void main(String[] args) {
		Driver testDriver = new Driver();
	}

	public Driver() {
		ev3Screen.drawString("START" , 0, 0);
		runObstacleCheck();
		runTrack();
	}
	
	private float getColourReading(){
		colourSensorProvider.fetchSample(colourSample, 0);
		return colourSample[0];
	}
	
	private float getDistanceReading(){
		ultrasonicSensorProvider.fetchSample(distanceSample, 0);
		return distanceSample[0];
	}
	
	private void runObstacleCheck() {
		new Thread(new Runnable() {
			public void run() {
				while(true){
					if (!obstacleInFront) {
						ev3Screen.drawString("Running ObsCheck" , 0, 1);
						float distanceReading = getDistanceReading();
						if (distanceReading < 0.15){
							obstacleInFront = true;
							manouverObstacle(distanceReading);
						}
					} else {
						ev3Screen.drawString("Stopped ObsCheck" , 0, 1);
					}
				}
			}
		}).start();
	}

	private void runTrack() {
		new Thread(new Runnable() {		
			public void run() {
				double currentReading = 0;
				double currentError = 0;
				double trackCorrection = 0;
				double derivative = 0;
				double lastError = 0;
				
				leftMotor.forward();
				rightMotor.forward();

				while(true){
					if(!obstacleInFront){
						ev3Screen.drawString("Running Track" , 0, 2);
						currentReading = getColourReading();
						currentError = currentReading - bwMidPoint;
						derivative = currentError - lastError;
						trackCorrection = (propConstant * currentError) + (diffConstant * derivative);
						
//						ev3Screen.drawString("Reading: " + currentReading, 0, 2);
//						ev3Screen.drawString("Error: " + currentError, 0, 1);
//						ev3Screen.drawString("Correction: " + trackCorrection, 0, 3);
						
						leftMotor.setSpeed((int) (motorPower - trackCorrection));
						rightMotor.setSpeed((int) (motorPower + trackCorrection));
						
						lastError = currentError;
					} else {
						ev3Screen.drawString("Stopped Track" , 0, 2);
					}
				}
			}			
		}).start();
	}

	private void manouverObstacle(float distanceFromObstacle) {
		
		leftMotor.stop();
		rightMotor.stop();
		
		headMotor.rotate(90);
		rightMotor.rotate(180);				
		
		double currentReading = 0;
		double currentError = 0;
		double obstacleCorrection = 0;
		double derivative = 0;
		double lastError = 0;
		
		leftMotor.forward();
		rightMotor.forward();
		
		float colourReading = getColourReading();
		
		ev3Screen.drawString("Manouvering" , 0, 3);
		
		while(colourReading > 0.5){
			currentReading = getDistanceReading();
			currentError = currentReading - distanceFromObstacle;
			derivative = currentError - lastError;
			obstacleCorrection = (propConstant * currentError) + (diffConstant * derivative);
			
			leftMotor.setSpeed((int) (motorPower - obstacleCorrection));
			rightMotor.setSpeed((int) (motorPower + obstacleCorrection));
			
			lastError = currentError;

			colourReading = getColourReading();
		}
		
		ev3Screen.drawString("Manouv. Comp!" , 0, 3);
		
		headMotor.rotate(-94);
		obstacleInFront = false;	
	}
}
