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
	
	EV3ColorSensor colourSensor;
	SampleProvider colourSensorProvider;
	float[] colourSample;
	
	EV3UltrasonicSensor ultrasonicSensor;
	SampleProvider ultrasonicSensorProvider;
	float[] distanceSample; 

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
		createSensor();
		runObstacleCheck();
		runTrack();
	}
	
	private void createSensor() {
		colourSensor = new EV3ColorSensor(colourSensorPort);
		colourSensorProvider = colourSensor.getRedMode();
		colourSample = new float[colourSensorProvider.sampleSize()];
		
		ultrasonicSensor = new EV3UltrasonicSensor(ultrasonicSensorPort);
		ultrasonicSensorProvider = ultrasonicSensor.getDistanceMode();
		distanceSample = new float[colourSensorProvider.sampleSize()];
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
					float distanceReading = getDistanceReading();
					if (distanceReading < 0.15){
						obstacleInFront = true;
						manouverObstacle(distanceReading);
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
		
		while(true){
			float colourReading = getColourReading();
			if (colourReading > 0.5){
				currentReading = getDistanceReading();
				currentError = currentReading - distanceFromObstacle;
				derivative = currentError - lastError;
				obstacleCorrection = (propConstant * currentError) + (diffConstant * derivative);
				
				leftMotor.setSpeed((int) (motorPower - obstacleCorrection));
				rightMotor.setSpeed((int) (motorPower + obstacleCorrection));
				
				lastError = currentError;
			} else {
				obstacleInFront = false;
				headMotor.rotate(-94);
				break;
			}
		}
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
						currentReading = getColourReading();
						currentError = currentReading - bwMidPoint;
						derivative = currentError - lastError;
						trackCorrection = (propConstant * currentError) + (diffConstant * derivative);
						
						ev3Screen.drawString("Reading: " + currentReading, 0, 2);
						ev3Screen.drawString("Error: " + currentError, 0, 1);
						ev3Screen.drawString("Correction: " + trackCorrection, 0, 3);
						
						leftMotor.setSpeed((int) (motorPower - trackCorrection));
						rightMotor.setSpeed((int) (motorPower + trackCorrection));
						
						lastError = currentError;
					}
				}
			}			
		}).start();
	}
}
