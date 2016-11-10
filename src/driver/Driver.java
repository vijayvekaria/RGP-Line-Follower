package driver;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;


public class Driver {
	
	Port colorSensor = LocalEV3.get().getPort("S3");
	EV3ColorSensor bwColorSensor;
	SampleProvider sensorProvidor;
	float[] bwSample;
	
	DifferentialPilot ev3Pilot;
	
	boolean soundActivated = false;
	
	private void createSensor() {
		bwColorSensor = new EV3ColorSensor(colorSensor);
		sensorProvidor = bwColorSensor.getRGBMode();
		bwSample = new float[sensorProvidor.sampleSize()];		
	}
	
	private void createPilot() {
		ev3Pilot = new DifferentialPilot(5.5, 12, Motor.B, Motor.C);
	}
	
	private double calculateAverage(){
		double avgSample = (bwSample[0] + bwSample[1] + bwSample[1]) / 3;
		return avgSample;
	}
	
	private void runTrack() {
		
		double bValue = 10;
		double wValue = 90;
		double bwMidPoint = (bValue + wValue) / 2;
		
		double propConstant = 180;
		double diffConstant = 6.3;
		double intgConstant = 1286;

		new Thread(new Runnable() {		
			@Override
			public void run() {
				double integral = 0;
				double lastError = 0;
				while(true){	//Needs to change to soundActivated when sound module is added
					double currentReading = calculateAverage();
					double currentError = bwMidPoint - currentReading;
					integral = currentError + integral;
					double derivative = currentError - lastError;
					double correction = (propConstant * currentError) + (intgConstant * integral) + (diffConstant * derivative);
					ev3Pilot.steer(correction);
					lastError = currentError;
				}
			}
			
			/*		Tuning Constants
			 * 				: http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
			 */
		});
	}
	
	private void runSound() {
		new Thread(new Runnable() {
			@Override
			public void run() {
				while(true){
					//insert sound check here;
				}
			}
		});
	}
	
	public Driver() {
		createSensor();
		createPilot();
		//runSound();
		runTrack();
	}
	
	public static void main(String[] args) {
		Driver testDriver = new Driver();
	}
}
