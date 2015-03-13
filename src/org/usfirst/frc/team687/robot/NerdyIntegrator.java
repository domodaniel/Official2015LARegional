package org.usfirst.frc.team687.robot;

public class NerdyIntegrator {
	private double error = 0, lastError = 0;
	private long time = 0, lastTime = -9001;
	private double integration = 0;
	
	
	public NerdyIntegrator()	{}
	
	public double perform(double e, long t)	{
		lastError = error;
		error = e;
		if(lastTime < 0)	{
			lastTime = t;
		}	else	{
			lastTime = time;
		}
		time = t;
		
		integration += ((error + lastError)/2)*(time-lastTime)/1000;
		return integration;
	}
	
	public void reset()	{
		integration = 0;
	}
}
