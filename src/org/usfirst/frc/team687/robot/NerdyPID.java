package org.usfirst.frc.team687.robot;

public class NerdyPID {
	private NerdyIntegrator integrator;
	
	//drive
	private final double driveKP = 0.00444444;
	private final double driveKI = 0.00004444;
	
	//artic
	private final double articKP = 0.0066666666;
	private final double articKI = 0.0000549450;
	private double clearance = 2, boxHeight = 12.1;
	
	//distance
	private final double distanceKP = 0.005;
	private final double distanceKI = 0.00005;
	public double tolerance = 10;
	
	//vars
	private double heading = 0, error = 0;
	private double height;
	private long time;
	
	public void setHeading(double h, long t)	{
		heading = h;
		time = t;
	}
	
	public void setHeight(double h, long t)	{
		height = h;
		time = t;
	}
	
	public NerdyPID()	{
		integrator = new NerdyIntegrator();
	}
	
	//drive functions
	private double getShortestRotation(double desired)	{
		double e = heading - desired;
        if(e > 180)    {
            e = -(Math.abs(360 - e)%180);
        }   else if(e < -180)   {
            e = Math.abs(360 + e)%180;
        }
        
        return e;
	}
	
	public double drive(double desired, long t)	{
		error = getShortestRotation(desired);
        double p = error * driveKP;
        time = t;
        double i =integrator.perform(error, time)*driveKI;
        return p+i;
	}
	
	public double artic(double level, double height, long t)	{
	//artic functions
		double desired = (boxHeight*level)+clearance;
		double error = desired - height;
		time = t;
		double p = error * articKP;
		double i = integrator.perform(error, time)*distanceKI;
		return p + i;
	}
	
	//distance function
	public double distance(double desired, double current, long t)	{
		double error=desired-current;
		time = t;
		double p = error*distanceKP;
		double i = integrator.perform(error, time)*distanceKI;
		return error>tolerance ? p+i : 0;
	}
}
