package org.usfirst.frc.team687.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class NerdyAutonomous {
	private int mode = -9001;
	Timer t;
	public NerdyAutonomous(int m)	{
		mode = m;
		t = new Timer();
	}
	boolean first;
	double[] output=new double[4];
	public double[] run()	{
		for(int i = 0; i < 4; i++)	{
			output[i]=0;
		}
		if(RobotState.isAutonomous())	{
			if(mode==0)	{
				if(first){
					t.start();
					first = false;
				}
				if(t.get() < 2)	{
					for(int i = 0; i<4; i++)	{
						output[i]=0.5;
					}
				}	else	{
					for(int i = 0; i<4; i++)	{
						output[i] = 0;
					}
				}
			}
		}
		
		return output;
		
	}
	
}
