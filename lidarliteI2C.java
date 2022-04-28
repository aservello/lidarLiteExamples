package org.ilite.frc.common.sensors;

import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class LidarLite
{
  
	private I2C lidari2c;
	private boolean hasSignal;
	private final Timer t = new Timer();
	private TimerTask lidarReadTask = null;
	private int mCount = 0;
	private byte[] distance;	
		
	
	private final static int LIDAR_ADDR = 0x62;
	private final static int LIDAR_CONFIG_REGISTER = 0x00;
	private final static int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	public LidarLite()
	{
		lidari2c = new I2C(Port.kOnboard, LIDAR_ADDR);
		hasSignal = false;
		distance = new byte[2];
	}
	
	public byte[] getDistanceRegister() {
		return Arrays.copyOf(distance, 2);
	}
	
	
	public double getDistance()
	{
		int distCm = (int) Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);
		return distCm / 100.0;
	}
	
	public boolean checkSignal()
	{
		return hasSignal;
	}
	
	public void start() {
		lidarReadTask = new TimerTask() {
			@Override
			public void run() {
				update();
			}
		};
		t.scheduleAtFixedRate(lidarReadTask, 1000, 1000);
	}
	
	public void stop() {
		lidarReadTask.cancel();
	}
	
	public void update()
	{
		if(lidari2c.write(LIDAR_CONFIG_REGISTER, 0x04b))
		{
			hasSignal = false;
			return;
		}
		edu.wpi.first.wpilibj.Timer.delay(0.05);
		if(!lidari2c.read(LIDAR_DISTANCE_REGISTER, 2, distance))
		{
			return;
		} else {
			hasSignal = true;
		}
		edu.wpi.first.wpilibj.Timer.delay(0.05);
	}
}
