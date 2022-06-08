/*
	LidarLite I2C Connection in Java
	useful info:
	
	https://docs.wpilib.org/en/stable/docs/hardware/sensors/serial-buses.html
	https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html#onboard-i2c-causing-system-lockups
	
	WIPLIB Refrence Used
	https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/package-summary.html
*/


package x.y.z;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;

public class LidarLite
{ 
    private I2C i2c;
    private byte[] distance;
    private java.util.Timer updater;
    private final int LIDAR_ADDR = 0x62;
    private final int LIDAR_CONFIG_REGISTER = 0x00;
    private final int LIDAR_DISTANCE_REGISTER = 0x8f;


    public pulsedLightLIDAR() {
        i2c = new I2C(Port.kMXP, LIDAR_ADDR);
        distance = new byte[2];
        updater = new java.util.Timer();
    }

    public double getDistance() {
        int distCM = (int) Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);
	System.out.println(distCM / 100)
	// return distCM / 100; for testing it will just print the distance
    }

    public void start() {
        updater.scheduleAtFixedRate(new LIDARUpdater(), 0, 100);
    }

    public void stop() {
        updater.cancel();
        updater = new java.util.Timer();
    }

    private void update() {
        i2c.write(LIDAR_CONFIG_REGISTER, 0x04); 
        Timer.delay(0.04);
        i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance);
        Timer.delay(0.005);
    }

    private class LIDARUpdater extends TimerTask {
        public void run() {
            while (true) {
                update();
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
