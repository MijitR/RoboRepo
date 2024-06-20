package DM.Micro;
import java.awt.geom.*;
import robocode.util.Utils;
/**
 * MyClass - a class by (your name here)
 */
public class WaveBullet
{
	private double startx, starty, startBearing, power;
	private long fireTime;
	private int direction;
	private int[] returnSegment;
	
	public WaveBullet(double x, double y, double bearing, double power, int direction, long time, int[] segment)
	{
		startx = x;
		starty = y;
		startBearing = bearing;
		this.power = power;
		this.direction = direction;
		fireTime = time;
		returnSegment = segment;
	}
	
	public double getBulletSpeed()
	{
		return 20-power*3;
	}
	
	public double maxEscapeAngle()
	{
		return Math.asin(8/getBulletSpeed());
	}
	
	public boolean checkHit(double enemyX, double enemyY, long currentTime)
	{
		//if the distance from the wave origin to our enemy has passed the distance the bullet would have traveled...
		if (Point2D.distance(startx, starty, enemyX, enemY) <= (currentTime-firetime)*getBulletSpeed())
		{
			double desiredDirection = Math.atan2(enemyX-startx, enemyY-starty);
			double angleOffset = Utils.normalRelativeAngle(desiredDirection-startBearing);
			double guessFactor = Math.max(-1, Math.min(1, angleOffset/maxEscapeAngle()))*direction;
			int index = (int)Math.round((returnSegment.length-1)/2*(guessFactor+1));
			returnSegment[index]++;
			return true;
		}
		return false;
	}
}
