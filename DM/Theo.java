package DM;
import robocode.*;
import java.awt.Color;
import java.awt.geom.*;
import robocode.util.Utils;

/**
 * Theo - a robot by Damij
 */
public class Theo extends TeamRobot
{
	List waves = new ArrayList();
	static int[] stats = new int[31];	//31 is the number of unique guessfactors we're using
	int direction = 1;
	/**
	 * run: Theo's default behavior
	 */
	public void run() {

		setColors(Color.red,Color.blue,Color.green);
		while(true) {
			setTurnRadarRight(360);
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e)
	{

		double absBearing = getHeadingRadians() + e.getBearingRadians();
		//find our enemy's location:
		double ex = getX() + Math.sin(absBearing)*e.getDistance();
		double ey = getY() + Math.cos(absBearing)*e.getDistance();
		
		//let's process the waves now:
		for (int i=0; i<waves.size(); i++)
		{
			WaveBullet currentWave = (WaveBullet)waves.get(i);
			if (currentWave.checkHit(ex, ey, getTime()))
			{
				waves.remove(currentWave);
				i--;
			}
		}
		
		double power = Math.min(3, Math.max(.1,  2 ));
		//don't try to figure out the direction they're moving if they're not moving, just use the direction we had before
		if (e.getVelocity() != 0)
			if (Math.sin(e.getHeadingRadians()-absBearing)*e.getVelocity() < 0)
				direction = -1;
			else
				direction = 1;
		int[] currentStats = stats;	//This seems silly, but I'm using it to show something else later
		WaveBullet newWave = new WaveBullet(getX(), getY(), absBearing, power, direction, getTime(), currentStats);


		int bestindex = 15;	//initialize it to be in the middle, guessfactor 0.
		for (int i=0; i<31; i++)
			if (currentStats[bestindex] < currentStats[i])
				bestindex = i;
		
		//this should do the opposite of the math in the WaveBullet:
		double guessfactor = (double)(bestindex-(stats.length-1)/2)/((stats.length-1)/2);
		double angleOffset = direction*guessfactor*newWave.maxEscapeAngle();
		setTurnGunRightRadians(robocode.util.Utils.normalRelativeAngle(absBearing-getGunHeadingRadians()+angleOffset));


		if (setFireBullet() != null)
			waves.add(newWave);
	}


	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		turnLeft(90 - e.getBearing());
	}
	
}
class WaveBullet
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