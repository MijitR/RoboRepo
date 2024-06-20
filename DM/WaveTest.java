package DM;
import robocode.*;
import java.awt.Color;
import java.awt.geom.*;

/**
 * WaveTest - a robot by (your name here)
 */
public class WaveTest extends AdvancedRobot
{
	Enemy enemy = new Enemy();
	final static double PI = Math.PI;
	static double direction = 1;
	static double startingX;
	static double startingY;
	static double xDif;
	static double yDif;
	static double PossibleX;
	static double PossibleY;
	static double pX = 1;
	static double pY = 1;
	/**
	 * run: WaveTest's default behavior
	 */
	public void run() {
		setColors(Color.blue,Color.green,Color.red);
		turnRadarRight(Double.POSITIVE_INFINITY);
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		enemy.health = 100;
		enemy.lastHealth = enemy.health;
		enemy.ctime = 0;
		execute();
		while(true) {
			// Replace the next 4 lines with any behavior you would like
			//setTurnRadarRight(360);
			out.println(pX + ", " + pY);
			//setTurnRadarRight(360);
			execute();
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		//if(enemy.name == null)
		doMovement();
		//doScanner();
		enemy.absBearing = (getHeadingRadians() + e.getBearingRadians()) * (PossibleX * (5 - pX * 5)) * (PossibleY * (5 - pY * 5));
		//enemy.absBearing = getHeadingRadians() + e.getBearingRadians();
		setTurnGunRightRadians(getHeadingRadians() + e.getBearingRadians() - getGunHeadingRadians());
		setTurnRadarLeft(getRadarTurnRemaining()*Math.random());
		if(getGunHeat()==0)
		setFire(2.5);
		enemy.ctime = getTime();
		enemy.health = e.getEnergy();
		enemy.x = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
		enemy.y = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
		execute();
	}
	
	public void doMovement(){
		//this is the absolute heading I want to move in to go clockwise or counterclockwise around my enemy
		//if I want to move closer to them, I would use less of an offset from absBearing (I'll go right toward them if I move at absBearing)
		double goalDirection = enemy.absBearing-Math.PI/2*direction;
		Rectangle2D fieldRect = new Rectangle2D.Double(18, 18, getBattleFieldWidth()-36, getBattleFieldHeight()-36);
		while (!fieldRect.contains(getX()+Math.sin(goalDirection)*120, getY()+Math.cos(goalDirection)*120))
		{
			goalDirection += direction*.1;	//turn a little toward my enemy and try again
			execute();
		}
		double turn = robocode.util.Utils.normalRelativeAngle(goalDirection-getHeadingRadians());
		if (Math.abs(turn) > Math.PI/2)
		{
			turn = robocode.util.Utils.normalRelativeAngle(turn + Math.PI);
			setBack(100);
		}
		else
			setAhead(100);
		setTurnRightRadians(turn);
		
		if(enemy.health!=enemy.lastHealth){
		enemy.shotPower = (enemy.health - enemy.lastHealth) * -1;
		startingX = getX();
		startingY = getY();
		PossibleX = (8 * (getRange(startingX, startingY, enemy.x, enemy.y) / (20 - 3 * enemy.shotPower)));
		PossibleY = (8 * (getRange(startingX, startingY, enemy.x, enemy.y) / (20 - 3 * enemy.shotPower)));
		enemy.lastHealth = enemy.health;
		}
		execute();
	}
	
	public void doScanner()
		{
			double radarOffset;
			if(getTime() - enemy.ctime > 2){
				radarOffset = 2*PI;
			}
			else{
			radarOffset = getRadarHeadingRadians() - (Math.PI/2 - Math.atan2(enemy.y - getY(),enemy.x - getX())); 

			radarOffset = NormaliseBearing(radarOffset);
			if (radarOffset < 0)
				radarOffset -= PI/10;
			else {
				radarOffset += PI/10; 
		}
	}
		setTurnRadarLeftRadians(radarOffset);
	}			

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		xDif = getX() - startingX;
		yDif = getY() - startingY;
		pX = xDif/PossibleX;
		pY = yDif/PossibleY;
	}
	
	public static double getRange(double x2, double y2, double x, double y)
	{
		double xo = x2 - x;
		double yo = y2 - y;
		double h = Math.sqrt( xo*xo + yo*yo );
		return h;	
	}
	
	public double normalRelativeAngle(double angle) {
		if (angle > -180 && angle <= 180)
			return angle;
		double fixedAngle = angle;
		while (fixedAngle <= -180)
			fixedAngle += 360;
		while (fixedAngle > 180)
			fixedAngle -= 360;
		return fixedAngle;
	}
	
	double NormaliseBearing(double ang) {
		if (ang > PI)
			ang -= 2*PI;
		if (ang < -PI)
			ang += 2*PI;
		return ang;
	}
	
}
class Enemy {
	public static String name;
	public static double x;
	public static double y;
	public static double ctime;
	public static double absBearing;
	public static double health;
	public static double lastHealth;
	public static double shotPower;
}