package DM;
import robocode.*;
import java.awt.Color;
import java.awt.geom.*;

/**
 * DM - a robot by (your name here)
 */
public class SPK extends Robot
{
	Enemy target;					//our current enemy
	final double PI = Math.PI;		//just a constant
	double bPower;				//the power of the shot we will be using
	double lastHealth;
	static double direction = 1;
	public void run() {
		target = new Enemy();
		target.distance = 100000;						//initialise the distance so that we can select a target
		setColors(Color.pink,Color.pink,Color.pink);	//sets the colours of the robot
		//the next two lines mean that the turns of the robot, gun and radar are independant
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		target.health = 100;
		turnRadarRightRadians(2*PI);	//turns the radar right around to get a view of the field
		doFirePower();				//select the fire power to use
		ahead(75*direction);
		while(true) {
			doMovement();				//Move the bot
			doScanner();				//Oscillate the scanner over the bot
			doGun();					//move the gun to predict where the enemy will be
			if (getGunHeat() == 0) {
				fire(bPower);
			}
			execute();					//execute all commands
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		double headingPlus = 6;
		if ((e.getDistance() < target.distance)||(target.name == e.getName())) {
			//the next line gets the absolute bearing to the point where the bot is
			double absbearing_rad = (getHeadingRadians()+e.getBearingRadians())%(2*PI);
			//this section sets all the information about our target
			target.name = e.getName();
			double h = NormaliseBearing(e.getHeadingRadians() - target.head);
			h = h/(getTime() - target.ctime);
			target.changehead = h;
			target.x = getX()+Math.sin(absbearing_rad)*e.getDistance(); //works out the x coordinate of where the target is
			target.y = getY()+Math.cos(absbearing_rad)*e.getDistance(); //works out the y coordinate of where the target is
			target.bearing = e.getBearingRadians();
			target.head = e.getHeadingRadians();
			target.ctime = getTime();				//game time at which this scan was produced
			target.speed = e.getVelocity();
			target.distance = e.getDistance();
			target.health = e.getEnergy();
			
		if(target.distance<200){
			if(direction==1)
				headingPlus = 15;
			else{
				headingPlus = -15;
			}
		}

		if(e.getBearing()>0)
		turnRight((e.getBearing() - 90) - headingPlus/* - enemy.escapeAngle*/);
		else if(e.getBearing()<=0)
		turnRight((e.getBearing() + 90) + headingPlus/* - enemy.escapeAngle*/);
		}
	}
		
	void doFirePower() {
		if (bPower > 3)
			bPower = 3;
		bPower = 3;
	}
	
	void doMovement() {
		if ((target.health - lastHealth >= -3 && target.health - lastHealth < 0) || target.distance < 150 || (target.health - lastHealth + (bPower*4) >= -3 && target.health - lastHealth + (bPower*4) < 0))  {
			ahead(75*direction);
		}
		out.println(target.health - lastHealth);
		lastHealth = target.health;
		//if(getVelocity() == 0)
		
	}
	
	void doScanner() {
		double radarOffset;
		if (getTime() - target.ctime > 4) { //if we haven't seen anybody for a bit....
			radarOffset = 4*PI;				//rotate the radar to find a target
		} else {	
			
			//next is the amount we need to rotate the radar by to scan where the target is now
			radarOffset = getRadarHeadingRadians() - (Math.PI/2 - Math.atan2(target.y - getY(),target.x - getX())); 
			//this adds or subtracts small amounts from the bearing for the radar to produce the wobbling
			//and make sure we don't lose the target
			radarOffset = NormaliseBearing(radarOffset);
			if (radarOffset < 0)
				radarOffset -= PI/10;
			else
				radarOffset += PI/10; 
		}
		//turn the radar
		turnRadarLeftRadians(radarOffset);
	}
	
	void doGun() {
		long time;
		long nextTime;
		Point2D.Double p;
		p = new Point2D.Double(target.x, target.y);
		if(target.speed > 0){
		for (int i = 0; i < 10; i++){
       		nextTime = (int)Math.round((getrange(getX(),getY(),p.x,p.y)/(20-(3*bPower))));
			time = getTime() + nextTime;
        	p = target.guessPosition(time);
		}
		}
		else{
			p = new Point2D.Double(target.x, target.y);
		}
		//offsets the gun by the angle to the next shot based on linear targeting provided by the enemy class
		double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(p.y - getY(),p.x -  getX()));
		turnGunLeftRadians(NormaliseBearing(gunOffset));
	}
	
	//if a bearing is not within the -pi to pi range, alters it to provide the shortest angle
	double NormaliseBearing(double ang) {
		if (ang > PI)
			ang -= 2*PI;
		if (ang < -PI)
			ang += 2*PI;
		return ang;
	}
	
	//if a heading is not within the 0 to 2pi range, alters it to provide the shortest angle
	double NormaliseHeading(double ang) {
		if (ang > 2*PI)
			ang -= 2*PI;
		if (ang < 0)
			ang += 2*PI;
		return ang;
	}
	
	//returns the distance between two x,y coordinates
	public double getrange( double x1,double y1, double x2,double y2 )
	{
		double xo = x2-x1;
		double yo = y2-y1;
		double h = Math.sqrt( xo*xo + yo*yo );
		return h;	
	}
	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		turnLeft(90 - e.getBearing());
	}
	
	public void onRobotDeath(RobotDeathEvent e) {
		if (e.getName() == target.name)
			target.distance = 10000; //this will effectively make it search for a new target
	}
	
	public void onHitWall(HitWallEvent e){
		reverse();
	}
	
	public void reverse(){
		direction = direction * -1;
		ahead(140*direction);
	}
	
	public void onWin(WinEvent e){
		ahead(75*direction);
		ahead(0);
		for (int i = 0; i < 100; i ++)
		turnLeft(50*direction);
		direction*=-1;
		}
}

class Enemy {
	/*
	 * ok, we should really be using accessors and mutators here,
	 * (i.e getName() and setName()) but life's too short.
	 */
	String name;
	public double health;
	public double bearing;
	public double head;
	public long ctime; //game time that the scan was produced
	public double speed;
	public double x,y;
	public double distance;
	public double changehead;
	public Point2D.Double guessPosition(long when) {
		double diff = when - ctime;
		double newY, newX;
		double radius = speed/changehead;
		double tothead = diff * changehead;
		newY = y + (Math.sin(head + tothead) * radius) - (Math.sin(head) * radius);
		newX = x + (Math.cos(head) * radius) - (Math.cos(head + tothead) * radius);
		return new Point2D.Double(newX, newY);
	}
}