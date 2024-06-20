package DM;
import robocode.*;
import java.awt.Color;
import java.awt.geom.*;
import robocode.util.Utils;
import java.util.Random;

/**
 * Chicken - a robot by Damij
 */
public class Chicken extends TeamRobot
{
Enemy enemy = new Enemy();
Cap Circle= new Cap();
public static double bPower = 1.5;
double timeOfShot;
static double timeOfHit;
static double headingPlus;
double countdown;
static int direction = 1;
double radarOffset;
final double PI = Math.PI;
final double waitTime = 10;
static int shots;
static int hits;
static double fieldWidth;
static double fieldHeight;
static long currTime;
static long nextTimeToHit;
static double heading;
private RoundRectangle2D.Double playField; // the playable battlefield
private final static int WALLAVOID = 50;	// the area to avoid near walls
public static double WALL_STICK = 160;
public static long nextTime;
Point2D.Double p;
Random headPlus = new Random();
public static long nextTimeNeededToHit;
public int reverseTimes;
public static boolean HOT = true;
static long lastTimeMove;
double maxDist;
double directionChanges;
double directionChangesWanted;
double directionChangeDif = 0;
boolean calledForWall;
int timesSinceWall;
double shots1;
double hits1;
double misses1;
double accuracy;
static double NOTHOTHITS = 0;
static boolean circularMovement;
static double CircleHits = 2;
static boolean randomMovement;
static double randomHits = 2;
static int timesThisShot;
static double hitsSinceChange = 0;
static double timeTillHit;
static double bulletSpeed;
static boolean choice;
static double goHits = 5;
static double reverseHits = 5;
static double choiceTimes = 5;
static int resetTimes;
static double pGo = 20;
static double pRev = 80;
static double timesToSwitchGun = 5;
	/**
	 * run: Chicken's default behavior
	 */
int in = 0;
	public void run() {
		
		setAdjustRadarForGunTurn(true);
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForRobotTurn(true);


		//PURPLE AND WHITE GO KAHOKS
		setColors(new Color(139,63,169),new Color(139,63,69),Color.white);
		enemy.health = 100;
		setTurnRadarRight(360);
		
		fieldWidth = getBattleFieldWidth();
		fieldHeight = getBattleFieldHeight();
		
		directionChanges = 0;
								
		playField = new RoundRectangle2D.Double( WALLAVOID, WALLAVOID,
	    fieldWidth - 2 * WALLAVOID, fieldHeight - 2 * WALLAVOID, 50, 50);
	
				
		maxDist = Math.sqrt((fieldWidth*fieldWidth)+(fieldHeight*fieldHeight));
	
		if(getOthers()>1||enemy.RepetitionGun==false)
		enemy.speedChanges=0;
		
		enemy.distance = 1000000;
		
		turnRadarRightRadians(2*PI);
		
		heading = getHeading();
		
		countdown = 0;
		
		if(HOT)
		{
			lastTimeMove++;
		}
		
		execute();
		
		while(true) {
			
			setTurnRadarRight(Double.POSITIVE_INFINITY);
						
			enemy.currTime = getTime();
			
			enemy.ourV = getVelocity();
                        
            enemy.ourX = getX();
                        
            enemy.ourY = getY();

			accuracy = hits1/shots1;
			
			decideMovement();
			
			if(timesToSwitchGun == 0)
			resetGun();
			
			//randomMovement = false;
			//HOT = false;
			//circularMovement = true;
			
			//if(countdown <= 0){
			//enemy.update();
			///if(enemy.shot)
			//countdown = enemy.whenTheyHit - countdown;
			//}
			if(countdown>(maxDist/9)) countdown = 0;
						
			//enemy.whenTheyHit = enemy.distance / (20-(3*enemy.shotPower)) + getTime();
			
			enemy.update();
			doRadar();
			checkMovement();
			doGun();
			

			execute();
			
		}
	}
	
	public void decideMovement(){
					
			if(NOTHOTHITS <= getRoundNum()/2){
			HOT = true;
			}
			
			else if(CircleHits - 2< getRoundNum() + 2 && NOTHOTHITS > getRoundNum()/2){
			circularMovement = true;
			HOT = false;
			randomMovement = false;
			}
			
			else if(CircleHits - 2>= getRoundNum() + 2 && NOTHOTHITS > getRoundNum()/2){
			randomMovement = true;
			HOT = false;
			circularMovement = false;
			}
			
	}
	
	public void rotateMovements(){
		if(circularMovement){
		circularMovement = false;
		HOT = false;
		randomMovement = true;
		hitsSinceChange = 0;
		return;
		}
		else if(randomMovement){
		circularMovement = true;
		HOT = false;
		randomMovement = false;
		hitsSinceChange = 0;
		return;
		}
	}
	
	public void resetGun(){
		timesToSwitchGun = 5;
		enemy.RepetitionGun = false;
		enemy.speedChanges = 0;
		enemy.speedChangeTime = 0;
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		double headingPlus = 900000;
		if ((e.getDistance() < enemy.distance)||(enemy.name == e.getName())||(enemy.name == null) && isTeammate(e.getName())==false) {
			enemy.name = e.getName();
			enemy.distance = e.getDistance();
			enemy.health = e.getEnergy();
			enemy.speed = e.getVelocity();
			enemy.x = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
			enemy.y = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
			
			double absbearing_rad = (getHeadingRadians()+e.getBearingRadians())%(2*PI);
			double h = NormaliseBearing(e.getHeadingRadians() - enemy.head);
			h = h/(getTime() - enemy.ctime);
			enemy.changeHead = h;
			enemy.headingChange = h;
			enemy.head = e.getHeadingRadians();
			enemy.bearing = e.getBearingRadians();
			enemy.headingChangeRate = enemy.headingChange/(getTime() - enemy.lastHeadingCheck);
			enemy.lastHeadingCheck = e.getTime();
			enemy.ctime = getTime();				//game time at which this scan was produced
						
			enemy.escapeAngle = enemy.shotHeading - getHeading();
			
			enemy.lastSpeedO = enemy.speedO;
			
			enemy.update();
			
			if(enemy.lastSpeedO!=enemy.speedO){
			enemy.speedChanges++;
			enemy.speedChangeTime = getTime();
			enemy.speedChangeTimeDif = enemy.speedChangeTime - enemy.lastSpeedChangeTime;
			enemy.avgSpeedChangeTime = (enemy.avgSpeedChangeTime+enemy.speedChangeTimeDif)/2;
			enemy.nextSpeedChangeTime = Math.round(enemy.avgSpeedChangeTime + enemy.speedChangeTime);
			enemy.lastSpeedChangeTime = getTime();
			}
			enemy.timeTillNextSpeedChange = enemy.nextSpeedChangeTime - getTime();
			
			//bPower = Math.round((4/enemy.speed + getBattleFieldHeight()/enemy.distance + getEnergy()/e.getEnergy()*2)/3 - (1 - accuracy));
			bPower = enemy.health/4  - (2 - (accuracy*2));
			//bPower -= .5;
		
		//if(!randomMovement){	
			
			if ( bPower>3||bPower==Double.POSITIVE_INFINITY);
			bPower = 3;
			
			if(e.getEnergy()<=.4)
			bPower = .1;
		//}
					//out.println(enemy.shot);
		if(enemy.distance<200){
			if(direction==1)
				headingPlus = 15;
			else{
				headingPlus = -15;
			}
		}
		else if(enemy.distance>200){
			if(direction==1)
				headingPlus = -15;
			else{
				headingPlus = 15;
			}
		}
		
		if(e.getBearing()>0)
		setTurnRight((e.getBearing() - 90) - headingPlus/* - enemy.escapeAngle*/);
		else if(e.getBearing()<=0)
		setTurnRight((e.getBearing() + 90) + headingPlus/* - enemy.escapeAngle*/);

	}
		else{
			return;
		}
	}
	void doGun() {
		p = new Point2D.Double(enemy.x, enemy.y);

       			nextTimeNeededToHit = (int)Math.round((getrange(getX(),getY(),p.x,p.y)/(20-(3*bPower))));
			nextTimeToHit = nextTimeNeededToHit + getTime();
        		p = enemy.guessPosition(nextTimeToHit);

		double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(p.y - getY(),p.x -  getX()));
		if(p.x!=0)				//If we didn't just reset the grid
		setTurnGunLeftRadians(NormaliseBearing(gunOffset));
		
		if ((getGunHeat() == 0.0)&&(getEnergy() > bPower + .1)) {
			//if(p.x!=0)
			enemy.lastHeadingChange = enemy.headingChange;
			setFire(bPower);
		}
	}
	void doRadar()
		{
			if(getTime() - enemy.ctime > 4){
				radarOffset = 4*PI;
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
	void checkMovement()
		{
		//if(!randomMovement){
			boolean Ok = true;
			if(circularMovement)
			Ok = checkWall(3);
		//}
			if(directionChangeDif <= 0 && circularMovement && Ok){
				Random duck = new Random();
				directionChangesWanted = Math.round(duck.nextDouble()*3+2);
				directionChanges = 0;
				reverse();
                                calledForWall = false;
                                timesSinceWall = 0;
			}	
			directionChangeDif = directionChangesWanted - directionChanges;
			
			if(enemy.shot){
				if(countdown<=0){
				timeOfShot = getTime();
				enemy.shotHeading = getHeading();
				bulletSpeed = 20 - (3 * (enemy.healthDif * -1));
				timeTillHit = enemy.distance/bulletSpeed;
				timeOfHit = (timeOfShot + timeTillHit) - 8;
				//System.out.println("distance " + enemy.distance + "   bSpeed " + bulletSpeed + "   time " + timeOfShot + "\nTime of Hit: " + timeOfHit + "   Time " + getTime());
				enemy.update();
				}
			}

			double dist = enemy.distance/(20-bPower);
			timeOfHit = (timeOfShot + timeTillHit);
			countdown = timeOfHit - getTime();
			wallAvoidance();

			if(HOT)
			{
				setAhead(100*direction);
				enemy.escapeAngle = 0;
			}

			else if(circularMovement)
				{
			if(enemy.shot){
				if(reverseTimes==0){
					if(lastTimeMove%2==0){
					if(getVelocity()==0)
					setAhead(80*direction);
					else{
					setAhead(0);}
					lastTimeMove++;
					timesSinceWall++;
					directionChanges++;
					reverseTimes++;
				}
					else{
					if(getVelocity()==0)
					setAhead(80*direction);
					else{
					setAhead(0);}
					lastTimeMove++;
					timesSinceWall++;
					directionChanges++;
					reverseTimes++;
					}
			}
			}
		}
	
			/*		else if(randomMovement){
				setAhead(100*direction);
				directionChanges ++;
				timesSinceWall++;
					}*/
			if(randomMovement){
				out.println(countdown);
			if (getDistanceRemaining() == 0)
				setAhead((Math.random()-.5)*countdown*1.5+((Math.random()-.2))*((Math.sin(enemy.distance)*(enemy.distance/(countdown/(8*bPower))))));
			}
			if(countdown < 0 && enemy.shot == true){ enemy.shot = false; enemy.escapeAngle = 0;}
				
			if(enemy.shot == false) reverseTimes = 0;
		
	}
		
	public boolean checkWall(double acceptedVar) {
		if(calledForWall){
			if(timesSinceWall > acceptedVar)
				return true;
			else{
				return false;
			}
		}
		else if(!calledForWall){
			return true;
		}
		return false;
	}
	
	public void averageBulletPower(double bPower)
		{
			
		}
		
	public boolean distanceChooser(){
		double i = negativityChance(goHits, reverseHits);
		boolean go;
		boolean stopShort;
		if(i < 0){
		go = true;
		return true;
		}
		else if(i > 0){
		stopShort = true;
		return false;
		}
		return false;
	}
	
    public static double negativityChance(double GoHits, double revHits){
        Random duck = new Random();
		double reverseAmounts = 1;
		double returnAmount = 1;
		choiceTimes++;
		
		//pGo = goHits/(choiceTimes) * 100;
		//pRev = revHits/(choiceTimes) * 100;
		
		double duck2 = Math.round(duck.nextDouble()*99+1);
		if(duck2>=1 && duck2<=pGo){
			returnAmount = -1;
		}
		else if(duck2>pGo && duck2<=100){
			returnAmount = 1;
		}
		System.out.println("PGo " + pGo + "\nPRev " + pRev + "\nDuck2 " + duck2 + "----\n");
        return returnAmount;
    }

	public void onHitWall(HitWallEvent e){
		reverse();
                calledForWall = true;
                timesSinceWall = 0;
	}
	public void onHitRobot(HitRobotEvent e){
		reverse();
                calledForWall = false;
	}
	public void onBulletHit(BulletHitEvent e){
		shots1++;
		hits1++;
	}
	public void onBulletMissed(BulletMissedEvent e){
		shots1++;
		misses1++;
		if(enemy.RepetitionGun)
		timesToSwitchGun--;
	}
	public void onRobotDeath(RobotDeathEvent e){
		if(e.getName() == enemy.name)
			enemy.name = null;
		if(goHits>reverseHits)
		goHits = reverseHits;
		else if(goHits<reverseHits)
		reverseHits = goHits + 1;
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
	public static double getrange( double x1,double y1, double x2,double y2 )
	{
		double xo = x2-x1;
		double yo = y2-y1;
		double h = Math.sqrt( xo*xo + yo*yo );
		return h;	
	}
	public static double getrange(Point2D.Double checkPoint, double x, double y)
	{
		double xo = checkPoint.x - x;
		double yo = checkPoint.y - y;
		double h = Math.sqrt( xo*xo + yo*yo );
		return h;	
	}
	public void reverse(){
		direction *= -1;
		enemy.lastReverseTime = getTime();
	}
	
	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 **/
	
	public void onHitByBullet(HitByBulletEvent e){
		
		enemy.timeSinceLastReverse = getTime() - enemy.lastReverseTime;
		long enemyBulletTravelTime = (long) ((enemy.distance) / (20-(3*e.getPower()))); 
		hitsSinceChange++;
    	if (enemy.distance>200&&(enemyBulletTravelTime < enemy.timeSinceLastReverse) && HOT == true) {
        	NOTHOTHITS++;
			hitsSinceChange = 0;
    	}

		if(circularMovement)
		CircleHits++;
		
		if(randomMovement){
		randomHits++;
		if(choice){
			goHits+=4;
			pGo -= 20;
			pRev += 20;
		}
		else if(!choice){
			reverseHits+=4;
			pGo += 20;
			pRev -= 20;
		}
		}
		
		if(randomMovement)
			bPower = e.getPower();

  }

	public static double degToRad(double angle){
		double RadAngle;
		RadAngle = 2*Math.PI*(angle/360);
		return RadAngle;
	}
	
	public static double radToDeg(double angle){
		double degAngle;
		degAngle = 360*(angle/(2*Math.PI));
		return degAngle;
	}
			
	public void wallAvoidance() {
        if(!playField.contains(getX(),getY())&&enemy.angChangeTimes==0) {
			reverse();
			calledForWall = true;
			timesSinceWall = 0;
			enemy.angChangeTimes++;
			execute();
        }
		else if(playField.contains(getX(),getY())){
			enemy.angChangeTimes=0;
		}
    }

}

class Enemy{

	static String name;
	static double distance;
	static double lastDistance;
	static double speed;
	static double newSpeed;
	static double speedChange;
	static boolean hitWall;
	static boolean shot;
	static double health;
	static double lastHealth;
	static double healthDif;
	static long currTime;
	static long lastTime;
	static double x;
	static double y;
    static double ourX;
    static double ourY;
	public double bearing;
	public double head;
	public long ctime;                 //game time that the scan was produced
	public static double ourV;
	public static double ourVChange;
	public static boolean slowing;
	public static double changeHead;
	public double angChangeTimes;
	public double diff;
	public static boolean RepetitionGun = false;
	static double oldSpeed;
	static double speedDif;
	static double speedChanges;
	static int speedPoNe;				//PoNe = Positive/Negative
	static int speedO;
	static int lastSpeedO;
	static long speedScanTime;
	static long speedScanTimeDif;
	static long lastSpeedScanTime;
	static int numSpeedScans;
	static int gunScanTimes;
	static long lastSpeedChangeTime;
	static long speedChangeTime;
	static long speedChangeTimeDif;
	static double avgSpeedChangeTime;
	static double nextSpeedChangeTime;
	static double timeTillNextSpeedChange;
	static double timeOfGunScan;
	static double timeSinceGunScan;
	static double numGunScans;
	static double xChange;
	static double yChange;
	static double yTSlope;
	static double xTSlope;
	static double oldY;
	static double oldX;
	static double ySlopeT;
	static double xSlopeT;
	static double newRelativeY;
	static double newRelativeX;
	static long lastReverseTime;
	static long timeSinceLastReverse;
	static double shotHeading;
	static double escapeAngle;
	static double shotTime;
	static double lastShotTime;
	static double rolledAverageTimeBetweenShots;
	static double TotalShotDelta;
	static double shotDelta;
	static double shotPower;
	static double lastShotPower;
	static double rolledAverageShotPower;
	static double TotalShotPowerDelta;
	static double whenTheyHit;
	static double lastHead;
	static double headingChange;
	static double lastHeadingChange;
	static double lastHeadingCheck;
	static double headingChangeRate;
	static double Circumference;
	static double arcAngle;
	static double AverageHeadChange;
	
	public Point2D.Double guessPosition(long when){

	double newX = 0;
	double newY = 0;
	double diff = when - currTime;

	if(RepetitionGun==false){
		if(Math.abs(AverageHeadChange) < 0.07 || (distance - .3 <= lastDistance && distance + .3 >= lastDistance)){
		//System.out.println("Par Gun"+ (Math.abs(AverageHeadChange)));
		if(numGunScans%3==0||numGunScans==0){
			timeOfGunScan = currTime;
			oldX = x;
			oldY = y;
			newX = 0;
		}

		else{
                    for(int i = 0; i < 10; i ++){
			timeSinceGunScan = currTime - timeOfGunScan;
			yChange = y - oldY;
			xChange = x - oldX;
			ySlopeT = yChange/timeSinceGunScan;
			xSlopeT = xChange/timeSinceGunScan;
			newRelativeX = xSlopeT * diff;
			newRelativeY = ySlopeT * diff;
			newY = y + newRelativeY;
			newX = x + newRelativeX;
                        diff = Chicken.getrange(newX, newY, ourX, ourY)/(20-(3*Chicken.bPower));
                    }
		}
                numGunScans++;
	}
	else {
		//System.out.println("C gun" + (Math.abs(AverageHeadChange)));
		double radius = speed/changeHead;
		for(int i = 0; i < 10; i ++){
		double tothead = diff * changeHead;
		newY = y + (Math.sin(head + tothead) * radius) - (Math.sin(head) * radius);
		newX = x + (Math.cos(head) * radius) - (Math.cos(head + tothead) * radius);
		diff = Chicken.getrange(newX, newY, ourX, ourY)/(20-(3*Chicken.bPower));
        }
	}
		}
	else{
		//System.out.println("Reap Gun");
		newY = y;
		newX = x;
	}
		lastDistance = distance;
		return new Point2D.Double(newX, newY);
	}

		public static void update(){
			if(currTime != lastTime){
				lastTime = currTime;
			}
			
			speedChange  = speed - newSpeed;
			newSpeed = speed;
			
			if(AverageHeadChange!=0)
			AverageHeadChange = (AverageHeadChange + changeHead)/2;
			else{
			AverageHeadChange = changeHead;
			}
			
			if(speedChange<3&&speedChange>=0)
			hitWall = false;
			if(speedChange>-3&&speedChange<=0)
			hitWall = false;
			
			if(speedChange<=8&&speedChange>=3)
			hitWall = true;
			if(speedChange>=-8&&speedChange<=-3)
			hitWall = true;
			
				ourVChange = ourV - ourVChange;
				if(ourVChange < 0)
				slowing = true;
				else{
				slowing = false;}
				
				if(healthDif != 0)
					healthDif = 0;
				
				if(lastHealth != health){
					healthDif = health - lastHealth;
					lastHealth = health;
				
				if(healthDif < 0 && healthDif >= -3 && !hitWall){
					shot = true;
					
					//Avarage Time Between Shots
					shotTime = currTime;
					shotDelta = shotTime - lastShotTime;
					if(TotalShotDelta!=0)
					rolledAverageTimeBetweenShots = ((shotDelta + TotalShotDelta)/2 + TotalShotDelta)/2;
					else{
					TotalShotDelta = shotDelta;
					rolledAverageTimeBetweenShots = TotalShotDelta;
					}
					
					//Average Power of Shots
					shotPower = healthDif*-1;
					if(rolledAverageShotPower!=0)
					rolledAverageShotPower = ((shotPower + TotalShotPowerDelta)/2 + TotalShotPowerDelta)/2;
					else{
					rolledAverageShotPower = shotPower;
					TotalShotPowerDelta = shotPower;
					}
					
				}
				else{
					shot = false;
					escapeAngle = 0;
				}
			  }
			  else{
				healthDif = 0;
			}
			
			if(speed != oldSpeed){
				if(speed>0){
					speedDif = speed - oldSpeed;
					if(speedDif>0)speedPoNe = 1;				//speeding up
					else if(speedDif<0)speedPoNe = -1;			//slowing down
					speedO = 1;

					numSpeedScans+=1;
					speedScanTime = currTime;
					speedScanTimeDif = speedScanTime - lastSpeedScanTime;
					lastSpeedScanTime = speedScanTime;

					speedScanTimeDif = currTime - lastSpeedScanTime;
					if(speedScanTime<450&&speedChanges>5&&(Chicken.nextTimeToHit)>timeTillNextSpeedChange){
						RepetitionGun = true;
					}
					else if(speedScanTimeDif>450){
					RepetitionGun = false;
					speedChanges=0;
					numSpeedScans=0;
					}
					if ((Chicken.nextTimeToHit)<timeTillNextSpeedChange){
						RepetitionGun = false;
					}
					if(speedChanges<5){
						RepetitionGun = false;
					}
				}
				else if(speed<0){
					speedDif = speed - oldSpeed;
					if(speedDif>0)speedPoNe = -1;				//slowing down
					else if(speedDif<0)speedPoNe = 1;				//speeding up
					speedO = -1;
					
					numSpeedScans+=1;
					speedScanTime = currTime;
					speedScanTimeDif = speedScanTime - lastSpeedScanTime;
					lastSpeedScanTime = speedScanTime;
					
					speedScanTimeDif = currTime - lastSpeedScanTime;
					if(speedScanTimeDif<450&&speedChanges>5&&(Chicken.nextTimeToHit)>timeTillNextSpeedChange){
						RepetitionGun = true;
					}
					else if(speedScanTimeDif>450){
					RepetitionGun = false;
					speedChanges=0;
					numSpeedScans=0;
					}
					if ((Chicken.nextTimeToHit)<timeTillNextSpeedChange){
						RepetitionGun = false;
					}
					if(speedChanges<5){
						RepetitionGun = false;
					}
				}
		}
		}
		
	}
