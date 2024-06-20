package DM;
import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.sql.Time;
import javax.swing.*;
import java.util.Date;
import java.util.Calendar;
import java.util.Random.*;
import java.util.Random;
import java.util.*;
import java.util.regex.Pattern;
import java.awt.geom.*;
import java.io.*;
import java.lang.*;

/**
 * Titan - a robot by Damij
 */
public class Titan extends AdvancedRobot
{


//////////////////////////////
double duck;//////////////////
double firstDuck;/////////////
double prevDuck;//////////////
boolean yeah = false;/////////
int countT;///////////////////
//////////////////////////////

double framesPerSecond;

String Tname;
String Tname2;
String Tname3;

double ourX;
double ourY;
double newX;
double newY;
double moveTime;
double heading;
double pointHeading = 0;
double nextX = 0;
double fish;
double gunTurnRatePercent = 18/360;
double spherePointX;
double spherePointY;
long timeOfShot;
double timeOfHit;
int count = 0;
double trigger = 0;
int side = 1;
int Year;
int month;
int dayOfWeek;
int hour;
int Minute;
int Second;
int corner = 0;
int hellos = 1;
int printColorTimes = 0;
double ourVelocity;
long time;

int tape2 = 0;
double bV = 0;
double print = 0.0;
double timeTillHit;
double ourX2;
double ourY2;
double yDif1;
double xDif1;
double radarOffset;

final int ROAMING = 3;

double distance = Double.POSITIVE_INFINITY;

int tape = 1;
int state;
int apple = 0;

double xDif;
double yDif;
double ourX1;
double ourY1;
double distanceMoved;
double distFromOrigin;
double ox;
double oy;

static final double PI = Math.PI;
static final double leastTurnTime = 20;
boolean moveDirection;
boolean closeToWall = false;
boolean pointCheck;
final boolean MOVE_FORWARD = true;
final boolean MOVE_BACKWARD = false;
final boolean FORWARD = true;
final boolean BACKWARD = false;
boolean performingScan;
boolean haveFreshScan;
static boolean oneOnOne;
static long nextTimeToHit;
public static long nextTimeNeededToHit;
double shots1 = 1;
double hits1;
double misses1;
double accuracy;
int timesPositionDecided;
int shotsSinceHit;
boolean SG;
boolean RM;
static int SGDeaths;
static int RMDeaths;
//public static long nextTimeNeededToHit;
public int reverseTimes;
public static boolean HOT = true;
static long lastTimeMove;
double maxDist;



private RoundRectangle2D.Double playField; // the playable battlefield
private final static int WALLAVOID = 50;	// the area to avoid near walls
static double fieldWidth;
static double fieldHeight;
double countdown;
static double NOTHOTHITS;
double directionChanges;
double directionChangesWanted;
double directionChangeDif = 0;
boolean calledForWall;
int timesSinceWall;
static int direction = 1;
static double headingPlus;



long lastScan;
double X;
double Y;

double desiredDistance;
double desiredX;
double desiredY;
Random random = new Random();
Point2D.Double p;

Enemy x;
Enemy y;
Enemy x2;
Enemy y2;
Enemy x3;
Enemy y3;
Enemy enemy;
Enemy enemy2;
Enemy enemy3;

Enemy currentTarget;

List targetInfo;
List targetNames;


// public static double x;
// public static double y;
 double life;
 double oldLife;
 long pastTime;
 long oldTime;
 double timeDif=0;
 double lifeDif = 0;
 double bPower;

boolean guessed = false;
	/**
	 * run: MoveTest's default behavior
	 */
	public void run() {
		this.clearAllEvents();
		
		setColors();
		
                    desiredDistance = getBattleFieldWidth()/4;

		fieldWidth = getBattleFieldWidth();
		fieldHeight = getBattleFieldHeight();
		
		playField = new RoundRectangle2D.Double( WALLAVOID, WALLAVOID,
	    fieldWidth - 2 * WALLAVOID, fieldHeight - 2 * WALLAVOID, 50, 50);

		Enemy.distance = 50000000;
		enemy.distance = 50000000;
		enemy.distance2 = 50000000;
		enemy.distance3 = 50000000;
		
			trigger = 1000000000;
			setMaxVelocity(8);
			ourVelocity = 8;
                   spherePointX = getX();
                   spherePointY = getY();
                   
				ourX1 = getX();
				ourY1 = getY();

                   targetInfo = Collections.synchronizedList(new ArrayList());
                   targetNames = Collections.synchronizedList(new ArrayList());
                   
				desiredX = getX();
				desiredY = getY();
				enemy = null;
				
					if(getOthers() == 1){oneOnOne = true;}
					else{oneOnOne = false;}
						
							playField = new RoundRectangle2D.Double( WALLAVOID, WALLAVOID,
	   						fieldWidth - 2 * WALLAVOID, fieldHeight - 2 * WALLAVOID, 50, 50);

			setAdjustGunForRobotTurn(true);
			setAdjustRadarForGunTurn(true);

               while(countT<1 && yeah == false){
                firstDuck = System.currentTimeMillis();
                countT++;
                yeah = true;
                }         
		enemy.speedChanges = 0;

		while(yeah) {
			time = getTime();
			if(countT<2){
			out.println("First Duck " + firstDuck);
			countT++;
			turnRadarRightRadians(2*Math.PI);
			}
			/*double newX1 = getX();
			double newY1 = getY();
			xDif = newX1 - ourX1;
			yDif = newY1 - ourY1;
			distanceMoved = Math.sqrt((xDif * xDif) + (yDif * yDif));*/
			
			//if(getTime() == 15)
			//setBack(100);
			//out.println(135%90);
			//if(getTime() == 10){
				//ox = getX();
				//oy = getY();
				///out.println("ox, oy " + ox + ", " + oy);
			//}
			/*if(getTime()>10){
				xDif =  getX() - ox;
				yDif = getY() - oy;
				distanceMoved = Math.sqrt((xDif * xDif) + (yDif * yDif));
			}
				
			if(getTime()==ourVelocity){
				//ourX1=getX();
				//ourY1=getY();
				ourX2 = getX(); 
				ourY2 = getY(); 
				xDif1 = ourX2 - ourX1;
				yDif1 = ourY2 - ourY1;
				out.println(ourX1);
				distFromOrigin = Math.sqrt((xDif * xDif) + (yDif * yDif));
				//out.println(getTime() + " time of accell " + distFromOrigin + " dist of accel");
			}
			
			if(getTime() >= ourVelocity){
				if(apple == 0){
					apple++;
					}
			double newX1 = getX();
			double newY1 = getY();
			//xDif = newX1 - ourX1;
			//yDif = newY1 - ourY1;
			//distanceMoved = Math.sqrt((xDif * xDif) + (yDif * yDif));
			execute();
			}*/
			
			duck = System.currentTimeMillis() - firstDuck;
                    
					//framesPerSecond = getTime()/(duck
					
                   if(duck%100 == 0){                                   //if it has been a tenth of a second...
                    prevDuck = duck/100;                            //make this what we check for next time...
				   }
				
				if(getRadarTurnRemaining() == 0)
					setTurnRadarRight(Double.POSITIVE_INFINITY);
				
				if(count == 0) {
					//turnLeft(getHeading() % 90);
			
			
					//ourX = getX();
					//ourY = getY();
					count = 800;
					//out.println("firstX " + ourX);
					//out.println("firstY " + ourY);
					setAhead(200);
					heading = getHeading();
					//out.println(heading + " firstHeading");
					//double distFromOrigin = 
					double expectedXDif = (((getHeading()%90) / 90)*(100/*-distFromOrigin*/));
					//out.println("expectedXDifference " + expectedXDif);
					double expectedX = ourX + expectedXDif;
					//out.println("\npercent90 " + getHeading()%90);
					//out.println("\nexpectedX " + expectedX + "\n");
					//setTurnRadarRight(Double.POSITIVE_INFINITY);
					execute();
				}
				//if(count < 5){
					//out.println("Time " + (time - ourVelocity) + "\n" + distanceMoved);
					//count++;
				//}
			//}
				if(SGDeaths<RMDeaths){
				SG = true;
				RM = false;
				}
				else{
				RM = true;
				SG = false;
				}
				
				
					if(NOTHOTHITS >= 2)
					HOT = false;
			
					else{
					HOT = true; }
				
				
				
				enemy.currTime = getTime();
				accuracy = hits1/shots1;
				if(shotsSinceHit > 5){
				switchGuns();
				out.println("Switched guns");
				shotsSinceHit = 0;}
				if(oneOnOne)
				doScanner();
				else{
				setTurnRadarRight(980); }
				//if(enemy.name!=null){
				enemy.lastSpeedO = enemy.speedO;
				enemy.update();
				if(enemy.lastSpeedO!=enemy.speedO){
					enemy.speedChanges++;
				}
				if(RM)
				BubbleMove();
				else if(SG)
				checkMovement();
				ParGun();
				//out.println(shotsSinceHit);
				//}
			//double timeTakenToTurnGun = GunTurnRate/
		// Add a custom event named "trigger hit",
			//fish = ParGun();
		addCustomEvent(
			new Condition("triggerhit") { 
				public boolean test() {
					//return (getVelocity() == trigger);
					return (distanceMoved >= trigger);
				};
			}
		);
			execute();
		}
		
    }


	public void setColors()
		{
			
		Random random1 = new Random();
		double random2 = random1.nextDouble();
		double random3 = random1.nextDouble();
		double random4 = random1.nextDouble();
		
		double redColor1 = ((random2*254) + 1);
		double greenColor1 = ((random3*254) + 1);
		double blueColor1 = ((random4*254) + 1);
		
		int redColor = (new Double(redColor1)).intValue();
		int greenColor = (new Double(greenColor1)).intValue();
		int blueColor = (new Double(blueColor1)).intValue();
		
		setColors(new Color(redColor, greenColor, blueColor), new Color(redColor, greenColor, blueColor), new Color(redColor, greenColor, blueColor));
		
		out.println("Red tint:   " + redColor);
		out.println("Green tint: " + greenColor);
		out.println("Blue tint:  " + blueColor);
		
		printColorTimes = 1;

		
	}		
        

    public void BubbleMove() 
	
		{			


        Random duck = new Random();	

        double duck2 = duck.nextDouble();

		double decidedBubbleSize = getBattleFieldWidth()/8;
				
				double theirSpotX = enemy.x;

                double theirSpotY = enemy.y;	
                
                double halfHeight = (getBattleFieldHeight()/2);
                
                double halfWidth = (getBattleFieldWidth()/2);
                
                double pickedCenterX = (halfWidth - theirSpotX)*2 + theirSpotX;
                
                double pickedCenterY = (halfHeight - theirSpotY)*2 + theirSpotY;


			if(getOthers() > 1 && enemy.name2 != null){
					
					double theirSpotX2 = enemy.x2;
					
					double theirSpotY2 = enemy.y2;
					
					double pickedCenterX2 = ((halfWidth - theirSpotX2)*2 + theirSpotX2 + pickedCenterX) / 2;
                
                	double pickedCenterY2 = ((halfHeight - theirSpotY2)*2 + theirSpotY2 + pickedCenterY) / 2;

					pickedCenterX = pickedCenterX2;
						
					pickedCenterY = pickedCenterY2;
					
				if(enemy.name3 != null){
					
						double theirSpotX3 = enemy.x3;
						
						double theirSpotY3 = enemy.y3;
						
						double pickedCenterX3 = ((halfWidth - theirSpotX3)*2 + theirSpotX3 + pickedCenterX) / 3;
                
                		double pickedCenterY3 = ((halfHeight - theirSpotY3)*2 + theirSpotY3 + pickedCenterY) / 3;

						pickedCenterX = pickedCenterX3;
						
						pickedCenterY = pickedCenterY3;
						
				}
				
			}

		double xDif = spherePointX - getX();
		
		double yDif = spherePointY - getY();
		
		double xDifSq = xDif * xDif;
		
		double yDifSq = yDif * yDif;
		
		double distanceToPoint = Math.sqrt(xDifSq + yDifSq);
		
		double offLimitX;
		
		double offLimitY;

		boolean reactToNextShot = false;
		
		if(enemy.shot){
		
		reactToNextShot = negativityChance();
		
		}
					
	if (distanceToPoint < 5 || closeToWall == true || (enemy.shot == true && timesPositionDecided == 0))  {
		
		if(enemy.shot)
		
		timesPositionDecided++;
		
		else{
		
		timesPositionDecided = 0; }
		
		double duck3 = duck.nextDouble();
		
		double bigSphereEdgeX;
		
		double smallSphereEdgeX;
		
		double bigSphereEdgeY;
		
		double smallSphereEdgeY;
		
		bigSphereEdgeX = (pickedCenterX + decidedBubbleSize);
		
		smallSphereEdgeX = (pickedCenterX - decidedBubbleSize);
		
		bigSphereEdgeY = (pickedCenterY + decidedBubbleSize);
		
		smallSphereEdgeY = (pickedCenterY - decidedBubbleSize);


		spherePointX = (duck3 * (bigSphereEdgeX - smallSphereEdgeX) + smallSphereEdgeX);
		
		spherePointY = (duck3 * (bigSphereEdgeY - smallSphereEdgeY) + smallSphereEdgeY);
		
		//spherePointX = pickedCenterX;
		//spherePointY = pickedCenterY;
                /*out.println("X " + Enemy.x);
                out.println("Y " + Enemy.y);
				out.println("Mirrored X " + pickedCenterX);
				out.println("Mirrored Y " + pickedCenterY);
				out.println("spherePointX " + spherePointX);
				out.println("spherePointY " + spherePointY);
				out.println("distance to point " + distanceToPoint);
				out.println("closeToWall? " + closeToWall);*/
                
	}
	
		//setTurnRight(targetBearing - 90);
		
	if (spherePointX < 50 || spherePointX > getBattleFieldWidth() - 50 || spherePointY < 50 || spherePointY > getBattleFieldHeight() - 50) {
			closeToWall = true;
			pointCheck = false;
	}

	else {
			closeToWall = false;
			pointCheck = true;
	}
	
	double distanceToEnemy = Math.sqrt((ourX-enemy.x)*(ourX-enemy.x) + (ourY-enemy.y)*(ourY-enemy.y));
	
	
	/*if(print == 0) {
	
		if (Tname!=null){
	
			if (enemy.lifeDelta<0 && enemy.lifeDelta>=-3) //if the enemy has fired, change speed & direction ////////////////////////////////////////////////////////////////////////////////////////
				{
				//out.println(enemy.lifeDelta + " Enemy lifeDelta");

				bV = 20 - (3*(enemy.lifeDelta*(-1)));
	
				timeTillHit = distanceToEnemy / bV;

				timeOfShot = getTime();
				
				timeOfHit = timeTillHit + getTime();

			out.println(bV + " bV");
			out.println("timeOfShot " + timeOfShot);
			out.println(timeTillHit + " timeTillHit");
			out.println(timeOfHit + " timeOfHit");
			print = 1;
		}
								
				//offLimitX = 

				//out.println(hellos);
				//out.println(getVelocity());
			}
			enemy.lifeDelta = 0;

		}*/
		    if(pointCheck){
                desiredX = spherePointX;
                desiredY = spherePointY;
                MoveToSpot();
			}
		
}
        
        
	//public void MoveToSpot() {
			//boolean left;
			//boolean right;
			
			//if(getX() <= 50 && getY() <= 50){
			//	corner = 1;
			//}
			
			//if(getX()>getBattleFieldWidth()-50 && getY()>getBattleFieldHeight()-50){
			//	corner = 2;
			//}
			
			//if(corner==0){
			//	desiredX=25;
			//	desiredY=25;
			//}
			//if(corner==1){
			//	desiredX = getBattleFieldWidth()-25;
			//	desiredY = getBattleFieldHeight()-25;
			//}
			//if(corner==2){
			//	desiredX = 25;
			//	desiredY = 25;
			//}
			
	void MoveToSpot()
		{

		double relativeDesiredX = desiredX-getX();
		double relativeDesiredY = desiredY-getY();

		double headingToPoint = this.arctan(relativeDesiredX,relativeDesiredY);

		headingToPoint+=random.nextGaussian()/6;

		double tankTurn=headingToPoint-getHeadingRadians();

		tankTurn=normalizeAngle(tankTurn)/2;

		if (tankTurn>Math.PI/4 || tankTurn<-Math.PI/4)
			{
			moveDirection = MOVE_BACKWARD;
			}
		else
			{
			moveDirection = MOVE_FORWARD;
			}

		if (moveDirection)
			{
			setTurnRightRadians(tankTurn);
			setAhead(200);
			}
		else
			{
			setTurnRightRadians(normalizeAngle(tankTurn-Math.PI));
			setBack(200);
			}
		}
		
		/////////////////////////////////////////////////////////////////////////////////////////////
		
		
	public void checkMovement()
		{
			
			if(directionChangeDif <= 0 && HOT == false && timesSinceWall > 4){
				Random duck = new Random();
				directionChangesWanted = Math.round(duck.nextDouble()*4+1);
				directionChanges = 0;
				reverse();
                                calledForWall = false;
                                timesSinceWall = 0;
			}				
			directionChangeDif = directionChangesWanted - directionChanges;
			if(directionChangeDif < 0) directionChangeDif = 0;
			
			if(enemy.shot){
				if(countdown<=0){
				timeOfShot = getTime();
				enemy.shotHeading = getHeading();
				double bulletSpeed = 20 - (3 * (enemy.healthDif * -1));
				double timeTillHit = enemy.distance/bulletSpeed;
				timeOfHit = (timeOfShot + timeTillHit) - 8;
				//System.out.println("distance " + enemy.distance + "   bSpeed " + bulletSpeed + "   time " + timeOfShot + "\nTime of Hit: " + timeOfHit + "   Time " + getTime());
				enemy.update();
				}
			}
			
			countdown = timeOfHit - getTime();
			wallAvoidance();

			if(HOT)
			{
				setAhead(100*direction);
				enemy.escapeAngle = 0;
			}
			
			else if(HOT == false)
				{
			if(enemy.shot){
			//if(countdown < 20 && countdown > 0){ 
				if(reverseTimes==0){
				if(lastTimeMove%2==0){
					if(getVelocity()==0)
					setAhead(50*direction);
					else{
					setAhead(0);}
					lastTimeMove++;
				}
				else {
					setAhead(100*direction);
					lastTimeMove++;
					if(calledForWall)
					timesSinceWall++;
					else{
					directionChanges++;}
				}
				reverseTimes++;
				}
			//}
				}
			}
			if(countdown < 0 && enemy.shot == true){ enemy.shot = false; enemy.escapeAngle = 0;}
				
			if(enemy.shot == false) reverseTimes = 0;
		}																																	

		///////////////////////////////////////////////////////////////////////////////////////////////

	public void doMeleeRadar() {
		
	}



	void ParGun()
		{
		p = new Point2D.Double(enemy.x, enemy.y);

       		nextTimeNeededToHit = (int)Math.round((getrange(getX(),getY(),p.x,p.y)/(20-(3*bPower))));
			nextTimeToHit = nextTimeNeededToHit + getTime();
        	//p = enemy.guessPosition(nextTimeToHit);
			//p = new Point2D(enemy.x, enemy.y);
		if(enemy.RepetitionGun == false){
		if(enemy.numGunScans%6==0||enemy.numGunScans==0){
			enemy.timeOfGunScan = enemy.currTime;
			enemy.oldX = enemy.x;
			enemy.oldY = enemy.y;
			X = 0;
			enemy.numGunScans++;
		}

		else{
			for(int i = 0; i < 12; i ++){ 
			enemy.timeSinceGunScan = enemy.currTime - enemy.timeOfGunScan;
			enemy.yChange = enemy.y - enemy.oldY;
			enemy.xChange = enemy.x - enemy.oldX;
			enemy.ySlopeT = enemy.yChange/enemy.timeSinceGunScan;
			enemy.xSlopeT = enemy.xChange/enemy.timeSinceGunScan;
			enemy.newRelativeX = enemy.xSlopeT * nextTimeNeededToHit;
			enemy.newRelativeY = enemy.ySlopeT * nextTimeNeededToHit;
			Y = enemy.y + enemy.newRelativeY;
			X = enemy.x + enemy.newRelativeX;
			nextTimeNeededToHit = (int)Math.round(getrange(X, Y, getX(), getY())/(20-(3*bPower)));
			}
			enemy.numGunScans++;
		}				
		}
		else{
			Y = enemy.y;
			X = enemy.x;
		}
			double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(Y - getY(),X -  getX()));
			
		if(X!=0)				//If we didn't just reset the grid
		setTurnGunLeftRadians(NormaliseBearing(gunOffset));
		
		if ((getGunHeat() == 0.0)&&(getEnergy() - 1 > bPower)) {
			setFire(bPower);
		}
	}
        
	void doScanner()
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

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		boolean fired = false;
                
            if(Tname == null){
				Tname = e.getName();
				//stop();
				BubbleMove();
				//resume();
				out.println("we got ourselves a new target " + Tname);
				enemy.distance2 = 5000000;
				enemy.distance3 = 5000000;
			}
		    if( Tname != null && Tname != e.getName() && e.getDistance() < enemy.distance ){
                    Tname = e.getName();
				out.println("we got ourselves a closer target " + Tname);
				//stop();
				BubbleMove();
				//resume();
                                enemy.name = e.getName();
				enemy.distance2 = 5000000;
				enemy.distance3 = 5000000;
				//distance = e.getDistance();
            }
			if(e.getDistance() > enemy.distance && e.getDistance() < enemy.distance2 && e.getDistance() < enemy.distance3){
				enemy.name2 = e.getName();
				enemy.distance2 = e.getDistance();
				enemy.x2 = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				enemy.y2 = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				BubbleMove();
				out.println("Secondary target : " + enemy.name2);
			}
			if(e.getDistance() > enemy.distance && e.getDistance() > enemy.distance2 && e.getDistance() < enemy.distance3){
				enemy.name3 = e.getName();
				enemy.distance3 = e.getDistance();
				enemy.x3 = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				enemy.y3 = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				BubbleMove();
				out.println("Thirdary target : " + enemy.name3);
			}
		if(Tname == e.getName()) {
			enemy.name = e.getName();
                    enemy.update();
					//out.println(Enemy.life);

			if(accuracy >= 1) accuracy = 1;

			//out.println(accuracy);

			if(e.getEnergy() != 0){
			bPower = (((e.getVelocity()/8) + (3/(e.getDistance()/getBattleFieldWidth())) + (getEnergy()/e.getEnergy())) / 3 - (1 - accuracy*2));
			}
			else{
			bPower = .1;
			}
			if(bPower>3)bPower=3;
			if(bPower<0)bPower=.1;
		enemy.ctime = getTime();				//game time at which this scan was produced
		enemy.health = e.getEnergy();
		double bulletV = 20 - (3 * bPower);
		enemy.distance = e.getDistance();
		enemy.speed = e.getVelocity();
		double timeTillPossibleHit = Math.round(enemy.distance/bulletV);
		double absoluteBearing = getHeading() + e.getBearing();
		double absBearing = getHeadingRadians() + e.getBearingRadians();
		double bearingFromGun = normalRelativeAngle(absoluteBearing - getGunHeading());
		double expectedXDif = (((e.getHeading()%90) / 90)*((timeTillPossibleHit*enemy.speed)));
		double expectedYDif = (((e.getHeading()%270) / 180)*((timeTillPossibleHit*enemy.speed)));
		//out.println("percent 270 / 180 " + (e.getHeading()%270) + " " + ((e.getHeading()%270)/180));
		//double expectedXDif = 5 * enemy.speed * Math.sin(e.getHeadingRadians()/* - this.getHeadingRadians()*/);
		if(e.getHeading() > 180){
			expectedXDif = expectedXDif*-1;
		}
		if(e.getHeading() > 90 && e.getHeading() < 270){
			expectedYDif = expectedYDif*-1;
		}
		double nextX = enemy.x + expectedXDif;
		double nextY = enemy.y + expectedYDif;
		
		double angleToNextPoint = Math.atan(nextX / nextY);
		
		
		if(SG){
			
			if(enemy.distance<300){
			if(direction==1)
				headingPlus = 15;
			else{
				headingPlus = -15;
			}
		}
		else if(enemy.distance>315){
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
		
		//setTurnGunRight(amountToTurn(angleToNextPoint));
		
		//double bearingToPoint = 
			//turnGunRight(bearingFromGun);
			//out.println("expectedXDif " + expectedXDif);
			//out.println("\ntheir current X " + enemy.x);
			//out.println("their current heading " + e.getHeading());
			//out.println("their next x at given time: " + nextX + ", " + getTime() + timeTillPossibleHit);
			//out.println("their current speed " + enemy.speed + "\n");
			//out.println("expectedYDif " + expectedYDif);
			//out.println("\ntheir current Y " + enemy.y);
			//out.println("their current heading " + e.getHeading());
			//out.println("their next Y at given time: " + nextY + ", " + getTime() + timeTillPossibleHit);
			//out.println("their current speed " + enemy.speed + "\n");
			//if (Math.abs(bearingFromGun) <= 3) {
		//fire(bPower);
			//fired = true;
			//}
		//out.println("possible time till hit " + timeTillPossibleHit);
		enemy.x = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
		enemy.y = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
		
		if(tape == 1){
		//out.println(Enemy.x + " x");
		//out.println(Enemy.y + " y");
		tape++;
		}
	//
	//
	//
	//
	//
	//
	//
	//
	//
	//
	
	//
	//
	//
	//
	//
	//
	//
	//
	}
 }

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	
	public void onBulletHit(BulletHitEvent e){
		shots1++;
		hits1++;
		shotsSinceHit = 0;
	}
	public void onBulletMissed(BulletMissedEvent e){
		shots1++;
		misses1++;
		shotsSinceHit++;
	}
	public void onHitWall(HitWallEvent e){
		if(SG)
		reverse();
	}
	public void onCustomEvent(CustomEvent e)
	{
		
		// If our custom event "triggerhit" went off,
		if (e.getCondition().getName().equals("triggerhit"))
		{
			if(tape2 == 0){
				tape2++;
			// Adjust the trigger value, or
			// else the event will fire again and again and again...
			trigger -= 200;
			newX = getX();
			newY = getY();
			moveTime = getTime();
			//out.println(getTime());
			double xDif = newX - ox;
			double yDif = newY - oy;
			double distanceMoved = Math.sqrt((xDif * xDif) + (yDif * yDif));
			//out.println(distanceMoved);
			out.println("moveTime " + moveTime);
			out.println("distanceMoved " + distanceMoved);
			out.println("duck in seconds " + duck/1000);
			double newTime = getTime() - timeOfShot;
			//out.println(timeOfShot + " timeOfShot");
			//out.println(timeTillHit + " timeTillHit");
			//out.println(timeOfHit + " timeOfHit");
			//out.println(getTime() + " actualTime");
			//out.println(newTime + " actualTimeTillHit");
			//out.println("Current Frames Per Second :  " + Math.round(moveTime/(duck/1000)));
			out.println("newX " + newX);
			//out.println("newY " + newY);
			//heading = getHeading();
			//out.println("Heading : " + heading);
		}
		}
	}
	
	
	public double normalRelativeAngle(double angle) 	
            {
		if (angle > - 180 && angle <= 180)
		return angle;
		double fixedAngle = angle ;
		while (fixedAngle <= -180)                  
		fixedAngle += 360;
		while (fixedAngle > 180)
		fixedAngle -= 360;
		return fixedAngle;
	}
	
	 public double arctan( double dy, double dx ) {

	if( dx == 0.0 )
            {
		if( dy > 0.0 ) return Math.PI*0.5;
		else return Math.PI*1.5;
            }
	else
            {
		if( ( dx > 0.0 ) && ( dy > 0.0 ) ) return normalizeAngle(Math.atan(dy/dx));
		else if( ( dx < 0.0 ) && ( dy > 0.0 ) ) return normalizeAngle(Math.PI - Math.atan(dy/Math.abs(dx)));
		else if( ( dx < 0.0 ) && ( dy < 0.0 ) ) return normalizeAngle(Math.PI + Math.atan(dy/dx));
		else return normalizeAngle(Math.PI*2 - Math.atan(Math.abs(dy)/dx));
            }
    }		

	private double normalizeAngle(double r)
		{
		while(r>Math.PI) r-=2*Math.PI;
		while(r<-Math.PI) r+=2*Math.PI;
		return r;
		}	
	
        public void onRobotDeath(RobotDeathEvent e){
            if(e.getName() == Tname) Tname = null;
        }
	double NormaliseBearing(double ang) {
		if (ang > PI)
			ang -= 2*PI;
		if (ang < -PI)
			ang += 2*PI;
		return ang;
	}
	public double getrange( double x1,double y1, double x2,double y2 )
		{
			double xo = x2-x1;
			double yo = y2-y1;
			double h = Math.sqrt( xo*xo + yo*yo );
			return h;	
		}
    public void switchGuns(){
		if(enemy.RepetitionGun == true){
			enemy.RepetitionGun = false;
			enemy.speedChanges = 0;
			enemy.speedScanTimeDif = 0;
		}
		else{
			enemy.RepetitionGun = true;
			enemy.speedChanges = 0;
			enemy.speedScanTimeDif = 0;
		}
	}
	public void reverse(){
		direction *= -1;
		enemy.lastReverseTime = getTime();
	}
	public void onHitByBullet(HitByBulletEvent e){
		if(SG){
		enemy.timeSinceLastReverse = getTime() - enemy.lastReverseTime;
		long enemyBulletTravelTime = (long) ((enemy.distance) / (20-(3*e.getPower()))); 

    	if (enemy.distance>200&&(enemyBulletTravelTime < enemy.timeSinceLastReverse)) {
        	NOTHOTHITS++;
    	}
		}
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
	public void onDeath(DeathEvent e) {
		if(RM)
		RMDeaths++;
		else if(SG)
		SGDeaths++;
		newX = getX();
		newY = getY();
		double xDif = newX - ourX1;
		double yDif = newY - ourY1;
		int losses, wins;
		//double distanceMoved = Math.sqrt((xDif * xDif) + (yDif * yDif));
		out.println("xDif: " + xDif);
		//out.println("yDif: " + yDif);
		//out.println("realX " + newX);
		//out.println("Move time: " + moveTime);
		//out.println("Velocity/MoveTime " + ourVelocity/moveTime);
		//out.println("X's per click : " + distanceMoved/moveTime);
		//out.println("Degrees per Click " + 360/moveTime);
		out.println("DistanceMoved X : " + xDif);
		//out.println("DistanceMoved Y : " + yDif);
		//out.println("xDif/heading " + xDif/heading);
		//out.println("yDif/heading " + yDif/heading);
		/*if(oneOnOne){
		try {
			BufferedReader r = new BufferedReader(new FileReader(getDataFile("wins.dat")));
			// Try to get the counts
			wins = Integer.parseInt(r.readLine());
			losses = Integer.parseInt(r.readLine());
		} catch (IOException o) {
			// Something went wrong reading the file, reset to 0.
			wins = 0;
			losses = 0;
		} catch (NumberFormatException o) {
			// Something went wrong converting to ints, reset to 0
			wins = 0;
			losses = 0;
		}

		// Increment the # of rounds

		//losses++;
		
		// If we havenn't incremented # of battles already,
		// (Note:  Because robots are only instantiated once per battle,
		//         member variables remain valid throughout it.
			// Increment # of battles
		
		try {
			PrintStream w = new PrintStream(new RobocodeFileOutputStream(getDataFile("wins.dat")));
			w.println(wins);
			w.println(losses);
			// PrintStreams don't throw IOExceptions during prints,
			// they simply set a flag.... so check it here.
			if (w.checkError())
				out.println("I could not write the count!");
			w.close();
		} catch (IOException o) {
			out.println("IOException trying to write: " + o);
		}
		if(oneOnOne){
		out.println("I have " + wins + " career one on one wins and \nI have " + losses + " career one on one losses");
	}
  }
*/
}
	public void onWin(WinEvent e){
		
		int wins, losses;
		
		/***************************************************
		
		***************************************************/
		/*if(oneOnOne){
		try {
			BufferedReader r = new BufferedReader(new FileReader(getDataFile("wins.dat")));
			// Try to get the counts
			wins = Integer.parseInt(r.readLine());
			losses = Integer.parseInt(r.readLine());
		} catch (IOException o) {
			// Something went wrong reading the file, reset to 0.
			wins = 0;
			losses = 0;
		} catch (NumberFormatException o) {
			// Something went wrong converting to ints, reset to 0
			wins = 0;
			losses = 0;
		}
		// Increment the # of rounds
		//wins++;
		
		// If we havenn't incremented # of battles already,
		// (Note:  Because robots are only instantiated once per battle,
		//         member variables remain valid throughout it.
			// Increment # of battles
		
		try {
			PrintStream w = new PrintStream(new RobocodeFileOutputStream(getDataFile("wins.dat")));
			w.println(wins);
			w.println(losses);
			// PrintStreams don't throw IOExceptions during prints,
			// they simply set a flag.... so check it here.
			if (w.checkError())
				out.println("I could not write the count!");
			w.close();
		} catch (IOException o) {
			out.println("IOException trying to write: " + o);
		}*/
		out.println("\n \n \n \n ********************* \n COMMENCE THE JIGGLIN! \n ********************* \n \n \n \n");
		int direction = 1;
		//stop();	
		//if(oneOnOne)
		//out.println("I have " + wins + " career one on one wins and \nI have " + losses + " career one on one losses");
		for(int i = 0; i < 500; i++){
			turnRight(30*direction);
			direction = direction *-1;
			}
		//}
	}
	private double amountToTurn(double heading) {
		double res = heading + getHeadingRadians() - getGunHeadingRadians();
		while (res > Math.PI) res -= 2 * Math.PI;
		while (res < -Math.PI) res += 2 * Math.PI;
		return res;
	}
	public static boolean negativityChance(){
        Random duck = new Random();
        double duck2 = Math.round(duck.nextDouble() + 1);
        int returnAmount = 1;
		boolean value;
        
        if(duck2 ==1){
            returnAmount =  1; 
        }       
        else if(duck2 == 2){
            returnAmount =  -1;
        }
        if(returnAmount==1)
		value = true;
		else{
		value = false; }
        return value;
    }
}

class Enemy{
    static String name;
	static String name2;
	static String name3;
	public static long currTime;
	public static long ctime;
    public static double x;
    public static double y;
	public static double x2;
	public static double y2;
	public static double x3;
	public static double y3;
	public static double ourX;
	public static double ourY;
	public static double nextX;
    static double health;
    static double lastHealth;
    static long timeStamp;
    static long lastTime;
    static double timeDelta = 0;
    static double healthDif = 0;
    static double speed;
    static double distance;
	static double distance2;
	static double distance3;
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
	static boolean shot;
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
	static double shotHeading;
	static double escapeAngle;	
	static long lastReverseTime;
	static long timeSinceLastReverse;
	public static double angChangeTimes;
	
	
	public Point2D.Double guessPosition(long when)
		{

		double newY = 0;
		double newX = 0;

		if(numGunScans%12==0||numGunScans==0){
			timeOfGunScan = currTime;
			oldX = x;
			oldY = y;
			newX = 0;
			numGunScans++;
		}

		else{
			timeSinceGunScan = currTime - timeOfGunScan;
			yChange = y - oldY;
			xChange = x - oldX;
			ySlopeT = yChange/timeSinceGunScan;
			xSlopeT = xChange/timeSinceGunScan;
			newRelativeX = xSlopeT * when;
			newRelativeY = ySlopeT * when;
			newY = y + newRelativeY;
			newX = x + newRelativeX;
			numGunScans++;
		}

		return new Point2D.Double(newX, newY);					
}				
	
        public Enemy(ScannedRobotEvent e){
            //timeStamp = e.getTime();
            //life = e.getEnergy();
            //oldLife = life;
        }

       /* public static void update(ScannedRobotEvent e){
            if(life!=life)
            {
                oldTime = timeStamp;
                timeStamp = e.getTime();
                oldLife = life;
                life = e.getEnergy();
                
                lifeDelta = life - oldLife;
                timeDelta = timeStamp - oldTime;
				oldLife = life;
            }
        }*/
		public static void update(){
			if(currTime != lastTime){
				lastTime = currTime;
			}
				
				if(healthDif != 0)
					healthDif = 0;
				
				if(lastHealth != health){
					healthDif = health - lastHealth;
					lastHealth = health;
				
				if(healthDif < 0 && healthDif >= -3){
					shot = true;
				}
				else{
					shot = false;
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
					//if(speedScanTimeDif<450&&speedChanges>5){
					//	RepetitionGun = true;
						//Titan.shotsSinceHit = 0;
				//	}
					//else/* if(speedScanTimeDif>450&&speedChanges<=5)*/{
					//RepetitionGun = false;
					//speedChanges=0;
					//numSpeedScans=0;
					//}
					//if(speedScanTimeDif>450&&RepetitionGun==false){
					//speedScanTimeDif = 0;
					//speedChanges = 0;
					//}
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
					//if(speedScanTimeDif<450&&speedChanges>5){
						//RepetitionGun = true;
						//Titan.shotsSinceHit = 0;
					//}
					//else /*if(speedScanTimeDif>450&&speedChanges<=5)*/{
					//RepetitionGun = false;
					//speedChanges=0;
					//numSpeedScans=0;
					//}
					//if(speedScanTimeDif>450&&RepetitionGun==false){
					//speedScanTimeDif = 0;
					//speedChanges = 0;
					//}

				}
		}										

}

}
class Point 
{
	
	public double x, y;
	
	public Point(double x, double y) {
		this.x = x; 
		this.y = y;
	}
	
	public double distanceFromOrigin() {
		return Math.sqrt((x * x) + (y * y));
	}		
}
