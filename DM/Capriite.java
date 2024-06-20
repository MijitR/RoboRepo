//
//
//
package DM;
import robocode.*;
import robocode.util.Utils;
import java.awt.geom.Point2D;
import DM.Cap.*;
import java.awt.Color;
import java.sql.Time;
import javax.swing.*;
import java.util.Random.*;
import java.util.Random;
import java.util.*;
import java.io.*;
import java.lang.*;

/**
 * Capricious - Seemingly random; Changing
 */

public class Capriite extends TeamRobot
{


//////////////////////////////
double duck;//////////////////
double firstDuck;/////////////
double prevDuck;//////////////
boolean yeah = false;/////////
int countT;///////////////////
//////////////////////////////

double framesPerSecond;

final double PI = Math.PI;

String Tname;
String Tname2;
String Tname3;

double ourX;
double ourY;
double newX;
double newY;
double moveTime;
double heading;
double spherePointX;
double spherePointY;
int count = 0;
double trigger = 0;
int side = 1;
int corner = 0;
int hellos = 1;
int printColorTimes = 0;
double ourVelocity;
double time;
double timeOfHit;
double countdown;
double badX;
double badY;
double bPower = 3;

//////
//gun
//////
double timeToTurnGun;
double gunScanFrequency=0;
double lastVX=0;
double newVX;
double lastVY=0;
double newVY;
double lastT=0;
double newT;
double oldPX=0;
double oldPY=0;
double newPX;
double newPY;
double oldX2=0;
double oldY2=0;
double newX2;
double newY2;
double diffPX;
double diffPY;
double nextPX;
double nextPY;
double nextX;
double nextY;
double lastTime;

int tape2 = 0;
double bV = 0;
double print = 0.0;
double timeTillHit;
double ourX2;
double ourY2;
double yDif1;
double xDif1;
double distanceToPoint = 0;
static double avgTimes;
double oldSpeed;
double newSpeed;
long timeSinceScan = 0;
long timeOfScan = 0;

double distance = Double.POSITIVE_INFINITY;

int tape = 1;
int state;
int apple = 0;

double xDif;
double yDif;
double ourX1;
double ourY1;
double actual_HitTime = 0;

boolean moveDirection;
boolean closeToWall = false;
boolean pointCheck;
final boolean MOVE_FORWARD = true;
final boolean MOVE_BACKWARD = false;
final boolean FORWARD = true;
final boolean BACKWARD = false;

static boolean oneOnOne;

double desiredDistance;
double desiredX;
double desiredY;
double gunOffset = 0;
double theta = 0;
Random random = new Random();

Cap Circle= new Cap();

Enemy x;
Enemy y;
Enemy x2;
Enemy y2;
Enemy x3;
Enemy y3;
Enemy enemy;
Enemy enemy2;
Enemy enemy3;



double actual_time = 3;


	/**
	 * run: Capriite's default behavior
	 */
	
	public void run() {
            
		this.clearAllEvents();
		
		reset();
		
		setColors();
		
			setMaxVelocity(8);
			ourVelocity = 8;
                        
                   spherePointX = getX();
                   spherePointY = getY();
                   
				ourX1 = getX();
				ourY1 = getY();
                   
				desiredX = getX();
				desiredY = getY();
				enemy = null;
				
					if(getOthers() == 1){oneOnOne = true;}
					else{oneOnOne = false;}
						

			setAdjustGunForRobotTurn(true);
			setAdjustRadarForGunTurn(true);
                        setAdjustRadarForRobotTurn(true);

                while(countT<1 && yeah == false){
                firstDuck = System.currentTimeMillis();
                countT++;
                yeah = true;
                }         
			heading = getHeading();

	while(yeah) {
			time = getTime();
			if(countT<2){
			out.println("First Duck " + firstDuck);
			spherePointX = getX();
			spherePointY = getY();
			timeOfHit = 1;
			countT++;
			}
			
			if(getTime() >= ourVelocity){
				if(apple == 0){
					apple++;
					}
				double newX1 = getX();
				double newY1 = getY();
				execute();
			}
			
			enemy.currTime = getTime();
			
			duck = System.currentTimeMillis() - firstDuck;
                    
			framesPerSecond = Math.round(getTime()/(duck/1000));

				if(getRadarTurnRemaining() == 0)
					setTurnRadarRight(Double.POSITIVE_INFINITY);
			
				if(enemy.name!= null){
					enemy.lastSpeedO = enemy.speedO;
					enemy.update();
					if(enemy.lastSpeedO!=enemy.speedO)enemy.speedChanges+=1;
					countdown = timeOfHit - getTime();
		//out.println(countdown);
					timeOfHit = Circle.CtDn(enemy.shot, enemy.healthDif, getTime(), countdown,  enemy.distance, ourX, ourY);
				}

			if(enemy.shot){
				if(countdown <= 0){
				badX = getX();
				badY = getY();
			  }
			}			
				
				tryGun();
				BubbleMove();
				
			execute();
			
		}
		
    }

	public void reset(){
		
			if(!oneOnOne)
			enemy.speedChanges=0;
		
			avgTimes = 0;
		
			badX = getX();
			badY = getY();
			
			spherePointX = getX();
			spherePointY = getY();
			
			enemy.name = null;
			enemy.name2 = null;
			enemy.name3 = null;
			
			enemy.health = 100;
			enemy.healthDif = 0;
			
			enemy.oldPlaceX = getX();
			enemy.oldPlaceY = getY();
		
			enemy.distance = Double.POSITIVE_INFINITY;
			enemy.distance2 = Double.POSITIVE_INFINITY;
			enemy.distance3 = Double.POSITIVE_INFINITY;
		
			trigger = Double.POSITIVE_INFINITY;
			
			
	}

        /*
        In order for the color to change each round, you have to go to "Options",
            and select the option to allow robots to change color repeatedly.
            Since Capriite only changes colors at the beginning of each round,
			this will not slow down the match.
        */
        
	public void setColors()
		{
			
		Random random1 = new Random();
		//.nextDouble() returns a double between 1 and 0.
		double random2 = random1.nextDouble();
		double random3 = random1.nextDouble();
		double random4 = random1.nextDouble();
		
		double randomRadarColor1 = random1.nextDouble();
		double randomRadarColor2 = random1.nextDouble();
		double randomRadarColor3 = random1.nextDouble();
		//Sets the double to the corresponding number between 1 and 255 using
		//the formula x = ((Random number between 1 and 0 * (Large desired number - Small desired number) + Small desired number)).
		double redColor1 = ((random2*254) + 1);
		double greenColor1 = ((random3*254) + 1);
		double blueColor1 = ((random4*254) + 1);
		
		double randomRadarRed1 = ((randomRadarColor1*254)+1);
		double randomRadarGreen1 = ((randomRadarColor2*254)+1);
		double randomRadarBlue1 = ((randomRadarColor3*254)+1);
		
		//Turns the double measurements into Doubles, and returns their equal integer value.
		int redColor = (new Double(redColor1)).intValue();
		int greenColor = (new Double(greenColor1)).intValue();
		int blueColor = (new Double(blueColor1)).intValue();
		
		int radarRed = (new Double(randomRadarRed1)).intValue();
		int radarGreen = (new Double(randomRadarGreen1)).intValue();
		int radarBlue = (new Double(randomRadarBlue1)).intValue();
		
		setColors(new Color(redColor, greenColor, blueColor), new Color(redColor, greenColor, blueColor), new Color(radarRed, radarGreen, radarBlue));
		
		System.out.println("Base red tint:   " + redColor);
		System.out.println("Base green tint: " + greenColor);
		System.out.println("Base blue tint:  " + blueColor);
		
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

                    //
                    //when moving, make sure to consider robots other than the target
                    //

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
		
		distanceToPoint = Math.sqrt(xDifSq + yDifSq);
		
		double offLimitLowX = theirSpotX - 50;
		
		double offLimitLowY = theirSpotY - 50;
		
		double offLimitHighX = theirSpotX + 50;
		
		double offLimitHighY = theirSpotY + 50;
						
		boolean isIn;
		
		if(theirSpotX<enemy.x+50&&theirSpotX>enemy.x-50&&theirSpotY<enemy.y+50&&theirSpotY>enemy.y-50){
			isIn = true;
		}
		else{
			isIn = false;
		}
		
					
	if (distanceToPoint < 5 || closeToWall == true)  {
		
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

	if (spherePointX < 60 || spherePointX > getBattleFieldWidth() - 60 || spherePointY < 60 || spherePointY > getBattleFieldHeight() - 60) {
			closeToWall = true;
			pointCheck = false;
	}

	else {
			closeToWall = false;
			pointCheck = true;
	}
	
	if (spherePointX > badX - 50 && spherePointX < badX + 50 && getTime() < timeOfHit){
			closeToWall = true;
			pointCheck = false;
	}
	if (spherePointY > badY - 50 && spherePointY < badY + 50 && getTime() < timeOfHit){
			closeToWall = true;
			pointCheck = false;
	}
	
	double distanceToEnemy = Math.sqrt((ourX-enemy.x)*(ourX-enemy.x) + (ourY-enemy.y)*(ourY-enemy.y));
	
	
            if(pointCheck){
                desiredX = spherePointX;
                desiredY = spherePointY;
                MoveToSpot();
                }
		
        }
			
	void MoveToSpot()
		{
		/*
                 *Moving in a straight line for too long isn't a good idea,
                 *So Capriite changes velocities with each of the enemy's shots.
        */
		if(distanceToPoint > 150){
		if(enemy.shot){
			if(ourVelocity == 8){ setMaxVelocity(4); ourVelocity = 4; }
			else if(ourVelocity == 4){ setMaxVelocity(8); ourVelocity = 8; }
		}
		}
		if(distanceToPoint < 150){
			setMaxVelocity(8);
			ourVelocity = 8;
		}
		
		double relativeDesiredX = desiredX-getX();
		double relativeDesiredY = desiredY-getY();

		double headingToPoint = this.arctan(relativeDesiredX,relativeDesiredY);

		headingToPoint+=random.nextGaussian()/5;

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

        
	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		
		if(isTeammate(e.getName())){
			return;
		}

            if(Tname == null){
				reset();
				enemy.numSpeedScans=0;
				enemy.health = e.getEnergy();
				enemy.lastHealth = e.getEnergy();
				Tname = e.getName();
				out.println("We got ourselves a new target " + Tname);
			}
		    if( Tname != null && Tname != e.getName() && e.getDistance() < enemy.distance ){
				reset();
				enemy.numSpeedScans=0;
				enemy.health = e.getEnergy();
				enemy.lastHealth = e.getEnergy();
                    Tname = e.getName();
				out.println("We got ourselves a closer target " + Tname);
                                enemy.name = e.getName();
				distance = e.getDistance();
            }
                    //get a secondary target
			if(e.getDistance() > enemy.distance && e.getDistance() < enemy.distance2 && e.getDistance() < enemy.distance3 && e.getName() != enemy.name){
				enemy.name2 = e.getName();
				enemy.distance2 = e.getDistance();
				enemy.x2 = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				enemy.y2 = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				if(enemy.name != e.getName())
				out.println("Secondary target : " + enemy.name2);
			}
                    //get a third target
			if(e.getDistance() > enemy.distance && e.getDistance() > enemy.distance2 && e.getDistance() < enemy.distance3){
				enemy.name3 = e.getName();
				enemy.distance3 = e.getDistance();
				enemy.x3 = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				enemy.y3 = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
				if(enemy.name != e.getName() && enemy.name2 != e.getName())
				out.println("Thirdary target : " + enemy.name3);
			}

		if(Tname == e.getName()) {
			enemy.name = e.getName();
			enemy.health = e.getEnergy();
			enemy.lastSpeedO = enemy.speedO;
			enemy.update();
			
			if(enemy.lastSpeedO!=enemy.speedO)enemy.speedChanges++;
			out.println(enemy.speedChanges);
			if(e.getEnergy() != 0){
			bPower = (((4/e.getVelocity()) + (2/(e.getDistance()/getBattleFieldWidth())) + (getEnergy()/e.getEnergy())) / 3);
			}
			else{
			bPower = .1;
			}

		timeSinceScan = getTime() - timeOfScan;
		enemy.distance = e.getDistance();
		enemy.speed = e.getVelocity();
		enemy.bearing = e.getBearingRadians();
		enemy.speed = e.getVelocity();
		enemy.heading = e.getHeadingRadians();
        enemy.x = getX() + (Math.sin(getHeadingRadians() + enemy.bearing) * enemy.distance);
		enemy.y = getY() + (Math.cos(getHeadingRadians() + enemy.bearing) * enemy.distance);

		double distanceToPoint = Math.sqrt((enemy.x - getX())*(enemy.x - getX()) + (enemy.y - getY())*(enemy.y - getY()));

		//nextY = enemy.speed*(enemy.distance/(20-3*bPower))*Math.cos(e.getHeadingRadians()) + enemy.x;
		//nextX = enemy.speed*(enemy.distance/(20-3*bPower))*Math.sin(e.getHeadingRadians()) + enemy.y;
		//theta = Utils.normalAbsoluteAngle(Math.atan2(nextX - getX(), nextY - getY()));
		//double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(enemy.y - getY(),enemy.x -  getX()));
		//setTurnGunLeftRadians(NormaliseBearing(gunOffset));
		//fire(bPower);
		//System.out.println("      " + enemy.speed);
		enemy.xDif = enemy.newPlaceX - enemy.oldPlaceX;
		enemy.yDif = enemy.newPlaceY - enemy.oldPlaceY;
	}

 }

	public void tryGun()
		{
				/*if(getTime()!=enemy.t1){
					enemy.t1 = enemy.t2;
					enemy.t2 = getTime();

					double changeY1 = enemy.yDif;
					double changeX1 = enemy.xDif;
					double changeT = enemy.t2 - enemy.t1;
					enemy.reletiveX = changeX1/changeT;    ////X per click
					enemy.reletiveY = changeY1/changeT;    ////Y per click
				for(int i=1; i < 10; i++){
				*/
				//double timeToTurnGun = (gunOffset-getGunHeadingRadians())/20;
				enemy.t = (enemy.distance/(20-3*1));
				double timeSinceScan = getTime()-lastTime;/*
				nextY = enemy.reletiveY*(enemy.t)*Math.cos(enemy.heading) + enemy.x;
				nextX = enemy.reletiveX*(enemy.t)*Math.sin(enemy.heading) + enemy.y;
				}
				gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(enemy.y - getY(),enemy.x -  getX()));
				setTurnGunLeftRadians(NormaliseBearing(gunOffset));
					setFire(bPower);
				//else{
					//bPower-=getEnergy()-bPower+.1;
					//setFire(bPower);
				//}
			}*/

			//if(getTime()-lastTime>5){
				//lastVX =(newVX);
				//lastVY =(newVY);
				//lastT =newT;
				//newVX =((enemy.speed+timeSinceScan)/**Math.sin(enemy.heading)*/);
				//newVY =((enemy.speed+timeSinceScan)/*Math.cos(enemy.heading)*/);
				//newT =(timeSinceScan+getTime());
				
			//oldPX = lastVX/lastT;
			//newPX = newVX/newT;
			//if(enemy.speed>0)
			//diffPX = newPX-oldPX;
			//else{
		//	diffPX = 0;}
		//	out.println(diffPX*enemy.t);
				
			//oldPY = lastVY/lastT;
			//newPY = newVY/newT;
			//if(enemy.speed>0)
			//diffPY = newPY-oldPY;
			//else{
			//diffPY = 0;}
			
			//out.println("    " + diffPY*(enemy.distance/ (20-3*1)));
			//nextPX= diffPX + newPX;
			//nextPY= diffPY + newPY;
			oldX2 = newX2;
			newX2 = enemy.x;
			oldY2 = newY2;
			newY2 = enemy.y;
			
			//diffPX=(newX2-oldX2)/timeSinceScan;
			//diffPY=(newY2-oldY2)/timeSinceScan;
			diffPX=(enemy.speed);
			diffPY=(enemy.speed);
			
			nextX = diffPX*enemy.t*Math.sin(enemy.heading) + enemy.x;
			nextY = enemy.t*diffPY*Math.cos(enemy.heading) + enemy.y;
		
			gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(nextY - getY(),nextX -  getX()));
			setTurnGunLeftRadians(gunOffset);
			fire(1);
			timeToTurnGun=20*gunOffset;
			lastTime=getTime();
			//enemy.gunScanTimes++;
			//if(enemy.gunScanTimes==1)
			//lastTime=getTime();
			//gunScanFrequency = enemy.gunScanTimes/lastTime;
		//else{
			///lastVX =(enemy.speed+enemy.t;
			//lastVY =newVY;
			//lastT =newT;
			
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		
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
	
	public double getrange(double x1,double y1, double x2,double y2) {
    double x = x2-x1;
    double y = y2-y1;
    double h = Math.sqrt( x*x + y*y );
    return h;	
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
	double NormaliseBearing(double ang) {
		if (ang > PI)
			ang -= 2*PI;
		if (ang < -PI)
			ang += 2*PI;
		return ang;
	}
	
        public void onRobotDeath(RobotDeathEvent e){
            //Be open for a new target if ours just died
            if(e.getName() == Tname) Tname = null;
			if(e.getName() == enemy.name) enemy.name = null;
			if(e.getName() == enemy.name2) enemy.name2 = null;
			if(e.getName() == enemy.name3) enemy.name3 = null;
        }
        
	public void onDeath(DeathEvent e){
		int losses, wins;
		reset();
		if(oneOnOne){
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

		// Add to the # of losses

		losses++;
		
		try {
			PrintStream w = new PrintStream(new RobocodeFileOutputStream(getDataFile("wins.dat")));
			w.println(wins);
			w.println(losses);
			// PrintStreams don't throw IOExceptions during prints,
			// they only set a flag.... so check it here.
			if (w.checkError())
				out.println("I could not write the count!");
			w.close();
		} catch (IOException o) {
			out.println("IOException trying to write: " + o);
		}
       	out.println("Fps at end of round: " + framesPerSecond);
		if(oneOnOne){
		out.println("I have " + wins + " career one on one wins and \nI have " + losses + " career one on one losses");
	}
  }
}
	public void onWin(WinEvent e){
		
		int wins = 0, losses = 0;

		/***************************************************
		
		***************************************************/
                
		if(oneOnOne){
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
		// Add to the # of wins
		wins++;
		
		try {
			PrintStream w = new PrintStream(new RobocodeFileOutputStream(getDataFile("wins.dat")));
			w.println(wins);
			w.println(losses);
			// PrintStreams don't throw IOExceptions during prints,
			// they only set a flag.... so check it here.
			if (w.checkError())
				out.println("I could not write the count!");
			w.close();
		} catch (IOException o) {
			out.println("IOException trying to write: " + o);
		}
		}
        out.println("Fps at end of round: " + framesPerSecond);
		out.println("\n \n \n \n ********************* \n COMMENCE THE JIGGLIN! \n ********************* \n \n \n \n");
		int direction = 1;
		//stop();	this will make Capriite go directly to the "jiggle" dance
		if(oneOnOne)
		out.println("I have " + wins + " career one on one wins and \nI have " + losses + " career one on one losses");
		for(int i = 0; i < 500; i++){
			reset();
			turnRight(30*direction);
			direction = direction *-1;
			}
	}
	private double amountToTurn(double heading) {
		double res = heading + getHeadingRadians() - getGunHeadingRadians();
		while (res > Math.PI) res -= 2 * Math.PI;
		while (res < -Math.PI) res += 2 * Math.PI;
		return res;
	}

}

class Enemy{
    static String name;
    static String name2;
    static String name3;
	public static boolean shot;
	static boolean RepetitionGun;
    public static double x;
	public static double gx;
    public static double y;
	public static double gy;
    public static double x2;
    public static double y2;
    public static double x3;
    public static double y3;
    static double speed;
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
	static double bearing;
    static double distance;
    static double distance2;
    static double distance3;
	static double health;
	static double lastHealth;
	static double healthDif;
	static long currTime;
	static long lastTime;
	static long scanTime;
	static long cTime;
	static double newPlaceY;
	static double newPlaceX;
	static double oldPlaceY;
	static double oldPlaceX;
	static double nextX;
	static double nextY;
	static double avgX;
	static double avgY;
	static double xDif;
	static double yDif;
	static double heading;
	static Random headChange1 = new Random();
	static double headChange;
	static double lineSlope;
	static double yInt;
	static double rx1;
	static double rx2;
	static double ry1;
	static double ry2;
	static double t1;
	static double t2;
	static double reletiveY;
	static double reletiveY1;
	static double reletiveX;
	static double reletiveX1;
	static double t;
	
	
		public static void update(){
			//System.out.println(RepetitionGun);
			//System.out.println(speedChanges);
			//headChange = headChange1.nextDouble();
			headChange = .1;
			//System.out.println(headChange);
			//System.out.println(speedScanTimeDif);
			if(currTime != lastTime){
				oldPlaceX = newPlaceX;
				newPlaceX = x;
				oldPlaceY = newPlaceY;
				newPlaceY = y;
				lastTime = currTime;
				
				if(healthDif != 0)
					healthDif = 0;
				
				if(lastHealth != health){
					healthDif = health - lastHealth;
					lastHealth = health;
				  //If there is a significant health difference that didn't
				  //come from us shooting them, they must've shot... or hit a wall.
				if(healthDif < 0 && healthDif >= -3){
					shot = true;
				}
				else{
					shot = false;
				}
			  }
			
			if(speed != oldSpeed){
				if(speed>0){
					speedDif = speed - oldSpeed;
					if(speedDif>0)speedPoNe = 1;				//speeding up
					else if(speedDif<0)speedPoNe = -1;			//slowing down
					speedO = 1;
					if(numSpeedScans<1){
					numSpeedScans+=1;
					speedScanTime = currTime;
					speedScanTimeDif = speedScanTime - lastSpeedScanTime;
					lastSpeedScanTime = speedScanTime;
					}
					speedScanTimeDif = currTime - lastSpeedScanTime;
					if(speedScanTime<100000&&speedChanges>5){
						RepetitionGun = true;
					}
				}
				else if(speed<0){
					speedDif = speed - oldSpeed;
					if(speedDif>0)speedPoNe = -1;				//slowing down
					else if(speedDif<0)speedPoNe = 1;				//speeding up
					speedO = -1;
					if(numSpeedScans<1){
					numSpeedScans+=1;
					speedScanTime = currTime;
					speedScanTimeDif = speedScanTime - lastSpeedScanTime;
					lastSpeedScanTime = speedScanTime;
					}
					speedScanTimeDif = currTime - lastSpeedScanTime;
					if(speedScanTimeDif<450&&speedChanges>8){
						RepetitionGun = true;
					}
					if(speedScanTimeDif>=450)
					numSpeedScans=0;
				}
			
			}
		}
		}

			

	
	
	
	
	
	
	

}