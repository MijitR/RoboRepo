

/////////////figure shot power(pattern in enemies)
////////////change guns...

package DM;
import DM.pgun.*;
import DM.utils.*;
import robocode.*;
import robocode.util.Utils;
import java.io.*;
import java.awt.Color;
import java.awt.geom.*;
import java.util.*;
import java.util.Random.*;
import javax.swing.*;
/**
 *                  -=DAMIJ=-
 * Ethereal - Not of this world; heavenly; spiritual. 
 *
 * Dedicated to Brendan for constructive abuse.
 *
 */
public class Ethereal extends TeamRobot
{
String nameOfPrey;
//usefull, random, and generally unused variables.
int i = 1;
int h = 0;
int r = 0;
int turns = 0;
int o;

double shots = 0;
double hits = 0;
int misses = 0;
int bulletHitBullet = 0;
int timeSinceScan = 0;
int count = 0;
int energy;
int direction = 1;
int turnAmount = 1;
int cornerNumber = 0;
int losses = 0;
String[] Teammates;

long currTime;

static final double PI = Math.PI;
static final double RADAR_OFFSET = PI/8;

Point2D	myPosition;					// my current position
Point2D	centerPoint;				// the centre of the battlefield

double	battleWidth;		// the width of the battlefield
double	battleHeight;	// the heigth of the battlefield
final static int WALLAVOID = 35;	// the area to avoid near walls
final static int RADARTIMEOUT = 20;	// turn radar at least once in this time

HashMap	Enemies;				// all available targets
EnemyInfo currentTarget;		// the current target
double lastscan;				// time of last radarscan

double ourHealth;
double theShotPower;
double targetDistance;
double targetBearing;
double travelDistance;
double moveAmount;
double fixedX;
double fixedY;
double desiredX;
double desiredY;
double fixedDuck;
double spherePointX;
double spherePointY;
double wins = 0;
double ourLife;

int target;
int enemies = 50;
int currentEnemies = enemies;
int currentScanTarget = -1;

double myX, myY;
double bestX, bestY;
double targetX, targetY;
double predictedX, predictedY;
double myHeading;
double myVelocity;
double myEnergy;
double circularLimit = 27;
double averageEnergy;

long gameTime;

boolean ramming;
boolean clingToWalls;
static boolean minRisk = true;
static boolean nextLong = false;
static boolean bubbleMove = false;
boolean moveDirection;
boolean closeToWall = false;
boolean weHadTeammates;
final boolean MOVE_FORWARD = true;
final boolean MOVE_BACKWARD = false;
final boolean FORWARD = true;
final boolean BACKWARD = false;

ScannedRobotEvent lastScanEvent;

Target robot;

enemy[] enemy;

Random random = new Random();

	public void run() {
			enemies = currentEnemies = getOthers() + 1;
			spherePointX = getX();
			spherePointY = getY();
			if(getRoundNum() == 0) {
				out.println("I come to purge this arena!");
			}
			else if (getRoundNum() + 1 != getNumRounds()) {
				int roundsPlayed = getRoundNum();
				int totalRounds = getNumRounds();
				int roundsLeft = getNumRounds() - getRoundNum();
				out.println(roundsLeft + " rounds left."); 
			}
			else if (getRoundNum() + 1 == getNumRounds()) {
				out.println("1 round left.");
			}
			
			enemy = new enemy[enemies];
				for(int i = 0; i < enemies; i++){
			enemy[i] = new enemy();
			enemy[i].name = "";
			enemy[i].active = 1; //1 = unknown, 2 = known, 0 = dead
				}
			
                	out.println("2kj2lj2lbm: " + bubbleMove + "\n nL: " + nextLong + "\n mr: " + minRisk);
 
                	battleHeight = getBattleFieldHeight();
                	battleWidth = getBattleFieldWidth();

			centerPoint = new Point2D.Double( battleWidth / 2, battleHeight / 2 );
                
                	Enemies = new HashMap();

			robot = new Target();
                	
			Teammates = getTeammates();
			if(Teammates != null) weHadTeammates = true;
			else weHadTeammates = false;
    while(true) {
              /*  currTime = getTime();
					while(currTime <= 5){
						out.println(currTime%5);
					}  */

			setAdjustGunForRobotTurn(true);
			setAdjustRadarForGunTurn(true);
                setAdjustRadarForGunTurn(true);
                count++;
                ourLife = getEnergy();
						if(getOthers() == 1) {
							setColors(new Color(254, 151, 0), new Color(254, 151, 0), new Color(254, 151, 0));
						}
						else if(getOthers() == 2) {
							setColors(new Color(230, 254, 0), new Color(230, 254, 0), new Color(236, 114, 213));
						}
						else if(getOthers() > 2) {
							setColors(new Color(141, 255, 152), new Color(141, 255, 152), new Color(185, 190, 195));
						}
                if (count > 11){
                     nameOfPrey = null;
                }
			setVariables();
			
			if(currentEnemies > 1){doMeleeRadar();}else{turnRadarRightRadians(Double.POSITIVE_INFINITY);}
            
			execute();
                }
			
        }
        
	/*
     *--------------------
	 * --ONsCANNEDrOBOT---
	 *--------------------
    */
    public void onScannedRobot(ScannedRobotEvent e ) {

            if (isTeammate(e.getName()))                        
		{
			return;
		}
                
		boolean fired = false;
		
                if(getGunHeat()==0) fired = true;        

		if ((fired == true) || (currTime >= (lastscan + RADARTIMEOUT))) {
			lastscan = currTime;
			
			/*if(minRisk) {
				Random duck = new Random();
				double maxSpeed;
				double duckSpeed = duck.nextDouble();
				double currentV = getVelocity();
				double currentVadjH = currentV + 4;
				double currentVadjL = currentV - 4;
				double maxSpeed1 = Math.round(duckSpeed * (8 - currentVadjH) + currentVadjH);
				double maxSpeed2 = Math.round(duckSpeed * (currentVadjL - 2) + 2);
				double SpeedDif1 = currentV - maxSpeed1;
				double SpeedDif2 = currentV - maxSpeed2;
					if(SpeedDif1 < 0) SpeedDif1 *= -1;
					if(SpeedDif2 < 0) SpeedDif2 *= -1;
				double maxSpeedDif = Math.max(SpeedDif1, SpeedDif2);
					if(maxSpeedDif == SpeedDif1) {
						maxSpeed = maxSpeed1;
					}
					else {
						maxSpeed = maxSpeed2;
					}
				setMaxVelocity(maxSpeed);
			}
			
		}
		else {
			setMaxVelocity(8);
		}*/


            targetDistance = e.getDistance();
            targetBearing = e.getBearing();
            lastScanEvent = e;
            timeSinceScan = 0;
            double theirLife = e.getEnergy();
            int enemiesLeft = getOthers();

            ourHealth = getEnergy();
			robot.X = (getX());
			robot.Y = (getY());
			double radarHeadingRadians = getRadarHeadingRadians();
			robot.V = (e.getVelocity());



		setVariables();
                    double absoluteBearing = Utils.normalAbsoluteAngle(myHeading + e.getBearingRadians());
		double enemyHeading = e.getHeadingRadians();
		double enemyVelocity = e.getVelocity();
		double enemyDistance = e.getDistance();
		double enemyX = myX + Math.sin(absoluteBearing) * enemyDistance;
		double enemyY = myY + Math.cos(absoluteBearing) * enemyDistance;
		double enemyEnergy = e.getEnergy();
		int known = 0;
		for(int i = 0; i < enemies; i++) {
			if((enemy[i].name.equals(e.getName()))){
				if(known == 0){
					nameOfPrey = e.getName();
					enemy[i].headingChange = -Utils.normalRelativeAngle(enemy[i].heading - enemyHeading)/(gameTime - enemy[i].lastScannedTime);
        			enemy[i].x = enemyX; 
       				enemy[i].y = enemyY; 			
					enemy[i].energy = enemyEnergy;
					enemy[i].heading = enemyHeading;
					enemy[i].velocity = enemyVelocity;
					enemy[i].distance = enemyDistance;
					enemy[i].absoluteBearing = absoluteBearing;
					enemy[i].active = 2;
					enemy[i].lastScannedTime = gameTime;
					known = 1;
				}
			}
			if(known == 0){
				if(enemy[i].active == 1){
					nameOfPrey = e.getName();
					enemy[i].headingChange = 0;
					enemy[i].name = e.getName();
					enemy[i].x = enemyX; 
       				enemy[i].y = enemyY; 			
					enemy[i].energy = enemyEnergy;
					enemy[i].heading = enemyHeading;
					enemy[i].velocity = enemyVelocity;
					enemy[i].distance = enemyDistance;
					enemy[i].absoluteBearing = absoluteBearing;
					enemy[i].active = 2;
					enemy[i].lastScannedTime = gameTime;
					known = 1;
				}
			}
		}
		setVariables();
		if(gameTime > 10){
			if(currentEnemies > 1){doMeleeRadar();}else{
				double radarTurn = Utils.normalRelativeAngle(absoluteBearing - getRadarHeadingRadians());
				radarTurn += findCharge(radarTurn) * RADAR_OFFSET;
				setTurnRadarRightRadians(radarTurn);
			}

		}
	doGun();
                        
	if(e.getEnergy() <= (ourHealth) && getOthers() < 2) {

		//if(theShotPower == Double.POSITIVE_INFINITY) theShotPower = 3;
		//setTurnRight(targetBearing);
                //setAhead(600);
		/////doMovement();
		BubbleMove();
		//nextLong();
            }

    if (enemiesLeft == 1 && theirLife > ourHealth && minRisk == true) {
		doMovement();
		//out.println("doMovement");
		//BubbleMove();
		//nextLong();
	}
	if (enemiesLeft == 1 && theirLife > ourHealth && nextLong == true) {
		//doMovement();
		//BubbleMove();
		nextLong();
		//out.println("nextLong");
	}
	if (enemiesLeft == 1 && theirLife > ourHealth && bubbleMove == true) {
		//doMovement();
		BubbleMove();
		//out.println("bubbleMove");
		//nextLong();
	}

	if (enemiesLeft == 2 && theirLife <= ourHealth) {                 
		BubbleMove();
		//nextLong();
	}

	if (enemiesLeft == 2 && theirLife > ourHealth) {
		BubbleMove();
		//nextLong();
	}
	if (enemiesLeft >= 3) {
            cornerIfy();
		//BubbleMove();
		//nextLong();
	}
	
            

}
}

	public void doGun() {
		findTarget();
		double futureX = myX + Math.sin(myHeading) * myVelocity;
		double futureY = myY + Math.cos(myHeading) * myVelocity;
		double bulletPower = findBulletPower();
		double bulletSpeed = 20 - (bulletPower * 3);
		double predictedVelocity = enemy[target].velocity;
		double predictedChange = enemy[target].headingChange;
		predictedX = enemy[target].x;
		predictedY = enemy[target].y;
		double predictedHeading = enemy[target].heading;
		long time = enemy[target].lastScannedTime - gameTime;
		while(time++ * bulletSpeed < Point2D.Double.distance(myX, myY, predictedX, predictedY)){	
			predictedHeading += predictedChange;
			predictedX += Math.sin(predictedHeading) * predictedVelocity;	
			predictedY += Math.cos(predictedHeading) * predictedVelocity;
			if((predictedX < 16.0)||(predictedY < 16.0)||(predictedX > battleWidth - 16.0)||(predictedY > battleHeight -16.0)){		
				predictedX = Math.min(Math.max(18.0, predictedX), battleWidth - 18.0);	
				predictedY = Math.min(Math.max(18.0, predictedY), battleHeight - 18.0);
				double targetSpeed = Point2D.Double.distance(myX, myY, predictedX, predictedY) / (time - enemy[target].lastScannedTime + gameTime);
				if((targetSpeed < 20)&&(targetSpeed > 11)){
					bulletPower = (targetSpeed - 20) / (-3);
				}
				break;
			}
		}
		if((enemy[target].distance / bulletSpeed > circularLimit)){
			predictedX = enemy[target].x;
			predictedY = enemy[target].y;
		}
		double gunTurn = Utils.normalRelativeAngle(Math.atan2(predictedX - futureX, predictedY - futureY) - getGunHeadingRadians());
		setTurnGunRightRadians(gunTurn);
		if((Math.abs(gunTurn) < PI/9)&&(getGunHeat() == 0)&&(bulletPower >= 0)){
			fire(bulletPower);
		}
	}
    /*
     *\\\\\\\\\\\\\\\\\\\
     *MOVEMENT FUNCTIONS/
     *\\\\\\\\\\\\\\\\\\\
     */

    public void BubbleMove() {						


		
		/* if (getX()<50 && getY()<50) //am in bottom left corner
			{
			cornerNumber = 1;
			}
		else if (getX()>(this.getBattleFieldWidth()-60) && getY()<60) //am in bottom right corner
			{
			cornerNumber = 2;
			}
		else if (getX()>(getBattleFieldWidth()-60) && getY()>getBattleFieldHeight()-60) //am in top right corner
			{
			cornerNumber = 3;
			}
		else if (getX()<60 && getY()>getBattleFieldHeight()-60) //am in top left corner
			{
			cornerNumber = 4;
			}
		else {
			cornerNumber = 0;
		}
	
				double side = Math.round(Math.random() + 1);
	
	if(cornerNumber != 0) {
	
		if (cornerNumber == 1) {
			if(side == 1) {
				desiredX = battleWidth/2;
				desiredY = 100;
			}
			else if(side == 2){ 
				desiredX = 100;
				desiredY = battleHeight/2;
			}
		}
		
		else if(cornerNumber ==2) {
			if(side == 1) {
				desiredX = battleWidth/2;
				desiredY = 100;
			}
			else if(side == 2){ 
				desiredX = battleWidth - 100;
				desiredY = battleHeight/2;
			}
		}
		else if (cornerNumber == 3) {
			if(side == 1) {
				desiredX = battleWidth - 100;
				desiredY = battleHeight/2;
			}
			else if(side == 2){ 
				desiredX = battleWidth/2;
				desiredY = battleHeight - 100;
			}
		}
		else if (cornerNumber == 4) {
			if(side == 1) {
				desiredX = battleWidth/2;
				desiredY = battleHeight - 100;
			}
			else if(side == 2){ 
				desiredX = 100;
				desiredY = battleHeight/2;
			}
		}
		MoveToSpot();
		out.println(cornerNumber);
	}
		 */

        Random duck = new Random();	

        double duck2 = duck.nextDouble();
				
		double ourSpotX = (getX());

        double ourSpotY = (getY());			

		double xDif = spherePointX - ourSpotX;
		
		double yDif = spherePointY - ourSpotY;
		
		double xDifSq = xDif * xDif;
		
		double yDifSq = yDif * yDif;
		
		double distanceToPoint = Math.sqrt(xDifSq + yDifSq);
		
	if (distanceToPoint < 5 || closeToWall == true)  {
		
		double duck3 = duck.nextDouble();
		
		double bigSphereEdgeX = (ourSpotX + 200);
		
		double smallSphereEdgeX = (ourSpotX - 200);
		
		double bigSphereEdgeY = (ourSpotY + 200);
		
		double smallSphereEdgeY = (ourSpotY - 200);
		
		spherePointX = (duck3 * (bigSphereEdgeX - smallSphereEdgeX) + smallSphereEdgeX);
		
		spherePointY = (duck3 * (bigSphereEdgeY - smallSphereEdgeY) + smallSphereEdgeY);
		
	}
	
		//setTurnRight(targetBearing - 90);
		
	if (spherePointX < 50 || spherePointX > battleWidth - 50 || spherePointY < 50 || spherePointY > battleHeight - 50) {
			closeToWall = true;
	}
            
	else {
			closeToWall = false;
	}
		                
                desiredX = spherePointX;
                desiredY = spherePointY;
                
                MoveToSpot();
		
}
		
	
	
	public double nextLong() {
		
		Random duck = new Random();	

                double duck2 = duck.nextDouble();		
		
		double placeX = duck2*(battleWidth);

                double placeY = duck2*(battleHeight);

		double ourSpotX = (getX());

                double ourSpotY = (getY());

		double moveDist;

		if(getTime()%20 == 0) 
			moveDist = 100;
		
		else 
			moveDist = (ourSpotX - placeX);
		
	if (i == 0) {
            setBack(moveDist);
            i++;
	}
	else {
		setAhead(moveDist);
		if (i==3) {
		i = 0;
            }
	}
	
	if (h == 0) {
		setTurnRight(ourSpotY - placeY);
		h++;
	}
	if (h == 1) {
		setTurnLeft(ourSpotY - placeY);
		h = 0;
	}
		return duck2;
    }

	public void cornerIfy () {

		double moveTime;
		Random duck = new Random();
		double duck2 = duck.nextDouble();
			if (getOthers() > 5){
		moveTime = (25);
			}
			else {
		moveTime = 20;
			}
		 if (getX()<50 && getY()<50) //am in bottom left corner
			{
			cornerNumber = 1;
			}
		else if (getX()>(this.getBattleFieldWidth()-60) && getY()<60) //am in bottom right corner
			{
			cornerNumber = 2;
			}
		else if (getX()>(getBattleFieldWidth()-60) && getY()>getBattleFieldHeight()-60) //am in top right corner
			{
			cornerNumber = 3;
			}
		else if (getX()<60 && getY()>getBattleFieldHeight()-60) //am in top left corner
			{
			cornerNumber = 4;
			}
		if (cornerNumber==0)
			{
                                if(getX() <= getBattleFieldWidth()*.5 && getY() < getBattleFieldHeight()*.5) {
			desiredX = 25;                                  //go to bottom left
			desiredY = 25;
				}
				else if(getX() > getBattleFieldWidth()*.5 && getY() < getBattleFieldHeight()*.5) {
			desiredX = getBattleFieldWidth()-25;            //go to bottom right
			desiredY = 25;
				}
				else if(getX() <= getBattleFieldWidth()*.5 && getY() >= getBattleFieldHeight()*.5) {
			desiredX = 25;                                  //go to top left
			desiredY = getBattleFieldHeight()-25;
				}
				else if(getX() > getBattleFieldWidth()*.5 && getY() >= getBattleFieldHeight()*.5) {
			desiredX = getBattleFieldWidth()-25;            //go to top right
			desiredY = getBattleFieldHeight()-25;
				}
			}
		if (cornerNumber == 1) {
			if(i == 1){
				desiredX = 100;
				desiredY = 50;
				if(getTime()%moveTime == 0){
					i++;
				}
			}
			else {
				desiredX = 50;
				desiredY = 100;
				if(getTime()%moveTime == 0){
					i = 1;
				}
			}
			
		}
		else if (cornerNumber == 2) {
			if (i == 1) {
				desiredX = (getBattleFieldWidth() - 100);
				desiredY =50;
				if(getTime()%moveTime == 0){
					i++;
				}
			}
			else {
				desiredX = (getBattleFieldWidth() - 50);
				desiredY = 100;
				if(getTime()%moveTime == 0){
					i = 1;
				}
			}
		}
		else if (cornerNumber == 3) {
			if (i == 1) {
				desiredX = (getBattleFieldWidth() - 50);
				desiredY = (getBattleFieldHeight() - 100);
				if(getTime()%moveTime == 0){
					i++;
				}
			}
			else {
				desiredX = (getBattleFieldWidth() - 100);
				desiredY = (getBattleFieldHeight() - 50);
				if(getTime()%moveTime == 0){
					i = 1;
				}
			}
		}
		else if (cornerNumber == 4) {
			if (i==1) {
				desiredX = 100;
				desiredY = (this.getBattleFieldHeight() - 50);
				if(getTime()%moveTime == 0){
					i++;
				}
			}
			else {
				desiredX = 50;
				desiredY = (this.getBattleFieldHeight() - 100);
				if(getTime()%moveTime == 0) {
				i = 1;
				}
			}
		}
		MoveToSpot();												
	}
        
    void doMovement() {
			Random duck = new Random();
			double duck23 = duck.nextDouble();
			double fixedDuck = Math.round((duck23*15)+5); //keep it between 5 and 20
			if(fixedDuck < turns) turns = 0;
		
       if (turns == fixedDuck)  {
			direction *= -1;
			setAhead(direction*400);
			turns = 0;
		}
			setTurnRight(targetBearing - 90);
			turns++;
			
		if (turns > 20){
			turns = 0;
		}
		/*double travel;
		double goalrisk;
		double testrisk;
		double angle;
		Point2D testPoint = new Point2D.Double();

		myPosition.setLocation( getX(), getY());
		if (currentTarget != null)
		{
			// if enemy is close, keep moving
			travel = Math.max( 60, myPosition.distance(currentTarget) * 0.35);
			goalrisk = calcRisk( myGoalPosition, getHeadingRadians());			// Is my Goal-path still good enough??

			for (angle = 0; angle < Math.PI*2; angle += Math.PI/36) {
				testPoint.setLocation( myPosition.getX() + Math.sin( angle) * travel, myPosition.getY() + Math.cos( angle) * travel);
				if ((playField.contains( testPoint)) && ((testrisk = calcRisk( testPoint, angle)) < goalrisk )) {
					myGoalPosition.setLocation( testPoint);
					goalrisk = testrisk;
				}
			}

			// set the turn and the distance
			angle = Utils.normalRelativeAngle( Math.atan2( myPosition.getX() - myGoalPosition.getX(), myPosition.getY() - myGoalPosition.getY()) - getHeadingRadians());
			setTurnRightRadians( Math.atan( Math.tan( angle)));
			setMaxVelocity( 10 - (4 * Math.abs(getTurnRemainingRadians())));
			setAhead( (angle == Math.atan( Math.tan( angle)) ? -1.0 : 1.0) * travel);
//			out.println(angle+"  "+myGoalPosition);
		}	*/							
									
	}

    public void MoveToSpot() {
			
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
    
    /*private double calcRisk(Point2D point, double movang)
	{
		// copied the framework from Kawigi's Coriantumr

		double totrisk = 0;
		double botrisk;
		double botangle;
		Collection enemySet;
		Iterator it = (enemySet = enemies.values()).iterator();

		// stay away from centre and corners
		totrisk = (10 * (getOthers() - 1)) / point.distanceSq( centerPoint);
		totrisk += 3 / point.distanceSq( 0, 0);
		totrisk += 3 / point.distanceSq( battleWidth, 0);
		totrisk += 3 / point.distanceSq( 0, battleHeight);
		totrisk += 3 / point.distanceSq( battleWidth, battleHeight);

		do
		{
			EnemyInfo e = (EnemyInfo)it.next();
			if (e.teammate == false) {
				// calculate the perpendicularity (0 = head-on)
			    botangle = Utils.normalRelativeAngle( Math.atan2( e.getX() - point.getX(), e.getY() - point.getY()) - movang);
				// standard risk not energy-related
				botrisk = 100;
				// current target is probably more dangerous
				if (e == currentTarget) botrisk += 40;
				// if they have hit me recently, the threat is bigger
//				if (currTime - e.lastHit < 100) botrisk += 30;
				// add perpendicularity to the threat (a bit linear on angle seems better!)
				botrisk *= (1.0 + ((1 - (Math.abs(Math.sin(botangle)))) + Math.abs(Math.cos(botangle))) / 2);
			}
			else {
				// to avoid collisions
				botrisk = 10;
			}

			// TODO:
			// try to never be the closest to anyone

			// all risks from any bot is distance related
			totrisk += (botrisk /point.distanceSq( e));
		}
		while (it.hasNext());
		// and finally make a random threat on my current location
		totrisk += (Math.random() * 0.5) / point.distanceSq( myPosition);
		return totrisk;												
	}					*/							
    
    /*/////
    //Unused movements
    *//////
    
    
    
	void doRam(double targetDistance, double targetBearing) {

		Random num = new Random();
		double adnum = (num.nextDouble() * 25 + 25);
		double TurnAmount = (targetBearing - 55 );
		TurnAmount += num.nextGaussian()/6;
				
			if (targetDistance > 200 || true) {
				setAhead(adnum);
				execute();
				setTurnRight(TurnAmount * direction);
				out.println(TurnAmount);
			}
			else if (targetDistance <= 200 && false) {
				setTurnRight(targetBearing);
				setAhead(600);
				execute();
			}
			if(getTime()%30 == 0) {
				direction = direction * -1;
			}
		}
   public void clingToWalls() {
		
		if (getX()<50 && getY()<50) //am in bottom left corner
			{
			cornerNumber = 1;
			}
		else if (getX()>(this.getBattleFieldWidth()-50) && getY()<50) //am in bottom right corner
			{
			cornerNumber = 2;
			}
		else if (getX()>(getBattleFieldWidth()-50) && getY()>getBattleFieldHeight()-50) //am in top right corner
			{
			cornerNumber = 3;
			}
		else if (getX()<25 && getY()>getBattleFieldHeight()-50) //am in top left corner
			{
			cornerNumber = 0;
			}
		
		if (cornerNumber==0)
			{
			desiredX = 25;	//go to bottom left
			desiredY = 25;
			}
		else if (cornerNumber==1)
			{
			desiredX = getBattleFieldWidth()-25;	//go to bottom right
			desiredY = 25;
			}
		else if (cornerNumber==2)
			{
			desiredX = getBattleFieldWidth()-25;	//go to top right
			desiredY = getBattleFieldHeight()-25;
			}
		else if (cornerNumber==3)
			{
			desiredX = 25;	//go to top left
			desiredY = getBattleFieldHeight()-25;
		}
		MoveToSpot();
        }

        //-----------------------------
        //------EVENT HANDLERS---------
        //-----------------------------
        
     public void onWin(WinEvent e) {
         
            double Accuracy = Math.round ((hits/shots) * 100);
            out.println("I have succeeded.");
            out.println("Shots:    " + shots);                //Stats
            out.println("Hits:     " + hits);
            out.println("Misses:   " + misses);
            out.println("Bullets hit Bullets: " + bulletHitBullet);
            out.println("Accuracy: " + Accuracy);
            ahead(0);
            wins++;
            
            setTurnRadarRight(30000 * direction);
            setTurnGunLeft(30000 * direction);
            setTurnRight(30000 * direction);
            if(getTime()%5 == 0) direction *= -1;
            
            if(getOthers() == 0) {
				if(weHadTeammates) out.println("My teammates have perished.");
                for(int i = 0; i < 500; i ++) {
                    fire(.1);
                }
            }
            
            else {
                out.println(getOthers() + " teammates survived.");
            }
            
	}
        public void onDeath(DeathEvent e) {
            double Accuracy = Math.round ((hits/shots) * 100);
			if(getRoundNum() + 1 == getNumRounds()) {
            out.println("My time here is over");
			}
			else {
			out.println("We'll settle this next round");
			}
			out.println(getRoundNum() + 1);
            out.println("Shots:  " + shots);
            out.println("Hits:   " + hits);
            out.println("Misses: " + misses);
            out.println("Bullets hit Bullets: " + bulletHitBullet);
            out.println("Accuracy: " + Accuracy);
			losses++;
			
				if(minRisk){ 
				minRisk = false;
				bubbleMove = true;
				}
				else if(bubbleMove){
				bubbleMove =  false;
				nextLong = true;
				}
				else if(nextLong){
				nextLong = false;
				minRisk = true;
				}
				out.println("bm: " + bubbleMove + "\n nL: " + nextLong + "\n mr: " + minRisk);
        }
   public void onRobotDeath(RobotDeathEvent e) {
            /*if (nameOfPrey.equals(e.getName())) {
            	out.println("I have smitten another.");
		}
            else {
		out.println("I'm sorry... someone else got to " + e.getName() + " first.");
		}
           /* if (Enemies.remove(e.getName()) == currentTarget) {
			currentTarget = null;
			setTurnRadarRight( Double.POSITIVE_INFINITY);
            }*/
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
        
        public void onHitRobot(HitRobotEvent e) {
             setTurnRadarRightRadians(Utils.normalRelativeAngle(getHeadingRadians() + e.getBearingRadians() - getRadarHeadingRadians()) * 2);
        }
        
        public void onBulletHit(BulletHitEvent e) {
            shots++; 
            hits++;
        }
        
        public void onBulletHitBullet(BulletHitBulletEvent e) {
            shots++;
            bulletHitBullet++;
        }
        
        public void onBulletMissed(BulletMissedEvent e) {
            shots++;
            misses++;
        }
        
        public void onHitWall(HitWallEvent e) {	
            if (moveDirection == (MOVE_FORWARD)) {
		moveDirection = MOVE_BACKWARD;
            }
            else {
		moveDirection = MOVE_FORWARD;
            }
    }

	public void setVariables(){
		myX = getX();
		myY = getY();
		myHeading = getHeadingRadians();
		myVelocity = getVelocity();
		myEnergy = getEnergy();
		currentEnemies = getOthers();
		double totalEnergy = 0;
		for(int i = 0; i < enemies; i++) {
			if(enemy[i].active == 2){
				enemy[i].distance = Point2D.Double.distance(myX, myY, enemy[i].x, enemy[i].y);
				enemy[i].absoluteBearing = Utils.normalAbsoluteAngle(Math.atan2(enemy[i].x - myX, enemy[i].y - myY));
				totalEnergy += enemy[i].energy;
			}
		}
		averageEnergy = totalEnergy / currentEnemies;
	}
	double findCharge(double v){
		return v > 0 ? 1 : -1;
	}

        //-----------------------------
        //-----UTILITY FUNCTIONS-------
        //-----------------------------
		
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
		
	double findBulletPower(){
		double bulletPower = (getOthers() == 1 ? 600 : 900) / enemy[target].distance;
		bulletPower = Math.max(Math.min(bulletPower, 0.2 + (enemy[target].energy / 4)), 0.1);
		bulletPower = Math.min(Math.min(bulletPower, (myEnergy - 3) / 4), 3);
		if(currentEnemies == 1){
			if(myEnergy > enemy[target].energy){bulletPower = Math.min(bulletPower, myEnergy - enemy[target].energy - 0.3);}
		}
		return bulletPower;
	}
	
	public void doMeleeRadar(){
		long lowestScanTime = gameTime * 2;
		int newScanTarget = 0;
		int locking = 0;
		for(int i = 0; i < enemies; i++) {
			if((enemy[i].active == 2)&&(enemy[i].lastScannedTime < lowestScanTime)){
				lowestScanTime = enemy[i].lastScannedTime;
				newScanTarget = i;
			}
		}
		int timeUntilScanTarget = (int)(Math.ceil(Math.abs(Utils.normalRelativeAngle(enemy[target].absoluteBearing - getRadarHeadingRadians())))/(PI/4)) + 1;
		if((timeUntilScanTarget >= getGunHeat()*10)){
			newScanTarget = target;
			locking = 1;
		}
		if(newScanTarget != currentScanTarget){
			setTurnRadarRightRadians(Double.POSITIVE_INFINITY*findCharge(Utils.normalRelativeAngle(enemy[newScanTarget].absoluteBearing - getRadarHeadingRadians())));
			currentScanTarget = newScanTarget;
		}
		if((locking == 1) && (gameTime - enemy[target].lastScannedTime < 1)){
			double radarTurn = Utils.normalRelativeAngle(enemy[target].absoluteBearing - getRadarHeadingRadians());
			radarTurn += findCharge(radarTurn) * RADAR_OFFSET;
			setTurnRadarRightRadians(radarTurn);
		}
	}
	
	public void findTarget(){
		double bestScore = Double.POSITIVE_INFINITY;
		for(int i = 0; i < enemies; i++){
			if(enemy[i].active == 2){
				double score = enemy[i].distance * (i == target ? Math.min(0.7, getGunHeat()) : 1) * (enemy[i].energy > 20 ? 1 : 0.85) * (enemy[i].energy/4 > Math.min(900/enemy[i].distance, (myEnergy - 3) / 4) ? 1 : 0.75 * (currentEnemies == 3 ? Math.pow(enemy[i].energy, 0.5) : 1));
				if(score < bestScore){
					target = i;
					bestScore = score;
				}
			}
		}
	}
			
}

    class Target 
{
	double health;
	double robotHeading;
	double distance;
	double X;
	double Y;
	double V;
}
    
    class EnemyInfo extends Point2D.Double
{
	double attrac;
	boolean teammate;
}

	class enemy{
		String name;
		int active;
		long lastScannedTime;
		double x,y;
		double distance;
		double energy;
		double velocity;
		double heading;
		double headingChange;
		double absoluteBearing;
		double distanceWeight;
		double parallelWeight;
	}
    
    
    
    
    
    
    
    
    
    
    
    