package test;
import robocode.*;
import java.awt.geom.*;
import robocode.util.Utils;
import java.awt.Color;
import java.util.Random;


// Editor's note: I believe the original code sources circa 2007

/**
  * Podgy - a robot by Damij
**/

//PODGY - plumpish or fat

public class Podgy extends AdvancedRobot
{
//This is how Podgy started out, with this line.
//static double gunPlus = Math.random() * 360;
BinSetter bin = new BinSetter();
Enemy enemy = new Enemy();
ShotTracker vshot = new ShotTracker();
static double gunPlus = 0;
double gunPoints;
double bulletsFired;
double futureChangeAmount = 360;
static double startingX;
static double startingY;
static double x;
static double y;
double lastX;
double lastY;
static double theirDirection;
static double distance;
static int direction = 1;
static double countdown;
static double bulletSpeed;
public static long nextTimeNeededToHit;
static long currTime;
static long nextTimeToHit;
static double distanceRatio;
static double theirDistance;
static double timeOfShot;
static double timeOfHit;
static double countDownTillHit;
static boolean okToFire;
static boolean fired;
static int binSet;
static double heading;
static double gunHeadR;
static double ourX;
static double ourY;
static double PossibleMovementDist;
static double speed;
double scans;
double absoluteBearing;
double bearingFromGun;
static double timesTurned;
double origPlus;
static double bPower = 3;
static double timeTillHit;
static boolean track = true;
boolean collectInfo;
double tHealth = 100;
static boolean PodgeGun;
static boolean ParGun;
static Point2D.Double p;
static double radarOffset;
static double nextTimeToHitPar;
static double nextTimeToHitPodge;
static double nextTimeNeededToHitPar = 150;
static double nextTimeNeededToHitPodge = 150;
static double countdownPar;
static double countdownPodge;
static int timesSetPar;
static int timesSetPodge;
static boolean vShotOk;
static Point2D.Double par = new Point2D.Double();
static Point2D.Double podge = new Point2D.Double();
static double PodgeHits;
static double ParHits;
static double gunUse;
static double lastGun;
static double gunHeat;
static double bHeight;
static double bWidth;
static double priorX;
static double priorY;
static int currentDirectionX;
static int currentDirectionY;
static int dAtShotX;
static int dAtShotY;
static double trackedDist;
static double desiredDist;
static double startX;
static double startY;
static double ourDirection = 1;
static double PossibleMove;
static int ourRunningDirection;
static int startingDirection;
static double prevX;
static double prevY;
static double prevT;
static int d = 1;
static int prevD = 1;
static double headingPlus;
static int timesOrientated;
static double ourHeading;
static int dAtShot = 1;
static int desiredMultiple;

static boolean movingX = true;
static boolean movingY = false;
static boolean wallTurnOK = true;
static boolean cornerTurnOK = true;
int reversed = 1;
int i = 1;

static double Bin;

public RoundRectangle2D.Double playField;
public Rectangle2D.Double trCorner,tlCorner, brCorner, blCorner;

Random duck = new Random();

private int timeSinceTurn;

private final int SIGHT_DELAY = 6;

	/**
	 * run: Podgy's default behavior
	 */
	public void run() {
		//setColors(new Color(132, 228, 238), new Color(132, 228, 238), new Color(132, 228, 238));
		/*setBodyColor(Color.lightBlue);
		setGunColor(Color.lightBlue);
		setRadarColor(Color.blue);
		setScanColor(Color.pink);*/
		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);
		okToFire = true;
		turnRadarRight(360);
		if(gunUse == 0) out.println("Starting round with Podgy's gun");
		if(gunUse == 1) out.println("Starting round with Parametric gun");
		bHeight = getBattleFieldHeight();
		bWidth = getBattleFieldWidth();
		playField = new RoundRectangle2D.Double( 18, 18,
	    bWidth - 36, bHeight - 36, 18, 18);
		blCorner = new Rectangle2D.Double(0, 0, 72, 72);
		brCorner = new Rectangle2D.Double(bWidth, 0, 72, 72);
		tlCorner = new Rectangle2D.Double(0, bHeight, 72, 72);
		trCorner = new Rectangle2D.Double(bWidth, bHeight, 72, 72);
		//Initialising the virtual shots so they don't spew out points before the round starts.
		nextTimeToHitPodge = 50;
		nextTimeToHitPar = 10;
		enemy.timeOfHit = 0;
		priorY = getY();
		priorX = getX();
		prevX = priorX;
		prevY = priorY;
		orientate();
		while(true) {
			
			setTurnRadarRight(Double.POSITIVE_INFINITY);
			//Try not to collect misleading information, so only do it during battaaallll
			if(tHealth >=.1&&getEnergy()>=.1)
			collectInfo = true;
			else{
			collectInfo = false;}
			enemy.currTime = getTime();
			ourX = getX();
			ourY = getY();
			doRadar();
			countdown = enemy.timeOfHit - getTime();
			enemy.ourX = getX();
            enemy.ourY = getY();
			currentDirectionX = getDirectionX();
			currentDirectionY = getDirectionY();
			countdownPodge = nextTimeToHitPodge - getTime();
			countdownPar = nextTimeToHitPar - getTime();
			countDownTillHit = timeOfHit - getTime();
			ourHeading = getHeading();
			gunHeat = getGunHeat();
			if(super.getTime()%SIGHT_DELAY==0) {
				doMovement();
			}
			doGun();
			vshot.updateShots();
			PrintGunUse();
			//out.println(binSet);
			//out.println(okToFire);
			//out.println(gunPlus);
			execute();
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		double headingPlus = 15;
		scans++;
		x = getX() + (Math.sin(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
		y = getY() + (Math.cos(getHeadingRadians() + e.getBearingRadians()) * e.getDistance());
		enemy.x = x;
		enemy.y = y;
		theirDistance = getrange(getX(), getY(), x, y);
		speed = e.getVelocity();
		tHealth = e.getEnergy();
		double h = NormaliseBearing(e.getHeadingRadians() - heading);
		h = h/(getTime() - enemy.ctime);
		enemy.changehead = h;
		heading = e.getHeadingRadians();
		enemy.health = e.getEnergy();
		enemy.distance = e.getDistance();
		absoluteBearing = getHeading() + e.getBearing();
		enemy.bearing = e.getBearing();
		bearingFromGun = normalRelativeAngle(absoluteBearing - getGunHeading() + gunPlus);
		double slope = (lastY-y)/(lastX-x);
		if(slope>0 && (getHeading() < 90 | (getHeading() >= 180 & getHeading() < 270)))
		theirDirection = 1;
		if(slope>=0 && ((getHeading() >= 90 & getHeading() < 180) | (getHeading() >= 270)))
		theirDirection = -1;
		lastY = y;
		lastX = x;
		enemy.ctime = getTime();				//game time at which this scan was produced
		enemy.update();
		ourRunningDirection = getSolidDirection();
		//setTurnRight(90+enemy.bearing);


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
		
		if(enemy.shot){
				if(countdown<=0){
				timeOfShot = getTime();
				enemy.shotHeading = getHeading();
				bulletSpeed = 20 - (3 * (enemy.healthDif * -1));
				timeTillHit = enemy.distance/bulletSpeed;
				timeOfHit = (timeOfShot + timeTillHit) - 8;
				dAtShot = ourRunningDirection;
				//System.out.println("distance " + enemy.distance + "   bSpeed " + bulletSpeed + "   time " + timeOfShot + "\nTime of Hit: " + timeOfHit + "   Time " + getTime());
				enemy.update();
				}
		}
		//Turn straight towards him, without spinning in circles.
		//The headingPlus however, I don't think has an effect like it did in Chicken, who it came from, so it's there for looks. Enjoy it.
		//if(e.getBearing()>0)
		//setTurnRight((e.getBearing() - 90) + headingPlus);
		//else if(e.getBearing()<=0)
		//setTurnRight((e.getBearing() + 90) - headingPlus);
		
		double dist = enemy.distance/(20-bPower);
		double timeOfHit = (timeOfShot + timeTillHit);
		double countdown = timeOfHit - getTime();
		
		//if (getDistanceRemaining() == 0){
			//This is certainly amoung the top three craziest lines of code I've written, so I decided to keep it.. even though it no longer does anything
			//double aheadAmount = Math.cos((Math.random()-.5)*countdown*1.5+((Math.random()-.2))*((Math.sin(enemy.distance)*(enemy.distance/(countdown/(8*bPower))))))*150;
			//setAhead(aheadAmount);
		//}

		//execute();
	}
	void doRadar() {
		if(getTime() - enemy.ctime > 4){
			radarOffset = 4*Math.PI;
		}
		else{
		radarOffset = getRadarHeadingRadians() - (Math.PI/2 - Math.atan2(enemy.y - getY(),enemy.x - getX())); 
		radarOffset = 1.7d*NormaliseBearing(radarOffset);
		/*if (radarOffset < 0)
			radarOffset -= Math.PI/10;
		else {
			radarOffset += Math.PI/10;
		}	*/
		}
		setTurnRadarLeftRadians(radarOffset);
	}
	public static int getDirectionX(){
		if(x - priorX >= 0){
			priorX = x;return 1;}
		else{
			priorX = x;return -1;
		}
	}
	public static int getDirectionY(){
		if(y - priorY >= 0){
			priorY = y;return 1;}
		else{
			priorY = y;return -1;
		}
	}
	public int getSolidDirection(){
		if(getTime()!=prevT){
			if(movingX){
				if(ourX-prevX>=0)
					d = 1;
				else if(ourX-prevX<0)
					d = -1;
			}
			else if(movingY){
				if(ourY-prevY>=0)
					d = 1;
				else if(ourY-prevY<0)
					d = -1;
			}
		}
		else{
			d = prevD;
		}
		prevD = d;
		prevX = ourY;
		prevY = ourY;
		return d;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	public void doMovement(){
		
		Point2D.Double a = new Point2D.Double();
		double heading = getHeading();
		
		timeSinceTurn ++;
		
		a = getDesiredBin();
		checkAngle();
		
		if(movingX) {
			desiredMultiple = ourX - a.x >= 0 ? -1 : 1;//getHeadingRadians() <= Math.PI ? -1 : 1 : getHeadingRadians() <= Math.PI ? 1 : -1;
		} else if(movingY) {
			desiredMultiple = ourY - a.y >= 0 ? -1 : 1;// *(int)ourDirection;//(getHeadingRadians() <= Math.PI / 2 || getHeadingRadians() >= 3*Math.PI/2) ? -1 : 1 : (getHeadingRadians() <= Math.PI / 2 || getHeadingRadians() >= 3*Math.PI/2) ? 1 : -1;
		}

		if(enemy.shot){
			PossibleMove = (enemy.distance/(20-3*enemy.shotPower))*8;
			startX = getX();
			startY = getY();
			startingDirection = ourRunningDirection;
		}
		
		if(startingDirection==0)
		startingDirection = ourRunningDirection;
		if(movingX)
		trackedDist = (getX() - startX);// * desiredMultiple * (enemy.bearing >= 0 ? 1 : -1);
		else if(movingY)
		trackedDist = (getY() - startY);// * desiredMultiple*  (enemy.bearing >= 0 ? 1 : -1);

		moveToDesiredBin(a);
		//execute();
	}
	
	public Point2D.Double getDesiredBin(){
		if(countdown<=0){
			Bin = bin.findLowestBin();
			//System.out.println(Bin);
		}
		
		return new Point2D.Double(Bin,10);
	}
	
	public void moveToDesiredBin(Point2D.Double binNumber){
		boolean go;
		desiredDist = PossibleMove*(binNumber.x/10);
		go = checkPoint();
		setMaxVelocity(8/(getTurnRemainingRadians()*3));
			//System.out.println(desiredDist + " " + PossibleMove);
			//out.println(reversed);
			//out.println(" " + enemy.bearing);
			//System.out.println(enemy.shot/* + "\ncD " + countdown*/);
		
		if(go&&getDistanceRemaining()<=12/*&&desiredMultiple==-1*/){
			//if(i == 1){
				setAhead((desiredDist - trackedDist));
			//	i = 2;}
			//else if(i == 2){
			//	setBack(desiredDist - trackedDist);
			//	i = 1;}
		}
		else if(!go) {/*if(getDistanceRemaining()<=12){*/
			setBack(desiredDist - trackedDist);
		}
		//execute();
	}
	
	public void addToBin(int directionChange){
		double percent;
		percent = Bin*10;
		//out.println(" " + percent);
		bin.AddToBin(percent, directionChange);
	}
	
	public boolean checkPoint(){
		boolean goAble = true;
		double newDist = 100;
		double nextP = 4;
		Point2D.Double p = new Point2D.Double();
		p = getNextPoint();
		if(playField.contains(p.x, p.y)&&(checkIfCorner(p.x, p.y)==false)){
			cornerTurnOK = blCorner.contains(p.x,p.y) || trCorner.contains(p.x,p.y) || brCorner.contains(p.x,p.y) || tlCorner.contains(p.x,p.y);
			wallTurnOK = !cornerTurnOK;
		}
		double ourDist = getrange(ourX, ourY, x, y);
		if(movingX){
			nextP = p.x;
			newDist = getrange(nextP-(5*ourRunningDirection), ourY, x, y);
			if(!playField.contains(nextP, ourY)){
			boolean cornerTurn = checkIfCorner(nextP, ourY);
			goAble = decideTurn(cornerTurn);
		}
		}
		else if(movingY){
			nextP = p.y;
			newDist = getrange(ourX, nextP-(5*ourRunningDirection), x, y);
			if(!playField.contains(ourX, nextP)){
			boolean cornerTurn = checkIfCorner(ourX, nextP);
			goAble = decideTurn(cornerTurn);
		}
		}
		if(newDist<=75&&ourDist>=newDist){
		reverse(); goAble = true;}

		//execute();
		
		return goAble;
	}
	
	public boolean decideTurn(boolean cornerTurn){
		if(wallTurnOK){
			if(!cornerTurn)
				turnTowardsThem();
			wallTurnOK = false;
			//cornerTurnOK = false;
			return true;
		}
		if(cornerTurnOK){
			if(cornerTurn){
				//turnAwayFromThem();
				turnTowardsThem();
			}
			//wallTurnOK = false;
			cornerTurnOK = false;
			return true;
		}
		return true;
	}
	
	public Point2D.Double getNextPoint(){
		if(movingX)
		return new Point2D.Double(ourX+(50*ourRunningDirection), ourY);
		else if(movingY)
		return new Point2D.Double(ourX, ourY+(50*ourRunningDirection));
		
		return new Point2D.Double(25, 25);
	}
	
	public void turnAwayFromThem(){
		if(getTurnRemaining()==0){
		if(enemy.bearing>=0) {
			//	(enemy.bearing <= Math.PI / 2d || enemy.bearing <= - Math.PI / 2d)) {
			setTurnLeft(90);
		}
		else{
			setTurnRight(90);
		}
		Switch();
		}
	}
	public void turnTowardsThem(){
		if(getTurnRemaining()==0){
		if(enemy.bearing*ourDirection>=0) {
				//(enemy.bearing <= Math.PI / 2d || enemy.bearing <= - Math.PI / 2d)) {
			setTurnRight(90);
		}
		else{
			setTurnLeft(90);
		}
		Switch();
		}
	}
	
	public void checkAngle(){
		if(timesOrientated==1){
		int direction;
		boolean turn = true;
		double aX = 0;
		double aY = 0;
		if(movingX){
			aX = absoVal(90-absoVal(enemy.bearing));
			aY = absoVal(90-(Math.max(absoVal(enemy.bearing), 90) - Math.min(absoVal(enemy.bearing), 90)));
		}
		else if(movingY){
		
			aX = absoVal(90-(Math.max(absoVal(enemy.bearing), 90) - Math.min(absoVal(enemy.bearing), 90)));
			aY = absoVal(90-absoVal(enemy.bearing));
		}
		//System.out.println(aX + ", " + aY);
		//System.out.println(enemy.bearing);
		double minAngle = Math.min(aX, aY);
		if(movingX){
			if(minAngle == aX)
				return;
		}
		else if(movingY){
			if(minAngle == aY)
				return;
		}
		if(getTurnRemaining()==0 && timeSinceTurn % 100  == 0)
		//out.println("Turning " + aX + " " + aY);
			turnTowardsThem();
	}
	}
	
	public boolean checkIfCorner(double px, double py){
		//if(blCorner.contains(enemy.ourX, enemy.ourY))
			//out.println("We are in Corner 1");
		//if(brCorner.contains(enemy.ourX, enemy.ourY))
			//out.println("C 2");
		//if(tlCorner.contains(enemy.ourX, enemy.ourY))
			//out.println("C 3");
		//if(trCorner.contains(enemy.ourX, enemy.ourY))
			//out.println("C 4");
		//if(blCorner.contains(px, py)||brCorner.contains(px, py)||tlCorner.contains(px, py)||trCorner.contains(px, py)){
			//out.println("Turned for to cornerzors");
		//	return true;
		//}
		///return false;
		return (blCorner.contains(px, py)||brCorner.contains(px, py)||tlCorner.contains(px, py)||trCorner.contains(px, py));
	}
	
	public static void Switch(){
		if(movingX){
			movingX = false;
			movingY = true;
		}
		else if(movingY){
			movingY = false;
			movingX = true;
		}
	}
	
	public void orientate(){
		if(timesOrientated<1){
			movingX = true;
			movingY = false;
			if(getHeading()<=180)
			setTurnLeft(getHeading()-90);
			if(getHeading()>180)
			setTurnRight(270-getHeading());
			timesOrientated++;
			//execute();
		}
	}
	
	public double absoVal(double fixNum){
        if(fixNum>=0)
            return fixNum;
        if(fixNum <=0)
            return (fixNum*-1);
        return fixNum;
    }
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	public void doGun(){
		boolean fire = false;
		double toGunTurn = 0;
		selectGun();
		PodgeGun = false;
		ParGun = true;
			if(PodgeGun) toGunTurn = doPodgyGunGun();
			if(ParGun) toGunTurn = doParGun();
			if(track) resetShot();
		setTurnGunLeftRadians(toGunTurn);
		fire = vshot.trackShots();
		if(getGunHeat()==0&&getEnergy()-3>.1){
			setFire(3);
			dAtShotX = currentDirectionX;
			dAtShotY = currentDirectionY;
		}
		//execute();
	}
	/////////////////////////////////////////
	public void selectGun(){
		//Simple hit amount comparison.
		if(PodgeHits>=ParHits){
			PodgeGun = true;
			ParGun = false;
		}
		if(PodgeHits<ParHits){
			ParGun = true;
			PodgeGun = false;
		}
	}
	/////////////////////////////////////////
	public double doPodgyGunGun(){
		//Self explanitory if you follow where it goes.
		double toGunTurn = 0;
		gunUse = 0;
			countDownTillHit = Math.round(timeOfHit - getTime());
				if(countDownTillHit <= 0){
					track = true;
					if(collectInfo)
					//This one collects information and returns and angle...
				gunPlus = bin.Binner();
			}
			//whereas this one just returns the angle.
			gunPlus = bin.updateAngle();
		return toGunTurn = (NormaliseBearing(gunPlus));
	}
	
	public double doParGun(){
		double toGunTurn = 0;
		gunUse = 1;
			p = new Point2D.Double(enemy.x, enemy.y);
       		nextTimeNeededToHit = (int)Math.round((getrange(getX(), getY(), x, y)/(20-(3*3))));
			nextTimeToHit = nextTimeNeededToHit + getTime();
        	p = enemy.guessPosition(nextTimeToHit, true, false);
				countDownTillHit = Math.round(timeOfHit - getTime());
				//Even if we're not using Podgy's gun, we need to collect info for it.
					if(countDownTillHit <= 0 && getGunHeat()==0){
						track = true;
						if(collectInfo)
					gunPlus = bin.Binner();
				}
				////////////////////////////////
		double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(p.y - getY(),p.x -  getX()));
			if(p.x!=0)				//If we didn't just reset the grid (more on that in the enemy class)
		toGunTurn = (NormaliseBearing(gunOffset));
		return toGunTurn;
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
	public void resetShot(){
		//Starter information for Podgy's gun, that we want to time with the shots.
		track = false;
		startingX = Podgy.x;
		startingY = Podgy.y;
		timeOfShot = enemy.currTime;
		timeOfHit = (countDownTillHit = (getrange(enemy.x, enemy.y, ourX, ourY))/(20-(3*3))) + timeOfShot;
		fired = true;
		okToFire = false;
		timesTurned = 0;
		binSet = 0;
		PossibleMovementDist = (theirDistance/(20-(3*3)))*8;
	}
	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		bPower = e.getPower();
		//System.out.println("tD: " + trackedDist +"\nsD " + startingDirection);
		if(trackedDist<=0&&startingDirection==1)
		addToBin(-1);
		else if(trackedDist<=0&&startingDirection==-1)
		addToBin(1);
		else if(trackedDist>=0&&startingDirection==1)
		addToBin(1);
		else if(trackedDist>=0&&startingDirection==-1)
		addToBin(-1);
	}
	
	public void onHitWall(HitWallEvent e){
		//reverse();
	}
	
	public void reverse(){
		ourDirection *= -1;
		if(reversed==1)
		reversed=-1;
		else if(reversed==-1)
		reversed = 1;
	}
	
	public void onBulletHit(BulletHitEvent e){
		double poop;
		//out.println("wheet!");
		gunPoints++;
		futureChangeAmount = (bulletsFired - gunPoints)*(360);
		poop = getrange(x, y, getX(), getY());
		distance = Math.sin(degToRad(gunPlus)) * poop;
		distanceRatio = gunPlus/distance;
		//Not too terribly sure what that did or why I made it... 
		//Oh no - wait - hm... I think I used it in infant Podgy for something...
	}
	
	public void onWin(WinEvent e){
		//out.println(getRoundNum());
		setAdjustRadarForGunTurn(false);
		setAdjustGunForRobotTurn(false);
		setAdjustRadarForRobotTurn(false);
		out.println("Accuracy update");
		out.println("Parametric gun : " + ParHits);
		out.println("Podgy's own    : " + PodgeHits);
		resetEnd();
		for(int i = 0; i < Double.POSITIVE_INFINITY; i++)
		{
			//Maxx Rektum has to defeatzor nooooobsss
			ROBOTDANCE();
		}
	}
	
	public void onDeath(DeathEvent e){
		//Don't do anything anymore, but played a starring role in an infant Podgy.
		double changeAmount = Math.random()*futureChangeAmount;
		gunPlus+=changeAmount;
		////////////////////////
		out.println("Accuracy update");
		out.println("Parametric gun : " + ParHits);
		out.println("Podgy's own    : " + PodgeHits);
		resetEnd();
	}
	
	public void resetEnd(){
		timesSetPodge = 0;
		timesSetPar = 0;
		track = true;
		nextTimeToHitPar = 0;
		nextTimeToHitPodge = 0;
		timesOrientated = 0;
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
	
	public static double getrange( double x1,double y1, double x2,double y2 )
	{
		double xo = x2-x1;
		double yo = y2-y1;
		double h = Math.sqrt( xo*xo + yo*yo );
		return h;	
	}

	//I was tired of getting stuck with radians when I want degrees, or the other way around...
	public static double degToRad(double angle){
		double RadAngle;
		RadAngle = 2*Math.PI*(angle/360);
		return RadAngle;
	}
	//so I made these two conversion methods.
	public static double radToDeg(double angle){
		double degAngle;
		degAngle = 360*(angle/(2*Math.PI));
		return degAngle;
	}
	//And then promptly found out that Math already has them.
	
	double NormaliseBearing(double ang) {
		if (ang > Math.PI)
			ang -= 2*Math.PI;
		if (ang < -Math.PI)
			ang += 2*Math.PI;
		return ang;
	}
	
	void PrintGunUse(){
		if(lastGun!=gunUse){
			if(gunUse == 0)
				out.println("Switching to Podgy's gun");
			else if(gunUse == 1)
				out.println("Switching to Parametric gun");
		}
		lastGun = gunUse;
	}
	
	void ROBOTDANCE(){
		turnGunLeft(50);
			turnRadarRight(50);
				turnLeft(90);
		setTurnRadarRight(360);
			turnRight(5);
				turnLeft(5);
			turnRight(90);
				setTurnRadarLeft(360);
		turnLeft(5);
				setTurnGunLeft(30);
			turnRight(5);
				turnGunRight(50);
		turnGunLeft(5);
			setTurnLeft(30);
				turnGunRight(5);
	}
	
}
class BinSetter {
	//Gun Bins
	static double binx0;
	static double binx1;
	static double binx2;
	static double binx3;
	static double binx4;
	static double binx5;
	static double binx6;
	static double binx7;
	static double binx8;
	static double binx9;
	static double binx10;
	static double binxM1;
	static double binxM2;
	static double binxM3;
	static double binxM4;
	static double binxM5;
	static double binxM6;
	static double binxM7;
	static double binxM8;
	static double binxM9;
	static double binxM10;
	
	static double biny0;
	static double biny1;
	static double biny2;
	static double biny3;
	static double biny4;
	static double biny5;
	static double biny6;
	static double biny7;
	static double biny8;
	static double biny9;
	static double biny10;
	static double binyM1;
	static double binyM2;
	static double binyM3;
	static double binyM4;
	static double binyM5;
	static double binyM6;
	static double binyM7;
	static double binyM8;
	static double binyM9;
	static double binyM10;
	static double PercentToUseX;
	static double PercentToUseY;
	static int cX;
	static int cY;
	static int lowLowQuartX;
	static int lowLowQuartY;
	static int highLowQuartX;
	static int highLowQuartY;
	static int lowHighQuartX;
	static int lowHighQuartY;
	static int highHighQuartY;
	static int highHighQuartX;
	static int neglowLowQuartY;
	static int neglowLowQuartX;
	static int neghighLowQuartY;
	static int neghighLowQuartX;
	static int neglowHighQuartY;
	static int neglowHighQuartX;
	static int neghighHighQuartX;
	static int neghighHighQuartY;
	static double highQuartX;
	static double highQuartY;
	//
	////////////////////
	////////////////////
	//Movement bins
	static double movY0;
	static double movY1;
	static double movY2;
	static double movY3;
	static double movY4;
	static double movY5;
	static double movY6;
	static double movY7;
	static double movY8;
	static double movY9;
	static double movY10;
	
	static double movX0;
	static double movX1;
	static double movX2;
	static double movX3;
	static double movX4;
	static double movX5;
	static double movX6;
	static double movX7;
	static double movX8;
	static double movX9;
	static double movX10;
	
	static double movMY1;
	static double movMY2;
	static double movMY3;
	static double movMY4;
	static double movMY5;
	static double movMY6;
	static double movMY7;
	static double movMY8;
	static double movMY9;
	static double movMY10;
	
	static double movMX1;
	static double movMX2;
	static double movMX3;
	static double movMX4;
	static double movMX5;
	static double movMX6;
	static double movMX7;
	static double movMX8;
	static double movMX9;
	static double movMX10;
	/////
	/////////////////////
	
	public static double Binner(){
		double angle;
		Point2D.Double a = new Point2D.Double();
		Point2D.Double b = new Point2D.Double();
		double c;
		addToBins();
		a = decideNextBin();
		b = figureNextPoint(a);
		c = figureNextFiringAngle(b);
		return c;
	}
	
	public static void addToBins(){
		double distMovedX;
		double distMovedY;
		double PercentX;
		double PercentY;
		int mX;
		int mY;
		//To start, we need to figure out how far he moved, and change it into a percent based on how far he could've moved.
		//To deal with negative percents, which you can't get because you don't have negative distances, I made multipliers.
		//These really only show whether he has increased or decreased his x and y.
		mX = getMultiplier(Podgy.startingX, Podgy.x);
		mY = getMultiplier(Podgy.startingY, Podgy.y);
		
		cX = directionChange("x");
		cY = directionChange("y");
		
		//TIP: To change a random number between 0 and 1 to a number between to other desired numbers, you can use the formula:
		//	randomNumber * (BigDesiredNumber - SmallDesiredNumber) + SmallDesiredNumber
		
		// What I did here was get the percent, and round it to the nearest whole number. I divided that by ten to get
		//a number with a ones place and a tenths place and rounded that to the nearest whole again, and then multiplied 
		//it by ten again, effectively rounding my percent to the nearest tens place. After I did that I used the multipliers
		//on my answer to turn them negative if he moved backwards, or keep them positive if he didn't.
		distMovedX = Math.max(Podgy.startingX, Podgy.x) - Math.min(Podgy.startingX, Podgy.x);
		distMovedY = Math.max(Podgy.startingY, Podgy.y) - Math.min(Podgy.startingY, Podgy.y);
		PercentX = Math.round((Math.round((distMovedX/Podgy.PossibleMovementDist)*100))/10) * 10 * mX * cX;
		PercentY = Math.round((Math.round((distMovedY/Podgy.PossibleMovementDist)*100))/10) * 10 * mY * cY;
		//These were used mainly for debugging, but are still cool to watch.
		/*System.out.println(distMovedY);
		System.out.println(Podgy.PossibleMovementDist);
		System.out.println(" " + PercentY);
		System.out.println(" " + PercentToUseY);*/
		
		//Now we use the percent that we had determined that he had moved, and increase the count for it's respective bin.
		
		if(PercentX == 0){
		binx0 ++; lowLowQuartX++;}
		else if(PercentX == 10){
		binx1 ++; lowLowQuartX++;}
		else if(PercentX == 20){
		binx2 ++; lowLowQuartX++;}
		else if(PercentX == 30){
		binx3 ++; highLowQuartX+=2;}
		else if(PercentX == 40){
		binx4 ++; highLowQuartX+=2;}
		else if(PercentX == 50){
		binx5 ++; highLowQuartX+=3;}
		else if(PercentX == 60){
		binx6 ++; lowHighQuartX+=3;}
		else if(PercentX == 70){
		binx7 ++; lowHighQuartX+=3;}
		else if(PercentX == 80){
		binx8 ++; highHighQuartX+=4;}
		else if(PercentX == 90){
		binx9 ++; highHighQuartX+=4;}
		else if(PercentX == 100){
		binx10 ++; highHighQuartX+=4;}
		//And negative...
		else if(PercentX == -10){
		binxM1 ++; neglowLowQuartX++;}
		else if(PercentX == -20){
		binxM2 ++; neglowLowQuartX++;}
		else if(PercentX == -30){
		binxM3 ++; neghighLowQuartX+=2;}
		else if(PercentX == -40){
		binxM4 ++; neghighLowQuartX+=2;}
		else if(PercentX == -50){
		binxM5 ++; neghighLowQuartX+=3;}
		else if(PercentX == -60){
		binxM6 ++; neglowHighQuartX+=3;}
		else if(PercentX == -70){
		binxM7 ++; neglowHighQuartX+=3;}
		else if(PercentX == -80){
		binxM8 ++; neghighHighQuartX+=4;}
		else if(PercentX == -90){
		binxM9 ++; neghighHighQuartX+=4;}
		else if(PercentX == -100){
		binxM10 ++; neghighHighQuartX+=4;}
		///////////////////////////////
		//Doing the same for the Y bins
		///////////////////////////////
		if(PercentY == 0){
		biny0 ++; lowLowQuartY++;}
		else if(PercentY == 10){
		biny1 ++; lowLowQuartY++;}
		else if(PercentY == 20){
		biny2 ++; lowLowQuartY++;}
		else if(PercentY == 30){
		biny3 ++; highLowQuartY+=2;}
		else if(PercentY == 40){
		biny4 ++; highLowQuartY+=2;}
		else if(PercentY == 50){
		biny5 ++; highLowQuartY+=2;}
		else if(PercentY == 60){
		biny6 ++; lowHighQuartY+=3;}
		else if(PercentY == 70){
		biny7 ++; lowHighQuartY+=3;}
		else if(PercentY == 80){
		biny8 ++; highHighQuartY+=4;}
		else if(PercentY == 90){
		biny9 ++; highHighQuartY+=4;}
		else if(PercentY == 100){
		biny10 ++; highHighQuartY+=4;}
		//And the negative...
		else if(PercentY == -10){
		binyM1 ++; neglowLowQuartY++;}
		else if(PercentY == -20){
		binyM2 ++; neglowLowQuartY++;}
		else if(PercentY == -30){
		binyM3 ++; neghighLowQuartY+=2;}
		else if(PercentY == -40){
		binyM4 ++; neghighLowQuartY+=2;}
		else if(PercentY == -50){
		binyM5 ++; neghighLowQuartY+=2;}
		else if(PercentY == -60){
		binyM6 ++; neglowHighQuartY+=3;}
		else if(PercentY == -70){
		binyM7 ++; neglowHighQuartY+=3;}
		else if(PercentY == -80){
		binyM8 ++; neghighHighQuartY+=4;}
		else if(PercentY == -90){
		binyM9 ++; neghighHighQuartY+=4;}
		else if(PercentY == -100){
		binyM10 ++; neghighHighQuartY+=4;}
	}
	
	public static Point2D.Double decideNextBin(){
		double maxNX = 0;
		double maxNY = 0;
		//One bin, usually zero, has a ton of hits, but later down the line, bins 8, 9, and 10 have a lot of hits between them.
		//In this case it would probably be better to shoot at the highest in a group, I believe this method is called something like 
		//"Cluster Aiming"
		
		//Now that the bins are segmented into quarter groups, you can find the quarter with the most hits in it...
		highQuartX = Math.max(Math.max(Math.max(Math.max(Math.max(Math.max(lowLowQuartX, highLowQuartX), Math.max(lowHighQuartX, highHighQuartX)), neglowLowQuartX), neghighLowQuartX), neglowHighQuartX), neghighHighQuartX);
		highQuartY = Math.max(Math.max(Math.max(Math.max(Math.max(Math.max(lowLowQuartY, highLowQuartY), Math.max(lowHighQuartY, highHighQuartY)), neglowLowQuartY), neghighLowQuartY), neglowHighQuartY), neghighHighQuartY);
		
		//and aim at the highest bin in that quarter.
		if(highQuartX == highHighQuartX)
			maxNX = Math.max(Math.max(binx8, binx9), binx10);
		else if(highQuartX == neghighHighQuartX)
			maxNX = Math.max(Math.max(binxM8, binxM9), binxM10);
		else if(highQuartX == lowHighQuartX)
			maxNX = Math.max(binx6, binx7);
		else if(highQuartX == neglowHighQuartX)
			maxNX = Math.max(binxM6, binxM7);
		else if(highQuartX == highLowQuartX)
			maxNX = Math.max(Math.max(binx3, binx4), binx5);
		else if(highQuartX == neghighLowQuartX)
			maxNX = Math.max(Math.max(binxM3, binxM4), binxM5);
		else if(highQuartX == lowLowQuartX)
			maxNX = Math.max(Math.max(binx0, binx1), binx2);
		else if(highQuartX == neglowLowQuartX)
			maxNX = Math.max(binxM1, binxM2);
		
		if(highQuartY == highHighQuartY)
			maxNY = Math.max(Math.max(biny8, biny9), biny10);
		else if(highQuartY == neghighHighQuartY)
			maxNY = Math.max(Math.max(binyM8, binyM9), binyM10);
		else if(highQuartY == lowHighQuartY)
			maxNY = Math.max(biny6, biny7);
		else if(highQuartY == neglowHighQuartY)
			maxNY = Math.max(binyM6, binyM7);
		else if(highQuartY == highLowQuartY)
			maxNY = Math.max(Math.max(biny3, biny4), biny5);
		else if(highQuartY == neghighLowQuartY)
			maxNY = Math.max(Math.max(binyM3, binyM4), binyM5);
		else if(highQuartY == lowLowQuartY)
			maxNY = Math.max(Math.max(biny0, biny1), biny2);
		else if(highQuartY == neglowLowQuartY)
			maxNY = Math.max(binyM1, binyM2);
		
		
		//System.out.println("x " + maxNX);
		//System.out.println("y " + maxNY);
		return new Point2D.Double (maxNX, maxNY);
	}
	
	public static Point2D.Double figureNextPoint(Point2D.Double bin){
		Point2D.Double Point = new Point2D.Double(3, 9);
		// Now we take the number we just found, the highest one, and find out again
		//what bin it came from. This way isn't exactly efficient and space-friendly but
		//it gets the job done, usually giving the benefit of a tie to the bin closest to 0.
		
		if(highQuartX == lowLowQuartX){
			if(bin.x == binx0)
				PercentToUseX = 0;
			else if(bin.x == binx1)
				PercentToUseX = 10;
			else if(bin.x == binx2)
				PercentToUseX = 20;
		}
		if(highQuartX == neglowLowQuartX){
			if(bin.x == binxM1)
				PercentToUseX = -10;
			else if(bin.x == binxM2)
				PercentToUseX = -20;
		}
		else if(highQuartX == highLowQuartX){
			if(bin.x == binx3)
				PercentToUseX = 30;
			else if(bin.x == binx4)
				PercentToUseX = 40;
			else if(bin.x == binx5)
				PercentToUseX = 50;
		}
		else if(highQuartX == neghighLowQuartX){
			if(bin.x == binxM3)
				PercentToUseX = -30;
			else if(bin.x == binxM4)
				PercentToUseX = -40;
			else if(bin.x == binxM5)
				PercentToUseX = -50;
		}
		else if(highQuartX == lowHighQuartX){
			if(bin.x == binx6)
				PercentToUseX = 60;
			else if(bin.x == binx7)
				PercentToUseX = 70;
		}
		else if(highQuartX == neglowHighQuartX){
			if(bin.x == binxM6)
				PercentToUseX = -60;
			else if(bin.x == binxM7)
				PercentToUseX = -70;
		}
		else if(highQuartX == highHighQuartX){
			if(bin.x == binx8)
				PercentToUseX = 80;
			else if(bin.x == binx9)
				PercentToUseX = 90;
			else if(bin.x == binx10)
				PercentToUseX = 100;
		}
		else if(highQuartX == neghighHighQuartX){
			if(bin.x == binxM8)
				PercentToUseX = -80;
			else if(bin.x == binxM9)
				PercentToUseX = -90;
			else if(bin.x == binxM10)
				PercentToUseX = -100;
		}
	////////////////////////////////////////////////////////////////////////////////
		if(highQuartY == lowLowQuartY){
			if(bin.y == biny0)
				PercentToUseY = 0;
			else if(bin.y == biny1)
				PercentToUseY = 10;
			else if(bin.y == biny2)
				PercentToUseY = 20;
		}
		if(highQuartY == neglowLowQuartY){
			if(bin.y == binyM1)
				PercentToUseY = -10;
			else if(bin.y == binyM2)
				PercentToUseY = -20;
		}
		else if(highQuartY == highLowQuartY){
			if(bin.y == biny3)
				PercentToUseY = 30;
			else if(bin.y == biny4)
				PercentToUseY = 40;
			else if(bin.y == biny5)
				PercentToUseY = 50;
		}
		else if(highQuartY == neghighLowQuartY){
			if(bin.y == binyM3)
				PercentToUseY = -30;
			else if(bin.y == binyM4)
				PercentToUseY = -40;
			else if(bin.y == binyM5)
				PercentToUseY = -50;
		}
		else if(highQuartY == lowHighQuartY){
			if(bin.y == biny6)
				PercentToUseY = 60;
			else if(bin.y == biny7)
				PercentToUseY = 70;
		}
		else if(highQuartY == neglowHighQuartY){
			if(bin.y == binyM6)
				PercentToUseY = -60;
			else if(bin.y == binyM7)
				PercentToUseY = -70;
		}
		else if(highQuartY == highHighQuartY){
			if(bin.y == biny10)
				PercentToUseY = 100;
			else if(bin.y == biny8)
				PercentToUseY = 80;
			else if(bin.y == biny9)
				PercentToUseY = 90;
		}
		else if(highQuartY == neghighHighQuartY){
			if(bin.y == binyM10)
				PercentToUseY = -100;
			else if(bin.y == binyM8)
				PercentToUseY = -80;
			else if(bin.y == binyM9)
				PercentToUseY = -90;
		}

		
		// A PercentToUse of 0 means they won't move at all, and one of 50 means that they will only move half
		//as much as they have a chance to, all based on the time it will take for our bullet to get there.
		
		double PossibleMovementDistance = 0;
		double newDist = Podgy.theirDistance;
		double PointX = 0;
		double PointY = 0;
		//System.out.println(PercentToUseX);
		//Using a new possible movement distance, we can multiply it by our new percent to obtain a new x or y.
			PossibleMovementDistance = (newDist/(20-(3*3)))*8;
			PointX = (PossibleMovementDistance*(PercentToUseX/100))*Podgy.getDirectionX()*cX + Podgy.x;
			PointY = (PossibleMovementDistance*(PercentToUseY/100))*Podgy.getDirectionY()*cY + Podgy.y;
			newDist = Podgy.getrange(PointX, PointY, Podgy.x, Podgy.y);
		//Once we have the new coordinates, store them together, kind of like a package, and check them before returning.
		Point = check(new Point2D.Double(PointX, PointY));
		//System.out.println("figured point " + Point);
		return Point = new Point2D.Double(PointX, PointY);
	}
	
	public static double figureNextFiringAngle(Point2D.Double point){
		double angle = 7;
		// Once you have your coordinates, its easy to find a firing angle,
		//but be careful because this particular one gives the gunTurn measurement back in radians.
		angle = Podgy.gunHeadR - (Math.PI/2 - Math.atan2(point.y - Podgy.ourY,point.x - Podgy.ourX));
		return angle;
	}
	
	///////////////////////////////////////////////
	
	public static int getMultiplier(double fT, double nT){
		//Used for possible negative percents.
		if(nT - fT > 0)
		return 1;
		else{
		return -1;}
	}
	
	public static int directionChange(String arg){
		if(arg.equals("x")){
			if(getMultiplier(Podgy.startingX, Podgy.x) != Podgy.dAtShotX)
				return -1;
			else{
				return 1;}
		}
		else if(arg.equals("y")){
			if(getMultiplier(Podgy.startingY, Podgy.y) != Podgy.dAtShotY)
				return -1;
			else{
				return 1;}
		}
		return 1;
	}
	
	public static double updateAngle(){
		//This method is just like the Binner method,
		//but it doesn't try to update bins, worrying only about returning a new, more relevent angle.
		Point2D.Double a = new Point2D.Double();
		Point2D.Double b = new Point2D.Double();
		double c;
		a = decideNextBin();
		b = figureNextPoint(a);
		c = figureNextFiringAngle(b);
		return c;
	}
	
	public static Point2D.Double vPoint(){
		//Same deal as the updateAngle() method, but we only need a point now.
		Point2D.Double a = new Point2D.Double();
		Point2D.Double b = new Point2D.Double();
		a = decideNextBin();
		b = figureNextPoint(a);
		return b;
	}
	
	public static Point2D.Double check(Point2D.Double Cpoint) {
		//Make sure that where we're aiming makes sense
		if(Cpoint.x < 18)
			Cpoint.x = 18;
		if(Cpoint.x > Podgy.bWidth - 18)
			Cpoint.x = Podgy.bWidth - 18;
		if(Cpoint.y < 18)
			Cpoint.y = 18;
		if(Cpoint.y > Podgy.bHeight - 18)
			Cpoint.y = Podgy.bHeight - 18;
		return Cpoint;
	}
	///////////////////////
	//Thats all for the gun
	///////////////////////
	
	//////////////////////
	//Now for the movement
	
	public static void AddToBin(double p, int direction){
		p*=direction;
		//System.out.println(" "+p);
		if(p==0)
			movX0++;
		else if(p==10)
			movX1++;
		else if(p==20)
			movX2++;
		else if(p==30)
			movX3++;
		else if(p==40)
			movX4++;
		else if(p==50)
			movX5++;
		else if(p==60)
			movX6++;
		else if(p==70)
			movX7++;
		else if(p==80)
			movX8++;
		else if(p==90)
			movX9++;
		else if(p==100)
			movX10++;

		else if(p==-10)
			movMX1++;
		else if(p==-20)
			movMX2++;
		else if(p==-30)
			movMX3++;
		else if(p==-40)
			movMX4++;
		else if(p==-50)
			movMX5++;
		else if(p==-60)
			movMX6++;
		else if(p==-70)
			movMX7++;
		else if(p==-80)
			movMX8++;
		else if(p==-90)
			movMX9++;
		else if(p==-100)
			movMX10++;
	}
	
	double findLowestBin(){
		double minN = 5;
		
		minN = Math.min(Math.min(Math.min(Math.min(Math.min(Math.min(movX1, movX2), Math.min(movX2, movX3)), Math.min(Math.min(movX4, movX5), Math.min(movX6, movX7))), Math.min(Math.min(Math.min(movX8, movX9), Math.min(movX10, movMX1)), Math.min(Math.min(movMX2, movMX3), Math.min(movMX4, movMX5)))), Math.min(Math.min(movMX6, movMX7), Math.min(movMX8, movMX9))), Math.min(movMX10, movX0));

		if(minN==movX10)
			return 10;
			else if(minN==movMX10)
			return -10;
			else if(minN==movX9)
			return 9;
			else if(minN==movMX9)
			return -9;
			else if(minN==movX8)
			return 8;
			else if(minN==movMX8)
			return -8;
			else if(minN==movX7)
			return 7;
			else if(minN==movMX7)
			return -7;
			else if(minN==movX6)
			return 6;
			else if(minN==movMX6)
			return -6;
			else if(minN==movX5)
			return 5;
			else if(minN==movMX5)
			return -5;
			else if(minN==movX4)
			return 4;
			else if(minN==movMX4)
			return -4;
			else if(minN==movX3)
			return 3;
			else if(minN==movMX3)
			return -3;
			else if(minN==movX2)
			return 2;
			else if(minN==movMX2)
			return -2;
			else if(minN==movX1)
			return 1;
			else if(minN==movMX1)
			return -1;
			else if(minN==movX0)
			return 0;

		System.out.println("For some reason, we did not fall into any bins that time.");
		
		return minN;
	}

	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//I pulled Podgy's parametric gun from my other robot, Chicken,
//but most of this actual code was written for one of my other bots, Capriite.
//I didn't really feel like sorting out what I need and what I don't, so I coppied the whole class.
//WOOT

class Enemy{
	
	//I bet I don't need more than half of these.
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
	static double timeOfHit;
	static double changehead;
	static double bRad;
	
	public Point2D.Double guessPosition(long when, boolean reset, boolean absolute){

		double newX = 0;
		double newY = 0;
		double midX = 0;
		double midY = 0;
		double diff = when - currTime;
		Point2D.Double returnP = new Point2D.Double(newX, newY);
		double headingFromMid = Podgy.heading/360d * Math.PI * 2d -Math.PI/2d;//Podgy.heading-(Math.PI/2);
		double radius = Podgy.speed/changehead;

		if (Math.abs(changehead) < 0.001) {
		//This is my other bot's (Chicken's) parametric gun.
		
		//First, you have to set a grid on the enemy to track his movement.
		//Its easy to just set an origin on him and use regular x's and y's.
		//We don't want to reset however if we're just using our vBullets, so we have a reset variable that we can set to false.
		if(((numGunScans%6==0||numGunScans == 0)&reset)||absolute){
			//This sets the "t" in a parametric equation to zero...
			timeOfGunScan = currTime;
			//While these place the origin of the graph on the enemy.
			oldX = x;
			oldY = y;
		}
		else{
			//This loop is placed because of a distancing problem with time to hit now, and the actual time to hit where they'll actually be.
			//Each run through the loop jumps closer on either side of the "actual" point. 
			//One will be too far, the next iteration will be too close, but closer and closer.
        for(int i = 0; i < 20; i ++){
			//This records the time passed since the grid was placed
			timeSinceGunScan = currTime - timeOfGunScan;
			//These record his total movement from our origin
			yChange = y - oldY;
			xChange = x - oldX;
			//Remember ChangeinY / ChangeinX is the slope? This is just ChangeinY / ChangeinT.
			ySlopeT = yChange/timeSinceGunScan;
			xSlopeT = xChange/timeSinceGunScan;
			//This is the full real equation, y = slope*x is now y = timeSlope * predictedTime.
			newRelativeX = xSlopeT * diff;
			newRelativeY = ySlopeT * diff;
			//These are relative because they are based from the origin set at his coordinates, so just adjust them.
			newY = y + newRelativeY;
			newX = x + newRelativeX;
			//Reset the time it will take to hit this new place, for to maximizor the benefit of the loop.
                diff = Podgy.getrange(newX, newY, ourX, ourY)/(20-(3*3));
            }
			//These are here just to make sure again that we won't shoot where they can't be.
			returnP = new Point2D.Double (newX, newY);
			returnP = check(returnP);
		}
			if(reset||absolute){
        	numGunScans++;
			lastDistance = distance;}
		}
		else{
			//10-.75*v
			double circumf = 2*Math.PI*radius;
			midX = x - Math.sin(headingFromMid)*radius;
			midY = y - Math.cos(headingFromMid)*radius;
			double Frames = circumf/Podgy.speed;
			double currentPositionInT;
			currentPositionInT = (headingFromMid/(2*Math.PI))*Frames;
			for(int i = 0; i < 10; i++){
				double futurePositionInT = currentPositionInT + diff;
				//while(futurePositionInT>Frames){
				//	futurePositionInT -= Frames;}
				//while(futurePositionInT<0){
				//	futurePositionInT = (2*Math.PI)+futurePositionInT;}
				double tDegrees = convertTimeToRadians(futurePositionInT, Frames);
				newX = Math.sin(tDegrees) * radius + midX;
				newY = Math.cos(tDegrees) * radius + midY;
				diff = Podgy.getrange(newX, newY, ourX, ourY)/(20-(3*3));
			}
			returnP = new Point2D.Double (newX, newY);
			
		}
		
		return returnP;
	}
	
	public static double convertTimeToRadians(double position, double timeMax){
		double answer = position / timeMax;
		answer = answer * (2 * Math.PI);
		return answer;
	}
		
		//Some of this might have been relevent at one point in time or another in a different robot, but not so much here.
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
			if(Podgy.countdown>=0){ shot = false; }
			if(shot)timeOfHit = distance/(20-3*shotPower)+currTime;
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
					if(speedScanTime<450&&speedChanges>5&&(Podgy.nextTimeToHit)>timeTillNextSpeedChange){
						RepetitionGun = true;
					}
					else if(speedScanTimeDif>450){
					RepetitionGun = false;
					speedChanges=0;
					numSpeedScans=0;
					}
					if ((Podgy.nextTimeToHit)<timeTillNextSpeedChange){
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
					if(speedScanTimeDif<450&&speedChanges>5&&(Podgy.nextTimeToHit)>timeTillNextSpeedChange){
						RepetitionGun = true;
					}
					else if(speedScanTimeDif>450){
					RepetitionGun = false;
					speedChanges=0;
					numSpeedScans=0;
					}
					if ((Podgy.nextTimeToHit)<timeTillNextSpeedChange){
						RepetitionGun = false;
					}
					if(speedChanges<5){
						RepetitionGun = false;
					}
				}
		}
	}
	
	public Point2D.Double check(Point2D.Double Cpoint) {
		if(Cpoint.x < 18)
			Cpoint.x = 18;
		if(Cpoint.x > Podgy.bWidth - 18)
			Cpoint.x = Podgy.bWidth - 18;
		if(Cpoint.y < 18)
			Cpoint.y = 18;
		if(Cpoint.y > Podgy.bHeight - 18)
			Cpoint.y = Podgy.bHeight - 18;
		return Cpoint;
	}
}
/***************************************************************************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
****************************************************************************************************************/

//I used to have this in the main class but it was kind of ugly, so I moved it down.
//It was a pain in the but though to type "Podgy." in front of everything though.
class ShotTracker{

static Point2D.Double reset = new Point2D.Double();
	
BinSetter bin = new BinSetter();
Enemy enemy = new Enemy();

	public boolean trackShots(){
		boolean fire = false;
		
		vShotPar();
		vShotPodge();
		
		return fire;
	}
	
	public void vShotPar(){
		if(Podgy.timesSetPar==0)
		{
       	Podgy.nextTimeNeededToHit = (int)Math.round((Podgy.getrange(Podgy.ourX, Podgy.ourY, Podgy.x, Podgy.y)/(20-(3*3))));
		Podgy.nextTimeToHit = Podgy.nextTimeNeededToHit + enemy.currTime;
		//This is the point that we expect him to be now.
        Podgy.par = check(enemy.guessPosition(Podgy.nextTimeToHit,  false, false));
		Podgy.nextTimeNeededToHitPar = (int)Math.round((Podgy.getrange(Podgy.ourX, Podgy.ourY, Podgy.par.x, Podgy.par.y)/(20-(3*3))));
		Podgy.nextTimeToHitPar = Podgy.nextTimeNeededToHitPar + enemy.currTime;
		//Make sure we only do this when a bullet is ready.
		Podgy.timesSetPar++;
		}
		//This resets the grid on Chicken's gun, so it won't waste a virtual shot, and so that it will get reset if were not using the gun.
		reset = enemy.guessPosition(0, false, true);
	}
	
	public void vShotPodge(){
		
		if(Podgy.timesSetPodge==0)
		{
		Podgy.podge = check(bin.vPoint());
		//System.out.println(Podgy.podge);
		//System.out.println(enemy.x + " " + enemy.y);
		//System.out.println(bin.PercentToUseX+"\n");
		Podgy.nextTimeNeededToHitPodge = (int)((Podgy.getrange(Podgy.ourX, Podgy.ourY, Podgy.podge.x, Podgy.podge.y)/(20-(3*3))));
		Podgy.nextTimeToHitPodge = Podgy.nextTimeNeededToHitPodge + enemy.currTime;
		Podgy.timesSetPodge+=2;
		}
	}
	
	public void updateShots(){
		
		checkHitPodge();
		checkHitPar();
		
	}
	
	public void checkHitPodge(){
		if(Podgy.countdownPodge<=0){
			//If we're near them, its a hit, nothing fancy,
			//none of that frame by frame checking or anythng... Pshaw.
			if(enemy.x-18<Podgy.podge.x && Podgy.podge.x<enemy.x+18){
				if(enemy.y-18<Podgy.podge.y && Podgy.podge.y<enemy.y+18){
					//About here I realised I could've coppied "Podgy." and pasted it severel times... too late.
					Podgy.PodgeHits++; //System.out.println("PodgeHits ++");
				}
			}
			//System.out.println(enemy.x + " " + enemy.y);
			//System.out.println(bin.PercentToUseX+"\n");
			//System.out.println(bin.PercentToUseY);
			Podgy.timesSetPodge = 0;
		}
	}
	
	public void checkHitPar(){
		if(Podgy.countdownPar<=0){
			if(enemy.x-18<Podgy.par.x && Podgy.par.x<enemy.x+18){
				if(enemy.y-18<Podgy.par.y && Podgy.par.y<enemy.y+18){
					Podgy.ParHits++;	//System.out.println("Par Hits ++\n" );
				}
			}
			Podgy.timesSetPar = 0;
		}
	}
	
	public Point2D.Double check(Point2D.Double Cpoint) {
		if(Cpoint.x < 18)
			Cpoint.x = 18;
		if(Cpoint.x > Podgy.bWidth - 18)
			Cpoint.x = Podgy.bWidth - 18;
		if(Cpoint.y < 18)
			Cpoint.y = 18;
		if(Cpoint.y > Podgy.bHeight - 18)
			Cpoint.y = Podgy.bHeight - 18;
		return Cpoint;
	}
}

			//Pretty Maxx Rektum no?