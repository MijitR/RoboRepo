package test;
import robocode.*;
import java.awt.Color;
import java.awt.geom.*;
import robocode.util.Utils;
import java.util.ArrayList;
import java.awt.Graphics2D;

/**
 * Nabob - a robot by Damij ("Duh-midge(t)")
 */
public class Nabob extends TeamRobot
{
	static NabUtils nU = new NabUtils();
	
	static Point2D.Double theirPos = new Point2D.Double();
	static Point2D.Double ourPos = new Point2D.Double();
	
	static long ctime;
	
	static double middleBin;
	static double positiveBin;
	static double negativeBin;
	static double BINS = 31;
	static double MOVEBINS = 61;
	static double ROUNDBINS = 127;
	static double fixedBinWidth = (2*Math.PI)/ROUNDBINS;
	static double binWidth;
	static double moveBinWidth;
	
	public static ArrayList waves;
	public static ArrayList surfWaves;
	public static double[] binArray = new double[new Double(BINS+1).intValue()];
	public static double[] avoidArray = new double[new Double((MOVEBINS+1)*2).intValue()];
	public static boolean gunType = true;
	
	public static double BULLET_POWER = 2;
	
	public static int lateralDirection;
	public static int ourLatDirection;
	public static double absBearing;
	public static double relativeBearing;
	public static double dist;
	public static double hits;
	
	public static double oldDist;
	public static double oldVelocity;
	public static double oldHeading;
	public static Point2D.Double oldTheirPos = new Point2D.Double();
	public static Point2D.Double oldOurPos = new Point2D.Double();
	
	public static int roundsWon;
	
	public static double oldLife = 100;
	public static double lifeDelta;
	
	public static Rectangle2D.Double _fieldRect;
	
	public static String enemyName;
	/**
	 * run: Nabob's default behavior
	 */
	public void run() {
		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:
		setBodyColor(Color.yellow);
		//setTurretColor(Color.black);
		setRadarColor(Color.yellow);
		setBulletColor(Color.orange);
		
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		
		turnRadarLeft(360);
		
		waves = new ArrayList();
		surfWaves = new ArrayList();
		
		avoidArray[new Double(Math.round((MOVEBINS))).intValue()] += 1;
		avoidArray[new Double((MOVEBINS+1)*2-1).intValue()] += 1;
		
		_fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, getBattleFieldWidth()-36, getBattleFieldHeight()-36);
		
		if(getRoundNum()%2==0){
			for(int i = 0; i < BINS; i++){
				binArray[i] = 0;
			}
		}
		
		while(true) {
			doScanner();
			//advance();
			doMovement();
			execute();
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		double bearing = e.getBearingRadians();
		double heading = getHeadingRadians();
		absBearing = (bearing + heading);
		relativeBearing = absoluteBearing(theirPos, oldOurPos);
		dist = e.getDistance();
		double enemyVelocity = e.getVelocity();
		
		enemyName = e.getName();
		
		//setTurnRadarRightRadians(Utils.normalRelativeAngle((e.getBearingRadians()+getHeadingRadians()) - getRadarHeadingRadians()) * 2.5);
		
		ourPos = new Point2D.Double(getX(), getY());
		theirPos = nU.project(ourPos, dist, absBearing);
		
		ctime = getTime();
		
		double timeTillHit = dist/(nU.bulletVelocity(BULLET_POWER));
		double nextX = theirPos.getX();
		double nextY = theirPos.getY();
		
		if (enemyVelocity != 0) {
			lateralDirection = nU.sign(enemyVelocity * Math.sin(e.getHeadingRadians() - absBearing));
		}
		
		for(int i = 0; i < 10; i ++){
		nextX = theirPos.getX() + e.getVelocity()*Math.sin(e.getHeadingRadians())*timeTillHit;
		nextY = theirPos.getY() + e.getVelocity()*Math.cos(e.getHeadingRadians())*timeTillHit;
		timeTillHit = nU.getDist(nextX, nextY, ourPos)/(nU.bulletVelocity(BULLET_POWER));
		}
		
		//double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(nextY - getY(),nextX -  getX()));
		//setTurnGunLeftRadians(NormaliseBearing(gunOffset));
		//out.println(mostVisitedBinAngle() + " " + absBearing);
		
		BULLET_POWER = Math.round((8/(e.getVelocity()+.1) + getBattleFieldHeight()/dist + e.getEnergy()/getEnergy()*2)/3);
		if(BULLET_POWER==Double.POSITIVE_INFINITY||BULLET_POWER>3)
		BULLET_POWER = 3;
		
BULLET_POWER = 1;
		
		if(getGunTurnRemaining()<=10 && getGunHeat()==0 && getEnergy()>BULLET_POWER){
			setFire(BULLET_POWER);
			NabWaves nW = new NabWaves();
			nW.bulletVelocity = nU.bulletVelocity(BULLET_POWER);
			nW.distanceTraveled = nW.bulletVelocity;
			nW.enemyDirection = lateralDirection;
			nW.shotLocation = ourPos;
			nW.shotTime = getTime();
			nW.bearing = absBearing;
			nW.distance = e.getDistance();
			nW.binWidth = binWidth;
			waves.add(nW);
			doGun();
		}
	
		setTurnGunRightRadians(NormaliseBearing(mostVisitedBinAngle() - getGunHeadingRadians()));
	
		lifeDelta = oldLife - e.getEnergy();
		oldLife = e.getEnergy();
		ourLatDirection = (getVelocity()*Math.sin(e.getBearingRadians()) > 0 ? 1 : -1);
		if(lifeDelta <= 3 && lifeDelta >= .01){
			enemyWaves eW = new enemyWaves();
			eW.bulletVelocity = nU.bulletVelocity(lifeDelta);
			eW.distanceTraveled = eW.bulletVelocity;
			eW.shotTime = e.getTime() - 1;
			eW.shotLocation = theirPos;
			eW.direction = ourLatDirection;
			eW.bearing = fixBearing(relativeBearing);
			double middleBin = fixBearing(relativeBearing);
			Point2D.Double futurePos = new Point2D.Double();
			Point2D.Double maxPos = new Point2D.Double();
			//double maxTheta = fixBearing(absoluteBearing(theirPos, nU.project(ourPos, (e.getDistance()/nU.bulletVelocity(lifeDelta))*getVelocity()*ourLatDirection, getHeadingRadians())));
			/***********************************************
			USE PREDICTPOSITION FOR THE FARTHEST WE CAN MOVE
			711 INFO: Wins: 1 / 2 (50.00%)
			************************************************/
			maxPos = predictPosition(eW, ourLatDirection);
			futurePos = nU.linearPredict(oldVelocity, nU.bulletVelocity(lifeDelta), oldDist, oldHeading, oldTheirPos, oldOurPos);
			double maxTheta = fixBearing(absoluteBearing(theirPos, maxPos));
			double linearTheta = fixBearing(absoluteBearing(theirPos, futurePos));
			eW.linearPos = futurePos;
			eW.maxPos = maxPos;
			//out.println(nU.radToDeg(middleBin) + " " + nU.radToDeg(maxTheta) + "!!");
			//out.println(nU.radToDeg(maxTheta - middleBin));
			double minTheta = middleBin - (maxTheta - middleBin);
			double minLinearTheta = middleBin - (linearTheta - middleBin);
			double moveBinWidth = Math.abs(linearTheta - minLinearTheta)/(MOVEBINS*2);
			if(moveBinWidth<.01) moveBinWidth = .01;
			double maxBin = Math.round((maxTheta - middleBin)/(fixedBinWidth));
			double minBin = Math.round((middleBin - minTheta)/(fixedBinWidth));
			eW.linearBearing = linearTheta;
			eW.maxBearing = maxTheta;
			eW.binWidth = moveBinWidth;
			eW.maxBin = maxBin;
			eW.minBin = minBin;
			eW.ourHeading = getHeadingRadians();
			surfWaves.add(eW);
		}

		oldDist = e.getDistance();
		oldVelocity = getVelocity();
		oldHeading = getHeadingRadians();
		oldTheirPos = theirPos;
		oldOurPos = ourPos;

		advance();
		doMovement();
		execute();
		
	}

	void doScanner()
		{
		double radarOffset = 0;
			if(getTime() - ctime > 4){
				radarOffset = 4*Math.PI;
			}
			else{
			radarOffset = getRadarHeadingRadians() - (Math.PI/2 - Math.atan2(theirPos.getY() - getY(),theirPos.getX() - getX())); 

			radarOffset = NormaliseBearing(radarOffset);
			if (radarOffset < 0)
				radarOffset -= Math.PI/10;
			else {
				radarOffset += Math.PI/10; 
		}
	}
		setTurnRadarLeftRadians(radarOffset);
	}			


	public void doGun(){
		advance();
	}

	public void doMovement(){
		enemyWaves eW = new enemyWaves();
		eW = getClosestSurfableWave();
		
		if (eW == null) return;
		
		double dangerLeft = 0;
		double dangerRight = 0;
		
		//if(nU.timeTillHit(eW.bulletVelocity, nU.getDist(eW.shotLocation.getX(), eW.shotLocation.getY(), ourPos), eW.shotTime, getTime()) < 15){
			dangerLeft = checkDanger(eW, -1);
		 	dangerRight = checkDanger(eW, 1);
		//}
		
		double goAngle = absoluteBearing(eW.shotLocation, ourPos);
        if (dangerLeft < dangerRight) {
            goAngle = wallSmoothing(ourPos, goAngle - (Math.PI/2), -1);
        } else {
            goAngle = wallSmoothing(ourPos, goAngle + (Math.PI/2), 1);
        }

        setBackAsFront(this, goAngle);
		
	}

	public double checkDanger(enemyWaves surfWave, int direction) {
		//int possibleBin = new Long(Math.round((fixBearing(absoluteBearing(surfWave.shotLocation, predictPosition(surfWave, direction))) - fixBearing(surfWave.bearing))/maxEscapeAngle(surfWave.bulletVelocity)*direction + ((MOVEBINS+1)/2))).intValue();
		Point2D.Double newPos = predictPosition(surfWave, direction);
		double possibleBin = getIndex(surfWave, newPos);
		//int possibleBin = new Double( Math.round( ( absoluteBearing(surfWave.shotLocation, nU.project(ourPos,  5*direction, getHeadingRadians())) - surfWave.bearing )/surfWave.binWidth) + ((MOVEBINS+1)/2) ).intValue();
		//out.println(" " + (possibleBin));
		//int currBin = new Long(Math.round((absoluteBearing(hitWave.shotLocation, ourPos)-hitWave.bearing)/hitWave.binWidth)).intValue();
		//int possibleBin = new Double( Math.round( (NormaliseBearing(absoluteBearing(surfWave.shotLocation, predictPosition(surfWave, direction))) - NormaliseBearing(surfWave.bearing)) / ((Math.PI/2)/nU.degToRad(MOVEBINS)) ) + ((MOVEBINS)/2)).intValue();
		//out.println("pB: " + (possibleBin-(MOVEBINS/2)) + " " + avoidArray[possibleBin]);
		double danger;
		try{ danger = avoidArray[new Double(possibleBin).intValue()]; /*out.println(possibleBin - ((MOVEBINS+1)/2) + " " + (danger));*/}
		catch(Exception e){ danger = hits; };
		//out.println(" " + (possibleBin-((ROUNDBINS+1)/2)) + " " + surfWave.maxBin);
		//out.println(" " + Math.abs(new Double(possibleBin).doubleValue()/surfWave.maxBin) * hits + "\n");
		//danger+=Math.abs(new Double(possibleBin).doubleValue()/surfWave.maxBin) * hits;
		//out.println(danger);
		//out.println("----");
		return danger;
		//return direction;
    }

	public Point2D.Double predictPosition(enemyWaves surfWave, int direction) {
    	Point2D.Double predictedPosition = (Point2D.Double)ourPos.clone();
    	double predictedVelocity = getVelocity();
    	double predictedHeading = getHeadingRadians();
    	double maxTurning, moveAngle, moveDir;

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;

    	do {
    		moveAngle =
                wallSmoothing(predictedPosition, absoluteBearing(new Point2D.Double(surfWave.shotLocation.getX(),surfWave.shotLocation.getY()) , predictedPosition) + (direction * (Math.PI/2)), direction)
                - predictedHeading;
    		moveDir = 1;

    		if(Math.cos(moveAngle) < 0) {
    			moveAngle += Math.PI;
    			moveDir = -1;
    		}

    		moveAngle = Utils.normalRelativeAngle(moveAngle);

    		// maxTurning is built in like this, you can't turn more then this in one tick
    		maxTurning = Math.PI/720d*(40d - 3d*Math.abs(predictedVelocity));
    		predictedHeading = Utils.normalRelativeAngle(predictedHeading
                + limit(-maxTurning, moveAngle, maxTurning));

    		// this one is nice ;). if predictedVelocity and moveDir have
            // different signs you want to breack down
    		// otherwise you want to accelerate (look at the factor "2")
    		predictedVelocity += (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir);
    		predictedVelocity = limit(-8, predictedVelocity, 8);

    		// calculate the new predicted position
    		predictedPosition = project(predictedPosition, predictedHeading, predictedVelocity);

            counter++;

            if (predictedPosition.distance(surfWave.shotLocation) <
                surfWave.distanceTraveled + (counter * surfWave.bulletVelocity)
                + surfWave.bulletVelocity) {
                intercepted = true;
            }
    	} while(!intercepted && counter < 500);

    	return predictedPosition;
    }

	public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
        while (!_fieldRect.contains(project(botLocation, angle, 160))) {
            angle += orientation*0.05;
        }
        return angle;
    }

	public static double absoluteBearing(Point2D.Double source, Point2D.Double target) {
        return Math.atan2(target.x - source.x, target.y - source.y);
    }

	public static double fixBearing(double bearing) {
        if(bearing>=0)
			return bearing;
		else {
			return (2*Math.PI) + bearing;
		}
    }

	/*public static double fixBin(double bin){
		if(bin>
	}*/

	public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
        return new Point2D.Double(sourceLocation.x + Math.sin(angle) * length,
            sourceLocation.y + Math.cos(angle) * length);
    }

	public static void setBackAsFront(AdvancedRobot robot, double goAngle) {
        double angle =
            Utils.normalRelativeAngle(goAngle - robot.getHeadingRadians());
        if (Math.abs(angle) > (Math.PI/2)) {
            if (angle < 0) {
                robot.setTurnRightRadians(Math.PI + angle);
            } else {
                robot.setTurnLeftRadians(Math.PI - angle);
            }
            robot.setBack(100);
        } else {
            if (angle < 0) {
                robot.setTurnLeftRadians(-1*angle);
           } else {
                robot.setTurnRightRadians(angle);
           }
            robot.setAhead(100);
        }
    }

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		if(!surfWaves.isEmpty()){
			Point2D.Double hitPosition = new Point2D.Double(e.getBullet().getX(), e.getBullet().getY());
			enemyWaves hitWave = null;
			
			for(int i=0; i<surfWaves.size(); i++){
				enemyWaves eW = (enemyWaves)surfWaves.get(i);
				if(Math.abs(eW.distanceTraveled - (nU.getDist(eW.shotLocation.getX(), eW.shotLocation.getY(), ourPos))) < 50 
					&& Math.round(nU.bulletVelocity(e.getBullet().getPower())*10) == Math.round(eW.bulletVelocity*10)){
						hitWave = eW;
						break;
					}
			}
		if(hitWave!= null){
			logHit(hitWave, hitPosition);
		}
		else{out.println("Unregistered Hit!!");}
	}
		execute();
	}

	public void logHit(enemyWaves hitWave, Point2D.Double hitPosition){
		/*middleBin = hitWave.bearing;
		double binWidth = ((Math.PI/2)/nU.degToRad(MOVEBINS));
		currentBin = hitWave.direction*ourLatDirection*Math.round(Math.abs((NormaliseBearing(NormaliseBearing(absoluteBearing(hitWave.shotLocation, hitPosition)) - NormaliseBearing(middleBin))))/(binWidth));*/
		/*double middleBin = surfWave.bearing;
		double maxTheta = absoluteBearing(surfWave.shotLocation, nU.project(ourPos, surfWave.range, surfWave.ourHeading));
		double minTheta = middleBin - (maxTheta - middleBin);
		double bWidth = (maxTheta - minTheta)/MOVEBINS;*/
		/*double maxAngle = fixBearing(absoluteBearing(hitWave.shotLocation, ourPos));
		double minAngle = fixBearing(hitWave.bearing);
		double angleDelta = (maxAngle - minAngle)*ourLatDirection;*/
		//out.println(nU.radToDeg(maxAngle) + " " + nU.radToDeg(minAngle));
		/*int direction = 1;
		if(angleDelta<0)
			direction = -1;
		angleDelta*=direction;*/
		//out.println(nU.radToDeg(angleDelta));
		/*out.println("---");
		out.println(nU.radToDeg(maxAngle) + " " + nU.radToDeg(minAngle) + " " + hitWave.binWidth);
		out.println(nU.radToDeg(maxAngle-minAngle) + " " + (((angleDelta))/hitWave.binWidth));*/
		//int currBin = (new Long(Math.round(angleDelta/hitWave.binWidth)).intValue());
		double currBin = getIndex(hitWave, hitPosition);
		//out.println("bw " + hitWave.binWidth);
		//out.println("cb " + currBin);
		//out.println("mb " + (hitWave.maxBin));
		//out.println("oX " + getX());
		//out.println("gX " + hitWave.linearPos.getX());
		//out.println("--------");
		hits++;
		try{
			for (int x = 0; x < (MOVEBINS+1)*2; x++) {
            	avoidArray[x] += 1.0 / ((Math.pow((currBin - x), 2) + 1));
        	}
		}
		catch(Exception e){
			out.println(e.getStackTrace());
		}
	}

	public static double maxEscapeAngle(double velocity) {
        return Math.asin(8.0/velocity);
    }

	public static double getIndex(enemyWaves wave, Point2D.Double location){
		double maxAngle = fixBearing(absoluteBearing(wave.shotLocation, location));
		double minAngle = fixBearing(wave.bearing);
		double angleDelta = (maxAngle - minAngle);
		double factor = Utils.normalRelativeAngle(angleDelta) / maxEscapeAngle(wave.bulletVelocity) * wave.direction;
		return limit(0, (factor * ((MOVEBINS - 1) / 2)) + ((MOVEBINS - 1) / 2), MOVEBINS - 1);
	}

	public static double limit(double min, double value, double max) {
        return Math.max(min, Math.min(value, max));
    }

	public void onWin(WinEvent e){
		turnLeft(9000);
		setTurnRadarRight(90000);
		roundsWon++;
		surfWaves.clear();
		waves.clear();
		execute();
	}

	public void onDeath(DeathEvent e){
		surfWaves.clear();
		waves.clear();
	}

	double NormaliseBearing(double ang) {
		if (ang > Math.PI)
			ang -= 2*Math.PI;
		if (ang < -Math.PI)
			ang += 2*Math.PI;
		return ang;
	}

	public void onPaint(Graphics2D g) {
		if(enemyName!=null)
		g.drawString("Fighting: " + enemyName, 2, new Double(this.getBattleFieldHeight()).intValue()-10);
		g.setColor(Color.RED);
		advance();
		g.drawOval(new Double(getX() - 25).intValue(), new Double(getY() - 25).intValue(), 50, 50);
		if(waves!=null)
		for(int i = 0; i < waves.size(); i ++){
			NabWaves nW = (NabWaves)waves.get(i);
			double radius = nW.distanceTraveled;
			Point2D.Double midBin = new Point2D.Double((radius*Math.sin(nW.bearing)) + nW.shotLocation.getX(), (radius*Math.cos(nW.bearing)) + nW.shotLocation.getY());
			g.setColor(Color.RED);
			g.drawOval(new Double(nW.shotLocation.getX() - (radius)).intValue(), new Double(nW.shotLocation.getY() - (radius)).intValue(), new Double(radius*2).intValue(), new Double(radius*2).intValue());
			g.setColor(Color.YELLOW);
			g.drawOval(new Double(midBin.x - 5).intValue(), new Double(midBin.y - 5).intValue(), 10, 10);
		}
		g.setColor(Color.green);
		for(int i = 0; i < surfWaves.size(); i ++){
			enemyWaves eW = (enemyWaves)surfWaves.get(i);
			double radius = eW.distanceTraveled;
			Point2D.Double midBin = new Point2D.Double((radius*Math.sin(eW.bearing)) + eW.shotLocation.getX(), (radius*Math.cos(eW.bearing)) + eW.shotLocation.getY());
			Point2D.Double linearGun = new Point2D.Double((radius*Math.sin(eW.linearBearing)) + eW.shotLocation.getX(), (radius*Math.cos(eW.linearBearing)) + eW.shotLocation.getY());
			Point2D.Double maxAngle = new Point2D.Double((radius*Math.sin(eW.maxBearing)) + eW.shotLocation.getX(), (radius*Math.cos(eW.maxBearing)) + eW.shotLocation.getY());
			g.drawOval(new Double(eW.shotLocation.getX() - (radius)).intValue(), new Double(eW.shotLocation.getY() - (radius)).intValue(), new Double(radius*2).intValue(), new Double(radius*2).intValue());
			g.setColor(Color.WHITE);
			g.drawOval(new Double(midBin.x - 5).intValue(), new Double(midBin.y - 5).intValue(), 10, 10);
			g.setColor(Color.PINK);
			g.drawOval(new Double(linearGun.x - 5).intValue(), new Double(linearGun.y - 5).intValue(), 10, 10);
			g.setColor(Color.YELLOW);
			g.drawOval(new Double(maxAngle.x - 5).intValue(), new Double(maxAngle.y - 5).intValue(), 10, 10);
		}
	}

	public void advance(){
		double bearingOffset = 0;
		if(waves!=null)
		for(int i = 0; i < waves.size(); i ++){
			NabWaves nW = (NabWaves)waves.get(i);
			nW.distanceTraveled = (getTime() - nW.shotTime) * nW.bulletVelocity;
			if(nW.distanceTraveled > nU.getDist(theirPos.getX(), theirPos.getY(), ourPos) + 16){
				bearingOffset = nW.bearing - absBearing;
				updateBins(bearingOffset, nW);
				waves.remove(i);
				i--;
			}
		}
		if(surfWaves!=null)
		for(int i = 0; i < surfWaves.size(); i ++){
			enemyWaves eW = (enemyWaves)surfWaves.get(i);
			eW.distanceTraveled = (getTime() - eW.shotTime) * eW.bulletVelocity;
			if(eW.distanceTraveled > nU.getDist(theirPos.getX(), theirPos.getY(), ourPos) + 16){
				surfWaves.remove(i);
				i--;
			}
		}
	}

	public enemyWaves getClosestSurfableWave() {
        double closestDistance = 50000; // I juse use some very big number here
        enemyWaves surfWave = null;

		if(surfWaves!=null)
        for (int x = 0; x < surfWaves.size(); x++) {
            enemyWaves ew = (enemyWaves)surfWaves.get(x);
            double distance = nU.getDist(ew.shotLocation.getX(), ew.shotLocation.getY(), ourPos)
                - ew.distanceTraveled;

            if (distance > ew.bulletVelocity && distance < closestDistance) {
                surfWave = ew;
                closestDistance = distance;
            }
        }

        return surfWave;
    }

	double currentBin;

	//(10 - .75 * abs(velocity)) degrees / tick   is the fastest you can go through a turn
	public void updateBins(double bearing, NabWaves nW){
		middleBin = nW.bearing;
		/*
		positiveBin = middleBin + lateralDirection*(( nU.degToRad((nW.distance)/nW.bulletVelocity * (10 - (.75 * 8) )) ));
		negativeBin = -1 * (positiveBin - middleBin) + middleBin;

		binWidth = (positiveBin + negativeBin) / BINS;
		//out.println("---");
		out.println(binWidth);
		//out.println("---");
		*/
		currentBin = nW.enemyDirection*Math.round((NormaliseBearing((absoluteBearing(nW.shotLocation, theirPos) - middleBin)))/nW.binWidth);
		//out.println(" " + currentBin + " / " + nW.binWidth + " / " + nW.enemyDirection);
		//out.println("-------");
		try{
			//binArray[new Double(currentBin+((BINS+1))).intValue()]+=1;
			for (int x = 0; x < BINS; x++) {
            // for the spot bin that we hit on, add 1;
            // for the bins next to it, add 1 / 2;
            // the next one, add 1 / 5; and so on...
            binArray[x] += 1.0 / (Math.pow((currentBin+((BINS+1)/2)) - x, 2) + 1);
        }
		}
		catch(Exception e){
			out.println(e.getStackTrace());
		}
	}

	public double mostVisitedBinAngle(){
		double mostVisited = 0;
		int bin = 0;
		for(int i = 0; i < BINS; i ++){
			if(binArray[i]>mostVisited){
				bin = i;
				mostVisited = binArray[i];
			}
		}
		//out.println("    " + bin + " " + binArray[bin]);
		middleBin = absBearing;

		positiveBin = middleBin + (( nU.degToRad(((dist)/nU.bulletVelocity(BULLET_POWER)) * (10 - (.75 * 8) )) ));
		negativeBin = -1 * (positiveBin - middleBin) + middleBin;

		binWidth = Math.abs((NormaliseBearing(positiveBin) - NormaliseBearing(negativeBin)) / BINS);
		return absBearing + lateralDirection*((bin-(BINS+1)/2)*binWidth);
	}


class NabWaves {
	
	double bulletVelocity;
	double distanceTraveled;
	int enemyDirection;
	Point2D.Double shotLocation;
	long shotTime;
	double bearing;
	double distance;
	double binWidth;
	
}

class enemyWaves {
	
	double bulletVelocity;
	double distanceTraveled;
	int direction;
	Point2D.Double shotLocation;
	long shotTime;
	double bearing;
	double binWidth;
	double ourHeading;
	double range;
	double maxBin;
	double minBin;
	
	Point2D.Double linearPos;
	double linearBearing;
	Point2D.Double maxPos;
	double maxBearing;
	
}

}

class NabUtils {
	
	public Point2D.Double project(Point2D ourPos, double distance, double angle){
		Point2D.Double theirPos;
		double x = Math.sin(angle)*distance + ourPos.getX();
		double y = Math.cos(angle)*distance + ourPos.getY();
		theirPos = new Point2D.Double(x, y);
		return theirPos;
	}
	
	public double getDist(double tx, double ty, Point2D us){
		double x = tx - us.getX();
		double y = ty - us.getY();
		return Math.sqrt(x*x + y*y);
	}

	public double bulletVelocity(double power){
		return (20-(3*power));
	}

	public double timeTillHit(double bV, double dist, double timeShot, double currTime){
		return  ((dist/bV) - (currTime - timeShot));
	}

	public int sign(double v) {
		return v < 0 ? -1 : 1;
	}

	public double degToRad(double deg){
		return (deg/360*2*Math.PI);
	}

	public double radToDeg(double rad){
		return (rad/(2*Math.PI)*360);
	}

	public Point2D.Double linearPredict(double ourV, double bV, double dist, double theta, Point2D.Double theirPos, Point2D.Double usPos){
		double x, y;
		double timeTillHit = dist/bV;
		Point2D.Double ourPos = new Point2D.Double(0, 0);
		for(int i = 0; i < 10; i ++){
			x = ourV*timeTillHit*Math.sin(theta) + usPos.getX();
			y = ourV*timeTillHit*Math.cos(theta) + usPos.getY();
			ourPos = new Point2D.Double(x, y);
			timeTillHit = getDist(theirPos.getX(), theirPos.getY(), ourPos)/bV;
		}
		return ourPos;
	}

}
																																																																									