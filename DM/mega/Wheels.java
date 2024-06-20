package DM.mega;

import robocode.*;
import robocode.util.Utils;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Arc2D;
import java.awt.geom.AffineTransform;
import java.awt.Shape;
import java.awt.Graphics2D;
import java.awt.Color;
import java.util.List;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Arrays;
import java.util.HashMap;
import java.util.stream.Collectors;

/**
 * MyClass - a class by Damij
 */
public class Wheels
{

	final static int PIXELATION = 2,
			LEFT = 0, TOP = 1, RIGHT = 2, BOTTOM = 3;
			
	final static double HALF_SHADOW = 7.5D;
			
	final static Color[] paints = new Color[]{Color.RED.darker().darker().darker(),Color.BLUE.brighter().brighter(),Color.YELLOW.brighter(),Color.RED.brighter(),Color.MAGENTA,Color.BLUE.brighter(),Color.PINK,Color.CYAN};//,Color.YELLOW,Color.PINK};
	
	final static BezierCalc myCalculator = new BezierCalc();
	
	final static int MAX_MAP_SIZE = 179, SHRINK_TARGET = 178, NUM_NEIGHBORS = 15, NUM_SECOND_NEIGHBORS = 7;
	
	final static int DIST_SEGS = 4, VEL_SEGS = 4, ACCEL_SEGS = 3, NEAR_WALL_SEGS = 4, HEAD_SEGS = 4, B_POWER_SEGS = 3;
	
	final static double WALL_DIST_SEG_SIZE = Bezier.MAX_DIST / NEAR_WALL_SEGS, B_POWER_SEG_SIZE = 3d / B_POWER_SEGS,
		POINT_MASS = 20d, BOT_MASS = 0.618d, ENEMY_BOT_MASS = 3000d;//26000d;

	private final AdvancedRobot self;
	
	private final HashMap<float[], Double> offsetMap;
	
	private final List<BullegonWheels> shadows;
	
	private final List<InPlight> waves;
	
	private final List<Point2D.Double> trackPoints;
	
	private final List<FutureSpot> driveOptions, futureDrives;
	
	private final List<float[]> neighbors, secondNeighbors;
	
//	private final float[][] myNeighbors, mySecondNeighbors;
	
	private final Line2D[] frameLines = new Line2D[4];
	
	private final Point2D.Double myCenter, enemyPos, oldEPos, myOldCenter, myShotCenter, base;
	
	private final FutureSpot targetPos;
	
	private final Path2D.Double raceTrack;
	
	private InPlight surfWave, secondWave;
	
	private double absBearingTo, eDist, eOldVelocity, eOldHealth, myHeading,
		myVelocity, eDelta, oldVelocity, expectedDamage, oldAbsBearingTo,
		oldEDist, myOldHeading, myOldVelocity, maxVel, inactiveDamage;
	
	private float
		distSeg, velSeg, accelSeg, wallSpaceSeg, revWallSpaceSeg, headSeg, bPowerSeg,
		oldDSeg, oldVSeg, oldASeg, oldWSeg, oldRWSeg, oldHSeg, oldBPSeg;
	
	private long timeOfHit, inactivityTime;
	
	private int eSide, dir, latDir, activeID, secondActiveID, sneakBullets, baseIndex,
			eHits,eShots,eRangeHits,eRangeShots,oldESide,oldDir,oldLatDir, inactivityCounter;
	
	private boolean isRoundOver;

	public Wheels(final AdvancedRobot self) {
		this.self = self;
		this.raceTrack = Bezier.getEnclosure();
		this.frameLines[0] = new Line2D.Double(0d,0d,0d,36d);
		this.frameLines[1] = new Line2D.Double(0d,36d,36d,36d);
		this.frameLines[2] = new Line2D.Double(36d,36d,36d,0d);
		this.frameLines[3] = new Line2D.Double(36d,0d,0d,0d);
		this.offsetMap = new HashMap<>();
		this.shadows = new ArrayList<>();
		this.waves = new ArrayList<>();
		this.trackPoints = new ArrayList<>();
		this.driveOptions = new ArrayList<>();
		this.futureDrives = new ArrayList<>();
		this.neighbors = new ArrayList<>();
		this.secondNeighbors = new ArrayList<>();
		this.myCenter = new Point2D.Double(0d,0d);
		this.enemyPos = new Point2D.Double(0d,0d);
		this.targetPos = new FutureSpot(new Point2D.Double(0d,0d), 0, 0, 0, 0);
		this.myOldCenter = new Point2D.Double(0d,0d);
		this.oldEPos = new Point2D.Double(0d,0d);
		this.myShotCenter = new Point2D.Double(0d,0d);
		this.base = new Point2D.Double(0d,0d);
//		this.myNeighbors = new float[NUM_NEIGHBORS][9];
//		this.mySecondNeighbors = new float[NUM_NEIGHBORS][9];
		
		this.eOldHealth = 100d;
		this.dir = 1;
		this.fillTrackSegments();
		this.maxVel = 8d;
	}
	
	public void setInactivityTime(final long iT) {
		this.inactivityTime = iT;
	}
	
	public void resetInactivityCounter() {
		this.inactivityCounter = 0;
	}
	
	public void update(final ScannedRobotEvent e,
				final List<Bullet> umbrellas)
	{
		if(this.isRoundOver) {
			reset();
		}
		
		this.inactivityCounter ++;
		if(inactivityCounter >= this.inactivityTime) {
			inactiveDamage = 0.1;
		} else {
			inactiveDamage = 0;
		}

		this.frameLines[0].setLine(myCenter.x-18,myCenter.y-18,myCenter.x-18,myCenter.y+18);
		this.frameLines[1].setLine(myCenter.x-18,myCenter.y+18,myCenter.x+18,myCenter.y+18);
		this.frameLines[2].setLine(myCenter.x+18,myCenter.y+18,myCenter.x+18,myCenter.y-18);
		this.frameLines[3].setLine(myCenter.x+18,myCenter.y-18,myCenter.x-18,myCenter.y-18);
		
		this.myShotCenter.x = myOldCenter.x;//myCenter.x
		this.myShotCenter.y = myOldCenter.y;
		
		this.oldEPos.x/*enemyPos.x*/ = enemyPos.x;//freshEPos.x
		this.oldEPos.y = enemyPos.y;
		
		this.myOldCenter.x/*myCenter.x*/ = myCenter.x;//freshPos.x;
		this.myOldCenter.y = myCenter.y;//freshPos.y;
	
		this.oldAbsBearingTo/*absBearingTo*/ = absBearingTo/*freshAbsBearingTo*/;
		
		this.myOldVelocity/*myVelocity*/ = myVelocity;//freshMyVelocity;
		
		this.oldEDist = eDist; //freshEDist
		this.oldESide = eSide; //freshESide
		
		this.oldDir/*dir*/= dir;// = myVelocity == 0d ? dir : sign(myVelocity);
		this.oldLatDir = latDir;
		
		this.myOldHeading = myHeading;
		
		double tempDelt = eOldHealth - e.getEnergy() + expectedDamage - inactiveDamage;
		
		getSurfWave();
		
//		this.myCenter.x = self.getX();
//		this.myCenter.y = self.getY();
		
		this.enemyPos.x = Math.sin(absBearingTo) * e.getDistance() + myCenter.x;
		this.enemyPos.y = Math.cos(absBearingTo) * e.getDistance() + myCenter.y;
		
		this.myCenter.x = self.getX();
		this.myCenter.y = self.getY();
		
		this.absBearingTo = Utils.normalRelativeAngle(self.getHeadingRadians() + e.getBearingRadians());
		
		this.myVelocity = self.getVelocity();
		
		this.eDist = e.getDistance();
		this.eSide = sign(e.getBearingRadians());
		
		this.myHeading = self.getHeadingRadians();
		
		this.dir = myVelocity == 0d ? dir : sign(myVelocity);
		
		determineSegmentation(e.getBearingRadians());
		this.latDir = dir*eSide;
		final double bearingDiff;
	//	this.latDir = ((bearingDiff = Utils.normalRelativeAngle(Math.atan2(this.myCenter.x-this.enemyPos.x,this.myCenter.y-this.enemyPos.y)-(absBearingTo+Math.PI))) == 0 ? this.latDir : bearingDiff > 0 ? 1 : -1);
		
		if(Double.compare(Math.round((tempDelt+inactiveDamage)*100d)/100d, Math.round(100d*2d*expectedDamage)/100d) != 0 && tempDelt <= 3 && Double.compare(tempDelt, 0.01) > 0 &&  Math.abs(Math.abs(e.getVelocity())-Math.abs(eOldVelocity)) <= 2.07d) {
			sneakBullets ++;
			if(expectedDamage <= 0) {
				sneakBullets --;
			}
			if(tempDelt >= 1.0) {
				resetInactivityCounter();
			}
			eDelta = tempDelt;
			this.bPowerSeg = (float) (eDelta / B_POWER_SEG_SIZE);
			waves.add(new InPlight(
				enemyPos.x, enemyPos.y, Math.atan2(/*myShotCenter.x*/myOldCenter.x-oldEPos.x,/*myShotCenter*/myOldCenter.y-oldEPos.y)/*Utils.normalAbsoluteAngle(absBearingTo+Math.PI)*/, Rules.getBulletSpeed(eDelta), self.getTime()-1, latDir,
					 //new float[]{oldDSeg,velSeg,oldASeg,oldWSeg,oldRWSeg,oldHSeg,oldBPSeg}
					new float[]{oldDSeg, oldVSeg, oldASeg, oldWSeg, oldRWSeg, oldHSeg, oldBPSeg}
					//new float[]{distSeg, velSeg, accelSeg, wallSpaceSeg, revWallSpaceSeg, headSeg, bPowerSeg}
					, true, enemyPos.distance(myOldCenter) > 375d
			));
		}
		this.expectedDamage = 0d;
		
		findHallowedGround(umbrellas);
		
		cullBullies();
		
		decideLocation();
		
		driveTo();
		
		this.eOldHealth = e.getEnergy();
		this.eOldVelocity = e.getVelocity();
	}
	
	private void getSurfWave() {
		final long currTime = self.getTime();
		double minHitTime = Double.POSITIVE_INFINITY, secondMinHitTime = Double.POSITIVE_INFINITY;
		final Iterator<InPlight> rayinator = waves.iterator();
		surfWave = null; secondWave = null; this.secondActiveID = 0;
		double hitTime = 0d;
		while (rayinator.hasNext()) {
			final InPlight w = rayinator.next();
			hitTime = w.sourceDist(new Point2D.Double(self.getX(),self.getY()))/w.velocity - (currTime - w.fireTime);
			if(hitTime <= 1d && w.isActive()) {
				eShots ++;
				if(w.isRange) {
					eRangeShots ++;
				}
				w.setInactive();
			} if(hitTime < -2) {
				rayinator.remove();
			}
			if(hitTime < minHitTime && hitTime > -1 && w.isActive() && w.isReal) {
				minHitTime = hitTime;
				surfWave = w;
				this.timeOfHit = (long)hitTime + currTime;
			} if(hitTime < secondMinHitTime && hitTime > minHitTime && hitTime >= 2 && w.isActive() && w.isReal) {
				secondMinHitTime = hitTime;
				secondWave = w;
				this.secondActiveID = w.hashCode();
			}
			
		}
		int codeTemp;
		if(surfWave != null && (codeTemp=surfWave.hashCode()) != this.activeID) {
			this.activeID = codeTemp;
			establishNeighbors();
		}
	}
	
	private void establishNeighbors() {
		neighbors.clear();
		
		/*final Iterator<float[]> flitt = offsetMap.keySet().iterator();
		
		int neighborsAdded = 0;
		for(int i = 0; i < NUM_NEIGHBORS; i ++) {
			Arrays.fill(mySecondNeighbors[i],0);
			if(flitt.hasNext()) {
				//System.arraycopy(flitt.next(), 0, myNeighbors[i], 0, myNeighbors[i].length);
				myNeighbors[i] = flitt.next();
				neighborsAdded ++;
			}
		}
		
		final double[] nearestDistancesSq = new double[NUM_NEIGHBORS];
		nearestDistancesSq[0] = eucishDistBetween(myNeighbors[0], surfWave.key);
		double longestDistSq = nearestDistancesSq[0];
		int longestIndex = 0;
		
		for(int x = 1; x < Math.min(NUM_NEIGHBORS-1, offsetMap.size()); x ++) {
			nearestDistancesSq[x] = eucishDistBetween(myNeighbors[x], surfWave.key);
			if(nearestDistancesSq[x] > longestDistSq){
				longestDistSq = nearestDistancesSq[x];
				longestIndex = x;
			}
		}
		
		for(int x = NUM_NEIGHBORS; flitt.hasNext(); x ++) {
			final float[] storagePoint = flitt.next();
			final double distSq = eucishDistBetween(storagePoint, surfWave.key);
			if(distSq < longestDistSq) {
				//System.arraycopy(storagePoint, 0, myNeighbors[longestIndex], 0, storagePoint.length);
				myNeighbors[longestIndex] = storagePoint;
				nearestDistancesSq[longestIndex] = distSq;
				longestDistSq = 0;
				for(int y = 0; y < nearestDistancesSq.length; y ++) {
					if(nearestDistancesSq[y] > longestDistSq) {
						longestDistSq = nearestDistancesSq[y];
						longestIndex = y;
					}
				}
			}
		}*/
		

		neighbors.addAll(
			offsetMap.keySet().stream().sorted(
				(e1,e2) -> Double.compare(myManDistBetween(e1,surfWave.key),myManDistBetween(e2,surfWave.key))
			).limit(NUM_NEIGHBORS).collect(Collectors.toList())
		);
		
		secondNeighbors.clear();
		if(secondWave != null) {
			secondNeighbors.addAll(
				offsetMap.keySet().stream().sorted(
					(e1,e2) -> Double.compare(myManDistBetween(e1,secondWave.key),myManDistBetween(e2,secondWave.key))
				).limit(NUM_SECOND_NEIGHBORS).collect(Collectors.toList())
			);
		}
		
/*		if(secondWave != null) {
			final Iterator<float[]> tilff = offsetMap.keySet().iterator();
			for(int i = 0; i < NUM_NEIGHBORS; i ++) {
				if(tilff.hasNext()) {
					mySecondNeighbors[i] = tilff.next();
				}
			}
			Arrays.fill(nearestDistancesSq, 0d);
			nearestDistancesSq[0] = eucishDistBetween(mySecondNeighbors[0], secondWave.key);
			longestDistSq = nearestDistancesSq[0];
			longestIndex = 0;
			for(int x = 1; x < Math.min(NUM_NEIGHBORS-1, offsetMap.size()); x ++) {
				nearestDistancesSq[x] = eucishDistBetween(mySecondNeighbors[x], secondWave.key);
				if(nearestDistancesSq[x] > longestDistSq) {
					longestDistSq = nearestDistancesSq[x];
					longestIndex = x;
				}
			}
			
			for(int x = NUM_NEIGHBORS; tilff.hasNext(); x ++) {
				final float[] storagePoint = tilff.next();
				final double distSq = eucishDistBetween(storagePoint, secondWave.key);
				if(distSq < longestDistSq) {
					mySecondNeighbors[longestIndex] = storagePoint;
					nearestDistancesSq[longestIndex] = distSq;
					longestDistSq = 0;
					for(int y = 0; y < nearestDistancesSq.length; y ++) {
						if(nearestDistancesSq[y] > longestDistSq) {
							longestDistSq = nearestDistancesSq[y];
							longestIndex = y;
						}
					}
				}
			}
		}
*/
		
	}
	
	private void decideLocation() {
		final Point2D.Double baseLoc = establishXYBase();
		base.x = baseLoc.x;
		base.y = baseLoc.y;
		
		this.baseIndex = trackPoints.indexOf(base);
		
		FutureSpot target;
		
		expand(target = new FutureSpot(baseLoc, myVelocity, self.getTime(), eSide, sign(myVelocity)*eSide), surfWave, driveOptions);

		if(surfWave != null) {
			expand(target = new FutureSpot(baseLoc, myVelocity, self.getTime(), eSide, surfWave.dir), surfWave, driveOptions);
			target = getLeastDangerous(base);
		} else {
			expand(target = new FutureSpot(baseLoc, myVelocity, self.getTime(), eSide, sign(myVelocity)*eSide), surfWave, driveOptions);
		}
		
		this.targetPos.x = target.x;
		this.targetPos.y = target.y;
		
		targetPos.setVelocity(target.velocity()).setDir(target.dir());
		
		final double myDist = dist(target.x,target.y,myCenter.x,myCenter.y);

		if(surfWave != null)
			maxVel = //8d;// /*latDir * target.dir() < 0 ? 1 : */
					//8d;
					//Math.abs(target.velocity());
					//8d;
					myDist/Math.max(.125d,(this.timeOfHit-self.getTime()-2));
			//maxVel = Math.min(Math.abs(target.velocity()), Math.max(0d,(Math.min(8d, myDist/Math.max(1d,(this.timeOfHit-self.getTime()-2))))));//-2-calcDeccelTicks(Math.abs(myVelocity))/2)))));
		else
			maxVel = 8d;
		
		/*final double angleToTarget = Math.atan2(target.x - myCenter.x, target.y - myCenter.y);
		
		final int frontBack = Math.abs(Utils.normalRelativeAngle(angleToTarget-myHeading)) <= Math.PI/2d ? 1 : -1;
		
		final double slowDownVelocity = (double) (int) (Math.sqrt(4.0 * myDist+ 1.0) - 1.0);
		
		final double deccelDist = calcDecelDist(Math.abs(myVelocity));
		
		final boolean follow = frontBack * myVelocity >= 0 && myDist >= 1d && (
				(limit(0d,myVelocity*frontBack + (frontBack*myVelocity<0?2*frontBack:frontBack),8d) >  slowDownVelocity
					|| myDist > deccelDist));
		
		//final double nextVel = surfWave == null ? 64d : Math.min(Math.abs(!follow?limit(-8d,myVelocity+2*frontBack,8d):myDist), myDist);
		final double nextVel = follow ? 24d : 0d;		
		
		this.targetPos.x = myCenter.x + nextVel * Math.sin(angleToTarget);
		this.targetPos.y = myCenter.y + nextVel * Math.cos(angleToTarget);*/
	}
	
	private double calcDecelDist(final double absVelocity) {
		if(absVelocity <= 2) {
			return 0;
		}
		return absVelocity - 2 + calcDecelDist(absVelocity - 2);
		//return 1.5d*absVelocity;
	}
	
	private double calcAccelDist(final double distRemaining, final double absVelocity) {
		//if(relVelocity >= 8 || distRemaining <= 1.5d*relVelocity) {//calcDecelDist(Math.abs(relVelocity))) {
		//	return 0d;
		//}
		//return relVelocity + 1 + calcAccelDist(distRemaining - relVelocity - 1, relVelocity + 1);
		return Math.min((8-absVelocity)*(8+absVelocity)/2d, Math.max(distRemaining - 1.5d * absVelocity,0));
	}
	
	private double calcAccelMax(final double distRemaining, final double absVelocity) {
		if(absVelocity >= 8 || distRemaining <= 1.5d * absVelocity) {//calcDecelDist(absVelocity)) {
			return Math.min(absVelocity+1, 8d);
		}
		return calcAccelMax(distRemaining - absVelocity - 1, absVelocity + 1);
		//return Math.min(8d, (distRemaining - absVelocity - 1)/(absVelocity+1) + absVelocity);
	}
	
	private int calcAccelTicks(final double dist, final double startAbsVel) {
		if(dist <= 0 || startAbsVel >= 8d) {
			return 0;
		} return 1 + calcAccelTicks(dist - startAbsVel - 1, startAbsVel + 1);
	}
	
	private int calcDeccelTicks(final double absVel) {
		if(absVel <= 0) {
			return 0;
		} return 1 + calcDeccelTicks(absVel - 2d);
	}
	
	private double sumUpTo(final int accelTicks) {
		if(accelTicks <= 0) {
			return 0;
		} return accelTicks + sumUpTo(accelTicks-1);
	}
	
	private double sumUpTo(final int accelTicks, final double alreadyVel) {
		if(accelTicks <= 0 || alreadyVel >= 8d) {
			return 0;
		} return accelTicks * alreadyVel + sumUpTo(accelTicks-1, alreadyVel + 1);
	}
	
	private void driveTo() {
		final double distToTarget = targetPos.distance(myCenter);
		double goDir = 1d;
		/*if(distToTarget < 0d) {
			//System.out.println(targetPos.dir() + " was dir so vel is : " + targetPos.velocity() + " and xy -> " + targetPos.x + ", " + targetPos.y);
			int nextIndex = trackPoints.indexOf(new Point2D.Double(targetPos.x,targetPos.y)) + 2*targetPos.dir();
			//System.out.println(trackPoints.indexOf(targetPos));
			if(nextIndex < 0) {
				nextIndex += trackPoints.size();
			}
			nextIndex %= trackPoints.size();
			final Point2D.Double clockPoint = trackPoints.get(nextIndex);//basePoint = trackPoints.get(targetPos);
		final double absAngleToTarget = Utils.normalAbsoluteAngle(Math.atan2(clockPoint.x - myCenter.x, clockPoint.y - myCenter.y));
		double turnAmount = Utils.normalRelativeAngle(absAngleToTarget - myHeading);
		
			if(Math.abs(turnAmount) > Math.PI/2d) {
				turnAmount = -Utils.normalRelativeAngle(Math.PI-turnAmount);
				goDir = -1;
			}
			self.setTurnRightRadians(
				turnAmount
			);
		}
		else {*/
			final double absAngleToTarget = Utils.normalAbsoluteAngle(Math.atan2(targetPos.x - myCenter.x, targetPos.y - myCenter.y));
		double turnAmount = Utils.normalRelativeAngle(absAngleToTarget - myHeading);
		
			if(Math.abs(turnAmount) > Math.PI/2d) {
				turnAmount = -Utils.normalRelativeAngle(Math.PI-turnAmount);
				goDir = -1;
			}
			

		self.setMaxVelocity(Math.min(maxVel,Math.PI/2d/(Math.abs(turnAmount))));

			self.setTurnRightRadians(
				turnAmount*Math.max(0d,Double.compare(distToTarget,1.1d))
			);
		//}
		/*if(Math.abs(turnAmount) > Math.PI/2d) {
			goDir = -1;
			turnAmount = -Utils.normalRelativeAngle(Math.PI-turnAmount);
		}
		//if(Math.abs(turnAmount) < Rules.getTurnRateRadians(Math.abs(myVelocity)) && distToTarget > 1d) {//0.2d*Math.PI) {
			//self.setAhead(2.1d*distToTarget*goDir);
		//} else {
		//	self.setAhead(0d);
		//}
		//final double absTo = surfWave == null ? absBearingTo : Math.atan2(surfWave.sourceX()-myCenter.x,surfWave.sourceY()-myCenter.y);
		
//		self.setMaxVelocity(maxVel);
		/*final int baseIndex = trackPoints.indexOf(base);
		int nextIndex = baseIndex + latDir;
		if(nextIndex < 0) {
			nextIndex += trackPoints.size();
		}
		nextIndex %= trackPoints.size();
		final Point2D.Double clockPoint = trackPoints.get(nextIndex);
		
		final double tangent;
		
		final double headingLikeness = Math.abs(Math.cos(Math.atan2(base.x-myCenter.x,base.y-myCenter.y)-(tangent=Math.atan2(clockPoint.x-base.x,clockPoint.y-base.y))));
		*/
		self.setAhead(/*(Math.abs(myVelocity)+1d)*/3d*(distToTarget)*goDir);
	/*	if(Double.compare(targetPos.distance(myCenter), 1.1d) < 0) {
			self.setTurnRightRadians(turnAmount*Math.max(0d,Double.compare(targetPos.distance(myCenter), 1d)));
		}
		else {
			int nextIndex = trackPoints.indexOf(targetPos) + 10*latDir;
			if(nextIndex < 0) {
				nextIndex += trackPoints.size();
			}
			nextIndex %= trackPoints.size();
			final Point2D.Double clockPoint = trackPoints.get(nextIndex);//basePoint = trackPoints.get(targetPos);
			turnAmount = Utils.normalRelativeAngle(
					Math.atan2(clockPoint.x-myCenter.x,clockPoint.y-myCenter.y) - myHeading 
				);
			if(Math.abs(turnAmount) > Math.PI/2d) {
				turnAmount = -Utils.normalRelativeAngle(Math.PI-turnAmount);
			}
			self.setTurnRightRadians(
				turnAmount
			);
		}*/
	}
	
	private void expand(final FutureSpot base, final InPlight toSurf, final List<FutureSpot> driverList) {
		driverList.clear();
		driverList.add(base);
		final int baseIndex = trackPoints.indexOf(base);
		final long playTime = this.timeOfHit-base.time();
		final long currTime = self.getTime();
		
		
		if(toSurf != null) {
			for(int dLDir = base.dir()/*sign(base.velocity()*base.eSide())*/, count = 0; count < 2; dLDir *= -1, count ++) {
				int runningIndex = baseIndex + dLDir;
				if(runningIndex < 0) {
					runningIndex += trackPoints.size();
				} runningIndex %= trackPoints.size();
				Point2D.Double testPos;
				double futures = 0d;
				double expectedVel = base.velocity();	
				do {				

					testPos = trackPoints.get(runningIndex);
					final double newDist = testPos.distance(base);
					final double deccelDist = calcDecelDist(Math.abs(base.velocity()));
		
					final long timeOfHit = toSurf.timeTillBreak(testPos, currTime);
					
					//continuing forward
					if(dLDir * base.dir() >= 0 && newDist > deccelDist) {
						final int accelTicks = calcAccelTicks(newDist, Math.abs(base.velocity()));
						final double accelDist = sumUpTo(accelTicks, Math.abs(base.velocity()));
						final int deccelTicks;
						//futures = Math.max(1d, (newDist-accelDist)/Math.max(1,Math.abs(expectedVel = (base.velocity() + accelTicks*sign(base.velocity()))))) + accelTicks;// + (deccelTicks = calcDeccelTicks(Math.abs(expectedVel = base.velocity()+accelTicks*sign(base.velocity()))))/3;
						
						futures = accelTicks + Math.max(0d,newDist-accelDist)/8d;//Math.max(1d,(expectedVel = base.velocity() + accelTicks*sign(base.velocity())));
						//expectedVel = base.velocity() + accelTicks*sign(base.velocity());
						final double potentialVel = sign(base.velocity()) * newDist/Math.max(.125d,(timeOfHit-currTime - 2));
						//expectedVel = Math.min(Math.abs(expectedVel), potentialVel) * sign(expectedVel);
						
						//expectedVel = 8d*sign(base.velocity());//newDist / Math.max(1d,(base.time()+(long)Math.ceil(futures)-1)) * sign(expectedVel);
						//deccelTicks = calceccelTicks(Math.abs(expectedVel+accelTicks));
						//expectedVel = base.velocity() + ((accelTicks) * sign(base.velocity()));
						//futures = newDist / 8d;
						//expectedVel = base.eSide()*dLDir*Math.max(0d,newDist/Math.max(1d,(playTime-futures-2-calcDeccelTicks(Math.abs(expectedVel))/2)));
						//System.out.println(expectedVel);
					}
					//turning around
					else {
						final int deccelTicks = calcDeccelTicks(Math.abs(base.velocity()));
						final int accelTicks = calcAccelTicks(/*newDist <= deccelDist ? newDist - deccelDist : */newDist - deccelDist, 0d);
						final double accelDist = sumUpTo(accelTicks, 0d);
						futures = accelTicks + deccelTicks + Math.max(0d, (newDist-accelDist+(dLDir == base.dir() ? 0 : deccelDist)))/8d;///Math.max(1d,Math.abs(expectedVel = (/*dLDir == base.dir() ? base.velocity() - 2*deccelTicks  * sign(base.velocity()) :*/ -accelTicks  * sign(base.velocity())))));
						//expectedVel = -8d*sign(base.velocity());
							//(base.velocity() - (2*deccelTicks+accelTicks)*sign(base.velocity()));
						//expectedVel = 0d;
						expectedVel = base.velocity() - 2d * (deccelTicks + 0.5d*accelTicks) * sign(base.velocity());//(deccelDist >= newDist ? expectedVel - 2*deccelTicks  * sign(expectedVel) : -accelTicks  * sign(expectedVel));
					}
					/*	if(dLDir*base.eSide() > 0) {
							expectedVel = Math.min(expectedVel, Math.max(0d,newDist/Math.max(1d,(playTime-futures))));
						} else {
							expectedVel = Math.max(expectedVel, Math.min(0d,-newDist/Math.max(1d,(playTime-futures))));
						}
					*/
					expectedVel = limit(-8d, expectedVel, 8d);
					
					driverList.add(new FutureSpot(testPos, expectedVel, base.time() + (long)Math.ceil(futures), eSide, dLDir));

					runningIndex += dLDir;
					if(runningIndex < 0) {
						runningIndex += trackPoints.size();
					} runningIndex %= trackPoints.size();
					
				} while(!(/*base.time() == self.getTime() ? */toSurf.breaks(testPos, base.time() + (int)Math.ceil(futures))));// : toSurf.checkHit(testPos, base.time() + (int)Math.ceil(futures))));
				//driverList.remove(driverList.size()-1);
			}
		}
	}
	
	private void expandBullshit(final FutureSpot base, final Point2D.Double start, final InPlight toSurf, final long currTime, final List<FutureSpot> driverList) {
		driverList.clear();
		final int baseIndex = trackPoints.indexOf(base);
		final Point2D.Double nextBase = trackPoints.get((baseIndex+1)%trackPoints.size());
		double distDelta = start.distance(base);
		
		final double headingLikeness = Math.abs(Math.cos(Math.atan2(base.x-start.x,base.y-start.y) - Math.atan2(nextBase.x-base.x,nextBase.y-base.y)));
		
//		driverList.add(new FutureSpot(base, base.velocity(), currTime, base.latDir()));
		
		if(toSurf != null) {
			for(int goDir = -1; goDir < 2; goDir += 2) {
			//	double velStart = 0d;
			//	double runningRelativeVelocity = currTime == self.getTime() ? (velStart = Math.abs(self.getVelocity()) * goDir * latDir * headingLikeness) : base.velocity() * goDir;
				double runningVelocity = 0d;//base.velocity() * headingLikeness * base.latDir() * goDir;
		
				int runningIndex = baseIndex + goDir;
				if(runningIndex < 0) {
					runningIndex += trackPoints.size();
				} runningIndex %= trackPoints.size();
				Point2D.Double testPos, lastPos = new Point2D.Double(base.x,base.y);
				//long accelDecelTicks = 0;
				double futures = 0, deltaFut, lastFutures = 0;
				do {
					testPos = trackPoints.get(runningIndex);
					
					//distDelta = lastPos.distance(testPos);
					
					futures = 0;
					double myDist = testPos.distance(start);
					
					double useableVel = Math.max(0d, runningVelocity * eSide * goDir);
					if(runningVelocity * eSide * goDir < 0) {
						myDist += calcDecelDist(Math.abs(runningVelocity));
						futures = calcDeccelTicks(Math.abs(runningVelocity));
					}
				//	final double accelMax = calcAccelMax(myDist, runningRelativeVelocity);
				//	accelDecelTicks = runningRelativeVelocity >= 0 ? (long)Math.ceil(accelMax/2d + accelMax - runningRelativeVelocity) : (long)Math.ceil(accelMax/2d - runningRelativeVelocity / 2d + accelMax);
					futures += myDist <= 16d ? myDist/(((8+useableVel)/2d)) : (myDist- (((8+useableVel)/2d) * ((8-useableVel)/2)))/8d;// * (< 0 ? -1.5d*runningRelativeVelocity + 10d + (myDist-10d)/8d : (8+runningRelativeVelocity)/2d + (myDist - ((8+runningRelativeVelocity)/2d) * ((8-runningRelativeVelocity)/2)) / 8d));
					deltaFut = Math.abs(futures - lastFutures);
					
		
//					runningVelocity += goDir * base.latDir() * base.velocity() < 0 ? 2d*goDir*base.latDir()*futures : goDir*base.latDir()*futures;
					runningVelocity = limit(-8d, runningVelocity, 8d);
		//			if(runningRelativeVelocity < 0) {
		//				final double decelDist = calcDecelDist(Math.abs(runningRelativeVelocity));
		//				final int accelTicks;
		//				futures = Math.ceil(Math.abs(runningRelativeVelocity)/2d) + (accelTicks = calcAccelTicks(decelDist, 0));
		//				final double useableVel = sumUpTo(accelTicks);
		//				futures += myDist <= 16d ? myDist/(((8+useableVel)/2d)) : (myDist- (((8+useableVel)/2d) * ((8-useableVel)/2)))/8d;
		//			} else {
					//	final double useableVel = runningRelativeVelocity;
					//	futures = myDist <= 16d ? myDist/(((8+useableVel)/2d)) : (myDist- (((8+useableVel)/2d) * ((8-useableVel)/2)))/8d;
				/*	if(runningRelativeVelocity >= 0) {
					final double maxVel = calcAccelMax(myDist, runningRelativeVelocity);
					final double deccelTicks = calcDeccelTicks(maxVel);
					final double deccelDist = calcDecelDist(maxVel);
					final double accelTicks = calcAccelTicks(Math.max(0d,myDist-deccelDist), 0);
					//final double decelDist = calcDecelDist(maxVel);
					
					futures = accelTicks + deccelTicks + Math.max(0d, (myDist - sumUpTo((int)Math.ceil(accelTicks)) - deccelDist)/8d);
					} else {
						final double initDeccel = calcDecelDist(Math.abs(runningRelativeVelocity));
						final double maxVel = calcAccelMax(myDist+initDeccel, 0d);
						final double deccelTicks = calcDeccelTicks(maxVel) + calcDeccelTicks(Math.abs(runningRelativeVelocity));
						final double decelDist = calcDecelDist(maxVel);
						final double accelTicks = calcAccelTicks(Math.max(0d,myDist+initDeccel-decelDist), 0);
					futures = accelTicks + deccelTicks + Math.max(0d, (myDist + initDeccel - sumUpTo((int)Math.ceil(accelTicks)) - decelDist)/8d);
					}*/
					
					
					driverList.add(new FutureSpot(testPos, runningVelocity, currTime + (long)Math.ceil(futures), eSide, goDir));
		//			}
					//	(myDist - calcDecelDist(Math.abs(runningRelativeVelocity)) - calcAccelDist(myDist, runningRelativeVelocity)) / 8d
					//		+ accelDecelTicks);
					//deltaFut = Math.abs(futures - lastFutures);
					lastFutures = futures;
					
					//System.out.println("Delta Fut:" + deltaFut + "\nLeadingTo New Fut: " + futures);
					
					//final double newRelVel = runningRelativeVelocity + deltaFut;//< 0 ? runningRelativeVelocity + 2d * deltaFut : runningRelativeVelocity + deltaFut;
		//			if(newRelVel * runningRelativeVelocity < 0) {
		//				runningRelativeVelocity = calcAccelTicks(futures - Math.ceil(Math.abs(runningRelativeVelocity) / 2d), 0);
		//			} else {
		//				if(runningRelativeVelocity <= 0) {
					//		runningRelativeVelocity = newRelVel;
		//				}
		//				else {
		//					runningRelativeVelocity = calcAccelMax(testPos.distance(base), velStart);//newRelVel;
		//				}
		//			}
					//runningRelativeVelocity = limit(-8d, runningRelativeVelocity, 8d);
					lastPos.x = testPos.x;
					lastPos.y = testPos.y;
					
					runningIndex += goDir;
if(runningIndex < 0) {
						runningIndex += trackPoints.size();
					} runningIndex %= trackPoints.size();
				} while(/*!toSurf.breaks(testPos, currTime + (int)Math.ceil(futures) + accelDecelTicks) ||*/ !toSurf.checkHit(testPos, currTime + (int) Math.ceil(futures)));
				driverList.remove(driverList.size()-1);
			}
		}
	}
	
	private FutureSpot getLeastDangerous(final Point2D.Double base) {
		final long currTime = self.getTime();
		double minDanger = Double.MAX_VALUE;
		FutureSpot target = new FutureSpot(new Point2D.Double(base.x,base.y), myVelocity, self.getTime(), eSide, latDir);
		double secondWaveScalar = 1d;
		if(secondWave != null) {
			secondWaveScalar = 1d;//(secondWave.velocity-20d)/-3d/((surfWave.velocity-20d)/-3d) * NUM_NEIGHBORS / (NUM_SECOND_NEIGHBORS)
			//	 / Math.max(1d, Math.log1p(surfWave.timeTillBreak(myCenter, self.getTime())-2));
		}//(((8+runningRelativeVelocity)/2d) * ((8-runningRelativeVelocity)/2))}
		if(surfWave != null) {
			for(final FutureSpot possible : driveOptions) {
				double distFrom = dist(possible.x,possible.y,surfWave.sourceX(),surfWave.sourceY());
				double stDev = 36d / distFrom / surfWave.mEA;
				double factor = surfWave.getOffsetFactor(possible);
				double danger = 0d;//Bezier.MAX_DIST/dist(possible.x,possible.y,enemyPos.x,enemyPos.y);
				boolean contained = false;
				//		System.out.println("Testing offset : " +factor);
				for(final BullegonWheels shader : shadows) {
					if(shader.parent == this.activeID
							//&& shader.contains(possible)
								&& shader.contains(new Rectangle2D.Double(possible.x-HALF_SHADOW,possible.y-HALF_SHADOW,2*HALF_SHADOW,2*HALF_SHADOW))) {
						contained = true;
						break;
					}
				}

				if(!contained) {
					for(final float[] nay : neighbors) {
						final double memoryFactor = offsetMap.get(nay);
						final double x = (factor-memoryFactor)/stDev;
						danger += Math.exp(-0.5d * x * x) / Math.max(0.01d,myManDistBetween(nay, surfWave.key));
					}
				}
				if(danger >= minDanger) {
					continue;
				}
				final List<FutureSpot> myFutures = new ArrayList<>();
				double secondDanger = 0d, minSecondDanger = 0.5d*Double.MAX_VALUE, avgSecondDanger = 0d, maxSecondDanger = 0d, crimpFactor = 1d;
				
				if(secondWave != null) {
					//secondWaveScalar =// (secondWave.velocity-20d)/-3d/((surfWave.velocity-20d)/-3d)// * Math.max(1d, 2d*Math.log1p(surfWave.timeTillBreak(myCenter, self.getTime())));
					//		  ;// * NUM_NEIGHBORS / (NUM_SECOND_NEIGHBORS);
					expand(possible, secondWave, myFutures);
					crimpFactor = Math.max(0.5d, Math.abs(Math.cos(Math.atan2(possible.x-base.x,possible.y-base.y)-Math.atan2(enemyPos.x-base.x,enemyPos.y-base.y))));
					for(final Point2D.Double morePossible : myFutures) {
						secondDanger = 0d;
						distFrom = dist(morePossible.x,morePossible.y,secondWave.sourceX(),secondWave.sourceY());
						stDev = 36d / distFrom / secondWave.mEA;
						factor = secondWave.getOffsetFactor(morePossible);
						for(final float[] flatty : secondNeighbors) {
							final double memoryFactor = offsetMap.get(flatty);
							final double x = (factor - memoryFactor)/stDev;
							secondDanger += Math.exp(-0.5d * x * x) / Math.max(0.01d, myManDistBetween(flatty, secondWave.key));
							
						}
						avgSecondDanger += secondDanger;
						minSecondDanger = Math.min(minSecondDanger, secondDanger);
						maxSecondDanger = Math.max(maxSecondDanger, secondDanger);
					}
				danger += 2d*crimpFactor * (minSecondDanger*myFutures.size()+avgSecondDanger)/(2d*myFutures.size()) * secondWaveScalar;
				//crimpFactor = Math.max(0.7618d, Math.abs(Math.cos(Math.atan2(possible.x-base.x,possible.y-base.y)-Math.atan2(secondWave.sourceX()-base.x,secondWave.sourceY()-base.y))));
				//danger += (avgSecondDanger/secondNeighbors.size() + minSecondDanger)/(2d);
				//danger += secondWaveScalar * (Math.sqrt(avgSecondDanger * maxSecondDanger) + avgSecondDanger / secondNeighbors.size())/2d;
				//danger += (avgSecondDanger / (3*secondNeighbors.size()) + (minSecondDanger + maxSecondDanger) / 3d) * secondWaveScalar;
 				}
				//final double crimpFactor = Math.max(0.1618d, Math.abs(Math.cos(Math.atan2(possible.x-base.x,possible.y-base.y)-Math.atan2(surfWave.sourceX()-possible.x,surfWave.sourceY()-possible.y))));
				//final double crimpFactor = Math.max(0.9618d, Math.abs(Math.cos(Math.atan2(possible.x-base.x,possible.y-base.y)-Math.atan2(enemyPos.x-base.x,enemyPos.y-base.y))));
				if(danger < minDanger) {
					minDanger = danger;
					target.setX(possible.x);
					target.setY(possible.y);
					target.setDir(possible.dir()).setVelocity(possible.velocity());
					futureDrives.clear();
					futureDrives.addAll(myFutures);
				}
			}
		}
		return target;
	}

	private Point2D.Double establishXYBase() {
		//final double comparator = Math.log1p(myCenter.distance(enemyPos));
		final double tangentAngle = absBearingTo+Math.PI/2d*eSide;//, tanAngle2 = absBearingTo - 7.1d*Math.PI/16d;
		return trackPoints.stream().sorted((p1,p2)-> { 
				return Double.compare(
					ENEMY_BOT_MASS*p2.distance(enemyPos)/Math.max(p2.distance(myCenter),6.618d) + BOT_MASS*Math.cos(Math.atan2(p2.x-myCenter.x,p2.y-myCenter.y)-tangentAngle)/eDist,// + Math.cos(Math.atan2(p2.x-myCenter.x,p2.y-myCenter.y) - myHeading)/eDist,
					ENEMY_BOT_MASS*p1.distance(enemyPos)/Math.max(p1.distance(myCenter),6.618d) + BOT_MASS*Math.cos(Math.atan2(p1.x-myCenter.x,p2.y-myCenter.y)-tangentAngle)/eDist// + Math.cos(Math.atan2(p1.x-myCenter.x,p1.y-myCenter.y) - myHeading)/eDist
				);
			}
		).findFirst().orElse(myCenter);
	}
	
	private double[] establishXYBaseLegacy() {
		int leftCounter = 1, rightCounter = 1, upCounter = 1, downCounter = 1;
		boolean left, right, up, down, vertical;
		if(Bezier.surfTains(myCenter)) {
			final Line2D.Double tester = new Line2D.Double(myCenter.x,myCenter.y,myCenter.x-PIXELATION,myCenter.y);
			while(Bezier.isInTrack(tester.getBounds())) {
				leftCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x-PIXELATION*leftCounter,myCenter.y);
			}
			tester.setLine(myCenter.x,myCenter.y,myCenter.x+PIXELATION,myCenter.y);
			while(Bezier.isInTrack(tester.getBounds()) && rightCounter < leftCounter) {
				rightCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x+PIXELATION*rightCounter,myCenter.y);
			}
			tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y+PIXELATION);
			while(Bezier.isInTrack(tester.getBounds())) {
				upCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y+PIXELATION*upCounter);
			}
			tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y-PIXELATION);
			while(Bezier.isInTrack(tester.getBounds())) {
				downCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y-PIXELATION*downCounter);
			}
			vertical = Math.min(upCounter,downCounter) < Math.min(leftCounter, rightCounter);
			up = upCounter <= downCounter; down = !up;
			left = leftCounter <= rightCounter; right = !left;
		} else {
			final Line2D.Double tester = new Line2D.Double(myCenter.x,myCenter.y,myCenter.x-PIXELATION,myCenter.y);
			while(!(left=Bezier.meetsTrack(tester.getBounds())) && !Bezier.isExterior(tester.getBounds())) {
				leftCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x-PIXELATION*leftCounter,myCenter.y);
			}
			
			tester.setLine(myCenter.x,myCenter.y,myCenter.x+PIXELATION,myCenter.y);
			while(!(right=Bezier.meetsTrack(tester.getBounds())) && !Bezier.isExterior(tester.getBounds())) {
				rightCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x+PIXELATION*rightCounter,myCenter.y);
			}
			tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y+PIXELATION);
			while(!(up=Bezier.meetsTrack(tester.getBounds())) && !Bezier.isExterior(tester.getBounds())) {
				upCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y+PIXELATION*upCounter);
			}
			tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y-PIXELATION);
			while(!(down=Bezier.meetsTrack(tester.getBounds())) && !Bezier.isExterior(tester.getBounds())) {
				downCounter ++;
				tester.setLine(myCenter.x,myCenter.y,myCenter.x,myCenter.y-PIXELATION*downCounter);
			}
			vertical = (up||down);
		}
		if(vertical) {
			return new double[]{myCenter.x, up ? myCenter.y + PIXELATION * upCounter : down ? myCenter.y - PIXELATION * downCounter : myCenter.y};
		}
		return new double[]{left ? myCenter.x - PIXELATION * leftCounter : right ? myCenter.x + PIXELATION * rightCounter : myCenter.x, myCenter.y};
	}
	
	private void findHallowedGround(final List<Bullet> apexes) {
		final long currTime = self.getTime();

		for(final InPlight ceptor : waves) {
			if(!ceptor.isReal) {
				continue;
			}
			final double distTravelled = ceptor.velocity*(currTime - ceptor.fireTime);
			
			final Ellipse2D[] g = new Ellipse2D[]{
				new Ellipse2D.Double(
					(ceptor.sourceX() - distTravelled+0.0d*ceptor.velocity), (ceptor.sourceY() - distTravelled + 0.0d*ceptor.velocity),
					2*(distTravelled - 0.0d*ceptor.velocity), 2*(distTravelled - 0.0d*ceptor.velocity)
				),
				new Ellipse2D.Double(
					(ceptor.sourceX() - distTravelled-1.049d*ceptor.velocity), ceptor.sourceY() - distTravelled - 1.049d*ceptor.velocity,
					2*(distTravelled + 1.049d*ceptor.velocity), 2*(distTravelled+1.049d*ceptor.velocity)
				)
			};
			
			final double distance = ceptor.sourceDist(new Point2D.Double(self.getX(),self.getY()));	
			outer:
			for(final Bullet steeple : apexes) {
				final double[] xPoints = new double[4];
				final double[] yPoints = new double[4];
				final double[] xy1 = project(steeple.getX(), steeple.getY(), -0.d*steeple.getVelocity(), steeple.getHeadingRadians());
				xPoints[0] = xy1[0]; yPoints[0] = xy1[1];
				final double[] xy2 = project(steeple.getX(), steeple.getY(), 1.d*steeple.getVelocity(), steeple.getHeadingRadians());
				xPoints[1] = xy2[0]; yPoints[1] = xy2[1];
				
				final Rectangle2D bulletTrail = new Line2D.Double(xPoints[0],yPoints[0],xPoints[1],yPoints[1]).getBounds2D();
			
				if((!g[0].intersects(bulletTrail)) && g[1].intersects(bulletTrail)) {
					final double[] xy3 = project(ceptor.sourceX(), ceptor.sourceY(), 1.85d*distance, Math.atan2(xPoints[1] - ceptor.sourceX(), yPoints[1] - ceptor.sourceY()));
					xPoints[2] = xy3[0]; yPoints[2] = xy3[1];
					final double[] xy4 = project(ceptor.sourceX(), ceptor.sourceY(), 1.85d*distance, Math.atan2(xPoints[0] - ceptor.sourceX(), yPoints[0] - ceptor.sourceY()));
					xPoints[3] = xy4[0]; yPoints[3] = xy4[1];
					shadows.add(new BullegonWheels(ceptor.hashCode(),xPoints,yPoints));
				}
			}
		}
	}
	
	private void cullBullies() {
		final Iterator<BullegonWheels> bullerator = shadows.iterator();
		while(bullerator.hasNext()) {
			final BullegonWheels shade= bullerator.next();
			if(!(waves.stream().filter(w -> w.hashCode() == shade.parent).findAny().isPresent())) {
				bullerator.remove();
			}
		}
	}
	
	public void storeBullet(final Bullet b, final boolean hitUs) {
		//double x = myCenter.x, y = myCenter.y;
		//double x = b.getX(), y = b.getY();
		eHits ++;
		if(!hitUs) {
			eHits --;
		//	x = b.getX(); y = b.getY();
		}
		final InPlight incident = getWave(b);
		
		final double head = b.getHeadingRadians();
			
	//	System.out.println(Utils.normalRelativeAngle(head - b.getHeadingRadians()));
		
		if(incident != null) {
			storeWave(incident, head);
			eRangeHits ++;
			if(!hitUs || !incident.isRange) {
				eRangeHits --;
			}if (hitUs) {
				System.out.println("hit at: " + incident.getOffsetFactor(head));
			}
			incident.setInactive();
			this.activeID = 0;
			this.getSurfWave();
		} else {
			System.out.println("Lost that curvacious bullet");
		}
	}
	
	private InPlight getWave(final Bullet b) {
		final long currTime = self.getTime();
		InPlight incident = null;
		for(final InPlight wave : waves) {
			if(wave.isReal && Math.abs(wave.velocity - b.getVelocity()) < .1d && Math.abs(dist(b.getX(),b.getY(),wave.sourceX(),wave.sourceY()) - wave.velocity * (currTime - 1 - wave.fireTime)) <= 1.5d*b.getVelocity()) {
				incident = wave;
				break;
			}
		}

		return incident;
	}
	
	private void storeWave(final InPlight w, final double hitHeading) {
		offsetMap.put(
			w.key,//new float[]{self.getRoundNum(),w.fireTime,w.distSegD,w.velSegD,w.accelSegD,w.wallSpaceSegD,w.revWallSpaceSegD,w.headSegD,w.bPowerSegD},\
			//new float[]{(int)w.key[0], (int)w.key[1], (int)w.key[2], (int)w.key[3], (int)w.key[4], (int)w.key[5], (int)w.key[6]},
			w.getOffsetFactor(hitHeading)
		);
	}
	
	public float manDistBetween(final float[] entry, final float[] key) {
		float sum = 0f;
		for(int i = 0; i < key.length; i ++) {
			sum += Math.abs(entry[i+2] - key[i]);
		}
		return sum;
	}
	
	public float myManDistBetween(final float[] entry, final float[] key) {
		float sum = 0f;
		for(int i = 0; i < key.length; i ++) {
			sum += Math.abs(entry[i] - key[i]);
		}
		return sum;
	}
	
	public float eucishDistBetween(final float[] entry, final float[] key) {
		float sum = 0f;
		for(int i = 0; i < key.length; i ++) {
			sum += (entry[i+2]-key[i]) * (entry[i+2] - key[i]);
		}
		return sum;
	}
	
	public float myEucishDistBetween(final float[] entry, final float[] key) {
		float sum = 0f;
		for(int i = 0; i < key.length; i ++) {
			sum += (entry[i]-key[i])*(entry[i]-key[i]);
		}
		return sum;
	}
	
	private double[] project(final double x, final double y, final double dist, final double angle) {
		return new double[]{x + dist * Math.sin(angle), y + dist * Math.cos(angle)};
	}

	private int sign(final double value) {
		return value < 0 ? -1 : 1;
	}
	
	private double limit(final double low, final double mid, final double high) {
		return Math.max(Math.min(low,high),Math.min(mid,Math.max(low,high)));
	}
	
	private double dist(final double x1, final double y1, final double x2, final double y2) {
		return Math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	}
	
	private double toDeg(final double rad) {
		return 180d*rad/Math.PI;
	}
	
	private void determineSegmentation(final double effBearing) {
		//latDir = lastVelocity == 0 ? latDir : effBearing * lastVelocity >= 0 ? 1 : -1;
		
		this.oldDSeg = distSeg;
		this.oldVSeg = velSeg;
		this.oldASeg = accelSeg;
		this.oldWSeg = wallSpaceSeg;
		this.oldRWSeg = revWallSpaceSeg;
		this.oldHSeg = headSeg;
		this.oldBPSeg = bPowerSeg;
		
		distSeg = (float)(eDist/WALL_DIST_SEG_SIZE);
		velSeg = (float)(Math.abs(myVelocity)/(8.1d/VEL_SEGS));
		accelSeg = (float)Double.compare(Math.abs(myVelocity), Math.abs(oldVelocity));
		headSeg = (float)((Math.abs(Math.PI/2d-Math.abs(effBearing)) / (Math.PI/2d)) * HEAD_SEGS);
		
		double realHeading = self.getVelocity () < 0 ? Math.PI + self.getHeadingRadians() : self.getHeadingRadians();
		
		double wallDistLat = Utils.normalAbsoluteAngle(realHeading) < Math.PI ? (self.getBattleFieldWidth() - myCenter.x) / (Math.cos((Math.PI/2d - realHeading)))
			: self.getX() / (Math.cos((3d*Math.PI/2d - realHeading)));
		double wallDistVirt = Math.abs(Utils.normalRelativeAngle(realHeading)) < Math.PI/2d ?
			(self.getBattleFieldHeight() - self.getY()) / (Math.cos(realHeading)) : self.getY() / (Math.cos(Math.PI - realHeading));
			
		wallSpaceSeg = (float) Math.min(NEAR_WALL_SEGS,Math.max(0, Math.min(wallDistLat,wallDistVirt) / WALL_DIST_SEG_SIZE));
		
		realHeading = Utils.normalRelativeAngle(realHeading + Math.PI);
		
		wallDistLat = Utils.normalAbsoluteAngle(realHeading) < Math.PI ? (self.getBattleFieldWidth() - self.getX()) / (Math.cos((Math.PI/2d - realHeading)))
			: self.getX() / (Math.cos((3d*Math.PI/2d - realHeading)));
		wallDistVirt = Math.abs(Utils.normalRelativeAngle(realHeading)) < Math.PI/2d ?
			(self.getBattleFieldHeight() - self.getY()) / (Math.cos(realHeading)) : self.getY() / (Math.cos(Math.PI - Utils.normalAbsoluteAngle(realHeading)));
			
		revWallSpaceSeg = (float) Math.min(NEAR_WALL_SEGS,Math.max(0, Math.min(wallDistLat,wallDistVirt) / WALL_DIST_SEG_SIZE));
		
	}
	
	public void fillTrackSegments() {
		final double[] coords = new double[6];
		final PathIterator pitt = raceTrack.getPathIterator(new AffineTransform(new double[]{1,0,0,1,0,0}));
		int counter = 0, colCounter = 0;
		boolean lastWasMove = false;
		while(!pitt.isDone()) {
			final double[] ocoords = Arrays.copyOf(coords, coords.length);
			final int type = pitt.currentSegment(coords);
			switch(type) {
				case PathIterator.SEG_CLOSE:
					break;
				case PathIterator.SEG_CUBICTO://0.75d*0.013333 //1.2*0.013333333 = perfect
					for(double spot = 0; spot < 1; spot += 0.5d*1.57d*0.7d*1.66666d*0.555d*0.013333333d) {
						myCalculator.setPath(ocoords[0],ocoords[1], coords);
						final double[] nextSpot = myCalculator.calc((((spot))));
						trackPoints.add(new Point2D.Double((int)nextSpot[0], (int)nextSpot[1]));
					}
					colCounter ++;
					lastWasMove = false;
					break;
				case PathIterator.SEG_LINETO:
					if(!lastWasMove) {
						final double segLength = dist(coords[0],coords[1],ocoords[4],ocoords[5]);
						if(segLength > 1d) {
							final double segAngle = Math.atan2(coords[0]-ocoords[4],coords[1]-ocoords[5]);
							//4.48/segLength//6.4 = perfect 
							for(double spot = 0; spot < 1; spot += 4.286d/segLength) {
								trackPoints.add(
									new Point2D.Double(
										(int)(ocoords[4] + Math.sin(segAngle) * spot * segLength),
										(int)(ocoords[5] + Math.cos(segAngle) * spot * segLength)
									)
								);
							}
						}
					}
					break;
				case PathIterator.SEG_MOVETO:
					lastWasMove = true;
					break;
				case PathIterator.SEG_QUADTO:
					break;
			}
			counter ++;
			pitt.next();
		}
	}
	
	public void endRound() {
		this.isRoundOver = true;
		
		System.out.println("Real enemy acc (idgafwyt): %" + (Math.round((double)eHits/Math.max(1d,eShots)*10000d)/100d));
		System.out.println("Real enemy range accuracy: %" + (Math.round((double)eRangeHits/Math.max(1d,eRangeShots)*10000d)/100d));
		
		System.out.println("Enemy has tried to sneak " + sneakBullets + " bullets past");
	}
	
	public void expectDamage(final double damage) {
		this.expectedDamage += damage;
	}
	
	private void reset() {
		waves.clear();
		driveOptions.clear();
		shadows.clear();
		futureDrives.clear();
		neighbors.clear();
		secondNeighbors.clear();
		
		resetInactivityCounter();
		
		maxVel = 8d;
		
		if(offsetMap.size() > MAX_MAP_SIZE) {
			final List<float[]> oldKeys = offsetMap.keySet().stream()/*.sorted(
				(d1, d2) -> d1[0] == d2[0] ? Double.compare(d1[1],d2[1]) : Double.compare(d1[0],d2[0])
			)*/.limit(offsetMap.size()- SHRINK_TARGET).collect(Collectors.toList());
			System.out.println("Clearing " + (oldKeys.size()) + " surf datums");
			for(final float[] oldKey : oldKeys) {
				offsetMap.remove(oldKey);
			}
		}
		this.isRoundOver = false;
	}
	
	public void onPaint(final Graphics2D g) {
		int colCounter1 = 0;
		for(final Point2D.Double spot : trackPoints) {
			if(driveOptions.contains(spot)) {
				g.setColor(Color.GREEN.brighter().brighter());
			} else if(futureDrives.contains(spot)) {
				g.setColor(Color.CYAN.brighter());
			} 
			else  {
				g.setColor(paints[colCounter1%paints.length]);
			}
			if(spot.equals(targetPos))
				g.draw(new Ellipse2D.Double(spot.x-4,spot.y-4,8,8));
			else
				g.fill(new Ellipse2D.Double(spot.x-1.25,spot.y-1.25,2.5,2.5));
			colCounter1 ++;
		}
		final long currTime = self.getTime();
		for(final InPlight wave : waves) {
			if(wave.hashCode() == this.activeID)
				g.setColor(Color.ORANGE);
			else if(wave.hashCode() == this.secondActiveID)
				g.setColor(Color.CYAN.darker().darker().darker());
			else
				g.setColor(Color.PINK.darker().darker().darker().darker());
			g.draw(wave.setRefTime(currTime));
			final double[] futurePos = wave.project(wave.groundHeading,currTime);
			g.draw(new Line2D.Double(wave.sourceX(),wave.sourceY(),futurePos[0],futurePos[1]));
		}
		for(final BullegonWheels shader : shadows) {
			if(shader.parent == this.activeID)
				g.setColor(Color.MAGENTA);
			else
				g.setColor(Color.GREEN.darker());
			g.draw(shader);
		}
	}

}

class FutureSpot extends Point2D.Double {
	
	private double velocity;
	private long timeOfArrival;
	private int eSide, dir;
	
	FutureSpot(final Point2D.Double place, final double velocity, final long timeOfArrival, final int eSide, final int dir) {
		super(place.x, place.y);
		this.velocity = velocity;
		this.timeOfArrival = timeOfArrival;
		this.eSide = eSide;
		this.dir = dir;
	}
	
	FutureSpot(final Point2D.Double place) {
		super(place.x,place.y);
	}
	
	FutureSpot setVelocity(final double velocity) {
		this.velocity = velocity;
		return this;
	}
	
	double velocity() {
		return this.velocity;
	}
	
	FutureSpot setArrival(final long time) {
		this.timeOfArrival = time;
		return this;
	}
	
	long time() {
		return this.timeOfArrival;
	}
	
	int eSide() {
		return this.eSide;
	}
	
	void setX(final double x) {
		super.x = x;
	}
	
	void setY(final double y) {
		super.y = y;
	}
	
	int dir() {
		return this.dir;
	}
	
	FutureSpot setDir(final int dir) {
		this.dir = dir;
		return this;
	}
	
}

class InPlight {
	
	private final Ellipse2D.Double waveFront;

	private final Point2D.Double source;
	
	final double groundHeading, mEA, velocity;
	
	final long fireTime, refTime;
	
	final float[] key;
	
	final int dir;
	
	final boolean isReal, isRange;
	
	private boolean isActive;
	
	InPlight(final double sourceX, final double sourceY, final double groundHeading,
			final double velocity, long fireTime,
				final int dir, final float[] key,
			final boolean isReal, final boolean isRange) {
		this.source = new Point2D.Double(sourceX, sourceY);
		this.mEA = Math.asin(8d/velocity);
		this.velocity = velocity;
		this.groundHeading = groundHeading;
		this.fireTime = fireTime;
		this.dir = dir;
		this.isRange = isRange;
		
		this.isActive = true;
		this.isReal = isReal;
		this.waveFront = new Ellipse2D.Double(sourceX-velocity,sourceY-velocity,2*velocity,2*velocity);
		this.refTime = fireTime + 1;
		this.key = key;// new float[]{distSegD,velSegD,accelSegD,wallSpaceSegD,revWallSpaceSegD,headSegD,bPowerSegD};
	}
	
	Ellipse2D setRefTime(final long time) {
		final double distTravelled = (time - fireTime) * velocity;
		waveFront.setFrame(source.x-distTravelled,source.y-distTravelled,2d*distTravelled,2d*distTravelled);
		return waveFront;
	}
	
	double[] project(final double heading, final double futTime) {
		final double distTravelled = (futTime-fireTime) * velocity;
		return new double[]{source.x + Math.sin(heading) * distTravelled, source.y + Math.cos(heading) * distTravelled};
	}
	
	boolean checkHit(final Point2D.Double self, long currentTime) {
		return this.setRefTime(currentTime).intersects(new Rectangle2D.Double(self.x-18d,self.y-18d,36d,36d));
	}
	
	boolean breaks(final Point2D.Double self, long currentTime) {
		return source.distance(self) <= (currentTime - fireTime) * velocity;
	}
	
	long timeTillBreak(final Point2D.Double self, long currentTime) {
		return (long)Math.ceil((source.distance(self) - (currentTime-fireTime)*velocity)/velocity);
	}
	
	double getOffsetFactor(final Point2D.Double target) {
		return limit(-1d, Utils.normalRelativeAngle(Math.atan2(target.x - source.x, target.y - source.y) - groundHeading) / mEA, 1d) * dir;
	}
	
	double getOffsetFactor(final double x, final double y) {
		return limit(-1d, Utils.normalRelativeAngle(Math.atan2(x - source.x, y - source.y) - groundHeading) / mEA, 1d) * dir;
	}
	
	double getOffsetFactor(final double heading) {
		return limit(-1d, Utils.normalRelativeAngle(heading-groundHeading)/mEA, 1d) * dir;
	}

	double sourceX() {
		return source.x;
	}
	double sourceY() {
		return source.y;
	}
	void setInactive() {
		this.isActive = false;
	}
	boolean isActive() {
		return this.isActive;
	}
	double sourceDist(final Point2D.Double pos) {
		return source.distance(pos);
	}
	
	private double limit(final double low, final double mid, final double high) {
		return Math.max(low, Math.min(mid,high));
	}
	
	@Override
	public int hashCode() {
		return source.hashCode() ^ Double.valueOf(groundHeading).hashCode() ^ Double.valueOf(mEA).hashCode() ^ Double.valueOf(velocity).hashCode() ^ ((int)fireTime) ^ dir;
	}
	
	@Override
	public boolean equals(final Object o) {
		if(o instanceof InPlight) {
			final InPlight w = (InPlight)o;
			return fireTime == w.fireTime;
		} return false;
	}
	
	@Override
	public String toString() {
		return "Wave: " + fireTime + " Vel: " + velocity;
	}
}

class BullegonWheels extends Polygon2D {
	final int parent;
	boolean golden;
	BullegonWheels(final int parent, final double[] xPoints, final double[] yPoints) {
		super(xPoints, yPoints, xPoints.length);
		this.parent = parent;
	}
}

class BezierCalc {
	private final double[][] points;
	private final double[] combinations;
	BezierCalc() {
		this.points = new double[4][2];
		
		this.combinations = new double[4];
		for(int i = 0; i < combinations.length; i ++) {
			combinations[i] = factorial(3)/(factorial(i)*factorial(3-i));
		}
	}
	public void setPath(final double prevX, final double prevY, final double[] path) {
		points[0][0] = prevX;
		points[0][1] = prevY;
		System.arraycopy(path,0,this.points[1],0,2);
		System.arraycopy(path,2,this.points[2],0,2);
		System.arraycopy(path,4,this.points[3],0,2);
	}
	
	public double[] calc(final double t) {
		return
			add(
				add(
					add(
						scale(points[0], getMult(0,t)
					),
					scale(points[1], getMult(1,t))
				), scale(points[2], getMult(2, t))
			), scale(points[3], getMult(3, t))
		);
	}
	
	public double getMult(final int m, final double t) {
		return combinations[m]*Math.pow(t,m)*Math.pow(1-t,3-m);
	}
	
	public double[] scale(final double[] value, final double scalar) {
		value[0] *= scalar;
		value[1] *= scalar;
		return value;
	}
	
	public double[] add(final double[] a, final double[] b) {
		a[0] += b[0];
		a[1] += b[1];
		return a;
	}
	
	private static double factorial(int m){
        if(m<=1)
            return 1;
        return m*factorial(m-1);
    }
}