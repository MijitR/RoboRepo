package DM.mega;

import robocode.*;
import robocode.util.Utils;
import java.util.List;
import java.util.ArrayList;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.Polygon;
import java.awt.Color;
import java.util.Iterator;
import java.util.HashMap;
import java.util.stream.Stream;
import java.util.stream.Collectors;
import java.util.Arrays;
import java.util.Spliterator;
import java.util.function.*;
import java.util.stream.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.DoubleAccumulator;

/**
 * MyClass - a class by Damij
 */
public class Legs
{

public static <A, B, C> Stream<C> zip(final Stream<A> streamA, final Stream<B> streamB, final BiFunction<A, B, C> zipper) {
    final Iterator<A> iteratorA = streamA.iterator();
    final Iterator<B> iteratorB = streamB.iterator();
    final Iterator<C> iteratorC = new Iterator<C>() {
        @Override
        public boolean hasNext() {
            return iteratorA.hasNext() && iteratorB.hasNext();
        }

        @Override
        public C next() {
            return zipper.apply(iteratorA.next(), iteratorB.next());
        }
    };
    final boolean parallel = streamA.isParallel() || streamB.isParallel();
    return iteratorToFiniteStream(iteratorC, parallel);
}

public static <T> Stream<T> iteratorToFiniteStream(Iterator<T> iterator, boolean parallel) {
    final Iterable<T> iterable = () -> iterator;
    return StreamSupport.stream(iterable.spliterator(), parallel);
}


	private final AdvancedRobot self;
	
	static final double HALF_SHADE_REQ = 3d;
	
	static final int DIST_SEGS = 3, VEL_SEGS = 3, ACCEL_SEGS = 3, WALL_DIST_SEGS = 3, HEAD_SEGS = 3,
		MAX_MAP_SIZE = 371, SHRINK_TARGET = 370, MAX_FLATTEN_SIZE = 431, FLATTENER_SHRINK_TARGET = 430,
		NUM_NEIGHBORS = 17, NUM_FLATTENER_NEIGHBORS = NUM_NEIGHBORS, B_POWER_SEGS = 3, SECOND_WAVE_NEIGHBORS = NUM_NEIGHBORS,
		ACCEL_BIN = 0, DIST_BIN = 1, VEL_BIN = 2, WALL_BIN = 3, REV_WALL_BIN = 4, HEAD_BIN = 5, BULLET_BIN = 6;
	
	final HashMap<float[], Double> offsetMap;
	final HashMap<float[], Double> flattenerMap;
	
	final List<InFire> waves;
	
	final List<BullegonLegs> shades;
	final List<Point2D.Double> futures, backups;
	final List<Point2D.Double> secondFutures;
	
	
	final List<float[]> neighbors, secondNeighbors;
	final List<float[]> flatteners, secondFlatteners;
	
	final Point2D.Double enemyPos, myHead, myCenter, outerDesiredLoc;
	
	final double wallDistSegSize = 800d / (double)WALL_DIST_SEGS;
	
	int activeID, activeSecondID;
	
	double lastEHealth, oldVelocity, 
		healthDisadvantage, flattenerWeight, lastShotPower, waveBreakHeading,
		waveBreakVelocity, eAcc, enemyPosX, enemyPosY, lastAbsBearing, lastDistance, lastHeading,
		lastVelocity, eRangeAcc, oldFlattenerWeight, eOldAbsVelocity, eGunHeat, expectedDamage;
		
	float distSeg, velSeg, accelSeg, wallSpaceSeg, revWallSpaceSeg, headSeg, bPowerSeg;
		
	long firstWaveBreak, timeOfHit;
	
	int dir, lastDir, latDir, goldenShades, emptyDir, sneakBullets,
		eShots, eHits, eSide, eRangeShots, eRangeHits;
	
	boolean swapped, swappedDualSurf, updated, surfTwo, beganDualWave, updatingPoints = true, isRoundEnded = false;
	
	public Legs(final AdvancedRobot self) {
		this.self = self;
		this.offsetMap = new HashMap<>();
		this.flattenerMap = new HashMap<>();
		this.waves = new ArrayList<>();
		this.shades = new ArrayList<>();
		this.futures = new ArrayList<>();
		this.secondFutures = new ArrayList<>();
		this.neighbors = new ArrayList<>();
		this.secondNeighbors = new ArrayList<>();
		this.flatteners = new ArrayList<>();
		this.secondFlatteners = new ArrayList<>();
		this.backups = new ArrayList<>();
		this.enemyPos = new Point2D.Double(0,0);
		this.myHead = new Point2D.Double(0,0);
		this.myCenter = new Point2D.Double(0,0);
		this.outerDesiredLoc = new Point2D.Double(self.getX(),self.getY());
		this.latDir = 1; this.emptyDir = 1;
		this.dir = 1; this.lastDir = 1;
		this.eSide = 1;
		this.flattenerWeight =  3d;
		this.lastShotPower = 1.7d;
	}
	
	public void update(final ScannedRobotEvent e, final List<Bullet> umbrellas, final double absBearing) {
	
		if(this.isRoundEnded) {
			reset();
		}
	
		boolean hiding=false;
		for(final BullegonLegs hider : shades) {
			if(hider.parent == this.activeID && hider.contains(new Point2D.Double(self.getX(),self.getY()))) {
				hiding = true; break;
			}
		}
		
		if(hiding && !swapped) {
			self.setColors(Color.ORANGE.darker().darker().darker().darker().darker(),Color.RED.darker().darker().darker().darker(),Color.MAGENTA.darker().darker().darker().darker());
			swapped = true;
		} else if(!hiding && swapped) {
			self.setColors(Color.ORANGE.brighter().brighter(),Color.RED.brighter().brighter().brighter(),Color.MAGENTA.brighter().brighter());
			swapped = false;
		}
		
		//System.out.println("omap: " + offsetMap.size());
		//System.out.println("fmap: " + flattenerMap.size());
		//System.out.println("wave: " + waves.size());

		this.eAcc = eHits/Math.max(eShots,1d);
		this.eRangeAcc = eRangeHits/Math.max(eRangeShots,1d);
		this.flattenerWeight = 0.0d;// eRangeShots < 50 ? 0d : /*limit(0d,Math.max(0d,(eRangeAcc-0.081d))/0.009, 1d)*/0.4618d*(Math.min((double)NUM_NEIGHBORS, (double)offsetMap.size()+1d)/(double)NUM_FLATTENER_NEIGHBORS);
		//this.myHead.setLocation(project(new Point2D.Double(self.getX(),self.getY()), 135d*sign(self.getVelocity()), self.getHeadingRadians()));
		final long currTime = self.getTime();
		
		enemyPosX = lastDistance * Math.sin(lastAbsBearing) + myCenter.x;
		enemyPosY = lastDistance * Math.cos(lastAbsBearing) + myCenter.y;
		
		//latDir = Utils.normalRelativeAngle(absBearing-lastAbsBearing) > 0 ? 1 : -1;
		//lastDir = dir;
		//dir = self.getVelocity() == 0 ? dir : Double.compare(self.getVelocity(),0);
		latDir = self.getVelocity() == 0 ? latDir : e.getBearingRadians() * self.getVelocity() >= 0 ? 1 : -1;
		
		eSide = Double.compare(e.getBearingRadians(),0) == 0 ? eSide : Double.compare(e.getBearingRadians(), 0);
		
		lastAbsBearing = absBearing;
		lastDistance = e.getDistance();
		lastHeading = e.getHeadingRadians();
		lastVelocity = self.getVelocity();
		
		determineSegmentation(e.getBearingRadians());
		
		/*if(updated) {
			System.arraycopy(activeColorBuffer, 0, activeColor, 0, activeColorBuffer.length);
		}
		if(Double.compare(flattenerWeight, oldFlattenerWeight) != 0) {
			updated = true;
			activeColor[0] = new Color(activeColorBuffer[0].getRed() + (int)((255-activeColorBuffer[0].getRed())*flattenerWeight),
					activeColorBuffer[0].getGreen() + (int)((255-activeColorBuffer[0].getGreen())*flattenerWeight),
							activeColorBuffer[0].getBlue() + (int)((255-activeColorBuffer[0].getBlue())*flattenerWeight));
			activeColor[1] = new Color(activeColorBuffer[1].getRed() + (int)((255-activeColorBuffer[1].getRed())*flattenerWeight),
					activeColorBuffer[1].getGreen() + (int)((255-activeColorBuffer[1].getGreen())*flattenerWeight),
							activeColorBuffer[1].getBlue() + (int)((255-activeColorBuffer[1].getBlue())*flattenerWeight));
			activeColor[2] = new Color(activeColorBuffer[2].getRed() + (int)((255-activeColorBuffer[2].getRed())*flattenerWeight),
					activeColorBuffer[2].getGreen() + (int)((255-activeColorBuffer[2].getGreen())*flattenerWeight),
							activeColorBuffer[2].getBlue() + (int)((255-activeColorBuffer[2].getBlue())*flattenerWeight));
		}*/
	
		oldFlattenerWeight = flattenerWeight;
		
		eGunHeat -= self.getGunCoolingRate();
		
		//if(hiding && (swapped || updated)) {
		//	self.setColors(activeColor[0].darker().darker().darker(),activeColor[1].darker().darker().darker(),activeColor[2].darker().darker().darker());
		//	swapped = false;
		//} else if ((!hiding && !swapped) || updated) {
		//	self.setColors(activeColor[0],activeColor[1],activeColor[2]);
		//	swapped = true;
		//}
		
		enemyPos.x = enemyPosX;
		enemyPos.y = enemyPosY;
		
		myCenter.x = self.getX();
		myCenter.y = self.getY();
		
		final double eDelta = lastEHealth - (e.getEnergy() + expectedDamage);
		if(eDelta <= 3 && eDelta > 0.1d && eGunHeat <= 0 && Math.abs(Math.abs(e.getVelocity())-eOldAbsVelocity) <= 2.05d) {
			bPowerSeg = (float)(eDelta/3f*B_POWER_SEGS);
			waves.add(new InFire(
				enemyPos.x, enemyPos.y, Math.atan2(myCenter.x-enemyPos.x,myCenter.y-enemyPos.y), Rules.getBulletSpeed(eDelta), currTime-1, latDir,
					distSeg, velSeg, accelSeg, wallSpaceSeg, revWallSpaceSeg, headSeg, true, bPowerSeg, enemyPos.distance(myCenter) > 450
			));
			lastShotPower = eDelta;
			eGunHeat = Rules.getGunHeat(eDelta);
			if(expectedDamage > 0) {
				sneakBullets ++;
			}
		} else if(!beganDualWave && eAcc > 0.11d && eShots > 70 && flattenerWeight > 0) {
			bPowerSeg = (float)(lastShotPower/3d*B_POWER_SEGS);
			waves.add(new InFire(
				enemyPos.x, enemyPos.y, Math.atan2(myCenter.x-enemyPos.x,myCenter.y-enemyPos.y), Rules.getBulletSpeed(eDelta), currTime-1, latDir,
					distSeg, velSeg, accelSeg, wallSpaceSeg, revWallSpaceSeg, headSeg, false, bPowerSeg, false
			));
		}
	//	if(expectedDamage != 0) {
	//		System.out.println("Expected damage with gun heat : " + eGunHeat);
	//		System.out.println("Expected damage: " + expectedDamage);
	//		System.out.println(eDelta);
			
	//	}
		this.expectedDamage = 0d;
		
		this.shades.addAll(findHallowedGround(umbrellas));
		
		lastEHealth = e.getEnergy();
		
		cull(shades);
		self.setMaxVelocity(8d);
		
		final InFire surfWave = getSurfWave();
		
			if(//!offsetMap.isEmpty()&&
				surfWave!=null&&enemyPos.distance(myCenter)>36d+68d) {
			final float[] mikey = new float[]{surfWave.accelSegD, surfWave.distSegD, surfWave.velSegD,
			surfWave.wallSpaceSegD, surfWave.revWallSpaceSegD, surfWave.headSegD, surfWave.bPowerSegD};
				this.emptyDir = latDir;
				final InFire secondWave = getSecondWave();
				
			if(updatingPoints) {
				
			//futures.clear();
			//backups.clear();
			
			//futures.addAll(surfFromTill(myCenter, surfWave, self.getHeadingRadians(), self.getVelocity(), self.getTime()));
			//backups.addAll(futures);
				neighbors.clear();
				secondNeighbors.clear();
				secondFlatteners.clear();
				neighbors.addAll(offsetMap.keySet().stream().sorted((d1,d2)
				-> Double.compare(eucishDistBetween(d1,mikey),eucishDistBetween(d2,mikey))
				).limit(NUM_NEIGHBORS).collect(Collectors.toList()));
				flatteners.clear();
				flatteners.addAll(flattenerMap.keySet().stream().sorted((d1,d2)->
					Double.compare(eucishDistBetween(d1,mikey),eucishDistBetween(d2,mikey))).limit(NUM_FLATTENER_NEIGHBORS).collect(Collectors.toList()));
			
			final float[] mikey2;
			if(secondWave != null) {
				mikey2 = new float[]{secondWave.accelSegD, secondWave.distSegD, secondWave.velSegD,
				secondWave.wallSpaceSegD, secondWave.revWallSpaceSegD, secondWave.headSegD, secondWave.bPowerSegD};
				secondNeighbors.addAll(offsetMap.keySet().stream().sorted((d1,d2)
						-> Double.compare(eucishDistBetween(d1,mikey2),eucishDistBetween(d2,mikey2))
					).limit(NUM_NEIGHBORS).collect(Collectors.toList()));
				secondFlatteners.addAll(flattenerMap.keySet().stream().sorted((d1,d2)->
					Double.compare(eucishDistBetween(d1,mikey2),eucishDistBetween(d2,mikey2))).limit(NUM_FLATTENER_NEIGHBORS).collect(Collectors.toList()));
			} else {
				mikey2 = null;
			}
			

			
}
			futures.clear();
			backups.clear();
			futures.addAll(surfFromTill(myCenter, surfWave, self.getHeadingRadians(), self.getVelocity(), self.getTime()));
			backups.addAll(futures);
			final Point2D.Double desiredLoc = (secondWave == null || (eRangeHits < 55)) ?
				getLeastDangerous(surfWave, mikey) : getLeastDangerousIsh(surfWave, mikey, secondWave);
		
			outerDesiredLoc.x = desiredLoc.x;
			outerDesiredLoc.y = desiredLoc.y; 
			
				
		driveTo(outerDesiredLoc);
		
			}
			else {
				emptySurf(enemyPos);
			}
		
		oldVelocity = lastVelocity;
		eOldAbsVelocity = Math.abs(e.getVelocity());
		
		
	}
	


	
	private void emptySurf(final Point2D.Double centroid) {
		flatteners.clear();
		secondNeighbors.clear();
		secondFlatteners.clear();
		futures.clear();
		secondFutures.clear();
		final double myAbsBearing = Math.atan2(centroid.x-myCenter.x,centroid.y-myCenter.y);
		final double bearing = Utils.normalRelativeAngle(myAbsBearing - self.getHeadingRadians());
		int currVelDir = emptyDir * sign(bearing);
		final double desiredAngle =  wallSmooth(myCenter, latDir, Utils.normalAbsoluteAngle(myAbsBearing-Math.PI/2d*sign(bearing)-latDir * (Math.PI*0.4d*(1d-enemyPos.distance(myCenter)/ClownCar.MAX_DIST))), currVelDir);
		final Point2D.Double nextPos = project(myCenter, 135d*currVelDir, desiredAngle);
		
		int foll = 1;
		if((!ClownCar.surfTains(nextPos) && nextPos.distance(enemyPos) < myCenter.distance(enemyPos)) || nextPos.distance(enemyPos) + 37d < myCenter.distance(enemyPos)) {
			foll = -1;
		} currVelDir *= foll;
		emptyDir *= foll;
		
		double turnAmount = Utils.normalRelativeAngle(desiredAngle-self.getHeadingRadians());
	
		self.setTurnRightRadians(turnAmount);
		self.setAhead(100d*currVelDir);
		self.setMaxVelocity(offsetMap.isEmpty()?4d:8d);
	}
	





	private void determineSegmentation(final double effBearing) {
		//latDir = lastVelocity == 0 ? latDir : effBearing * lastVelocity >= 0 ? 1 : -1;
		
		distSeg = (float)(lastDistance/wallDistSegSize);
		velSeg = (float)(Math.abs(lastVelocity)/(8.1d/VEL_SEGS));
		accelSeg = (float)Double.compare(Math.abs(lastVelocity), Math.abs(oldVelocity)) + 1;
		headSeg = (float)((Math.abs(Math.PI/2d-Math.abs(effBearing)) / (Math.PI/2d)) * HEAD_SEGS);
		
		double realHeading = self.getVelocity () < 0 ? Math.PI + self.getHeadingRadians() : self.getHeadingRadians();
		
		double wallDistLat = Utils.normalAbsoluteAngle(realHeading) < Math.PI ? (self.getBattleFieldWidth() - myCenter.x) / (Math.cos((Math.PI/2d - realHeading)))
			: self.getX() / (Math.cos((3d*Math.PI/2d - realHeading)));
		double wallDistVirt = Math.abs(Utils.normalRelativeAngle(realHeading)) < Math.PI/2d ?
			(self.getBattleFieldHeight() - self.getY()) / (Math.cos(realHeading)) : self.getY() / (Math.cos(Math.PI - realHeading));
			
		wallSpaceSeg = (float)Math.min(WALL_DIST_SEGS,Math.max(0, Math.min(wallDistLat,wallDistVirt) / wallDistSegSize));
		
		realHeading = Utils.normalRelativeAngle(realHeading + Math.PI);
		
		wallDistLat = Utils.normalAbsoluteAngle(realHeading) < Math.PI ? (self.getBattleFieldWidth() - self.getX()) / (Math.cos((Math.PI/2d - realHeading)))
			: self.getX() / (Math.cos((3d*Math.PI/2d - realHeading)));
		wallDistVirt = Math.abs(Utils.normalRelativeAngle(realHeading)) < Math.PI/2d ?
			(self.getBattleFieldHeight() - self.getY()) / (Math.cos(realHeading)) : self.getY() / (Math.cos(Math.PI - Utils.normalAbsoluteAngle(realHeading)));
			
		revWallSpaceSeg = (float) Math.min(WALL_DIST_SEGS,Math.max(0, Math.min(wallDistLat,wallDistVirt) / wallDistSegSize));
	}
	



	private InFire getSurfWave() {
		final long currTime = self.getTime();
		double minHitTime = Double.POSITIVE_INFINITY;
		InFire surfWave = null;
		final Iterator<InFire> rayinator = waves.iterator();
		while (rayinator.hasNext()) {
			final InFire w = rayinator.next();
			double hitTime = w.sourceDist(new Point2D.Double(self.getX(),self.getY()))/w.velocity - (currTime - w.fireTime);
			if(hitTime <= -3 && w.isReal) {
				rayinator.remove();
				eShots ++;
				eRangeShots ++;
				if(!w.isRange) {
					eRangeShots --;
				}
				continue;
			} if(w.checkHit(myCenter,currTime)&&w.isActive()) {
				flatten(w);
				w.setInactive();
			}
			else if(hitTime < minHitTime && hitTime > 1 && w.isActive() && w.isReal) {
				minHitTime = hitTime;
				surfWave = w;
				this.timeOfHit = (long)hitTime;
			}
		}
		if(surfWave != null && (updatingPoints = (surfWave.hashCode() != this.activeID))) {
				
			this.activeID = surfWave.hashCode();
		} else {
			updatingPoints = false;
		}
		return surfWave;
	}
	
	private InFire getSecondWave() {
		InFire secondWave = null;
		//final float[] key = new float[]{surfWave.accelSegD, surfWave.distSegD, surfWave.velSegD,
		//	surfWave.wallSpaceSegD, surfWave.revWallSpaceSegD, surfWave.headSegD, surfWave.bPowerSegD};
		final long currTime = self.getTime();
		double minHitTime = Double.POSITIVE_INFINITY;
		final Iterator<InFire> rayinator = waves.iterator();
		while (rayinator.hasNext()) {
			final InFire w = rayinator.next();
			double hitTime = w.sourceDist(new Point2D.Double(self.getX(),self.getY()))/w.velocity - (currTime - w.fireTime);
			if(w.hashCode() != this.activeID && hitTime < minHitTime && hitTime > this.timeOfHit && w.isActive() && w.isReal) {
				minHitTime = hitTime;
				secondWave = w;
				activeSecondID = w.hashCode();
			}
		}
		return secondWave;
	}
	




	private List<Point2D.Double> surfFromTill(final Point2D.Double startPos, final InFire wave, final double startHeading, final double startVelocity, final long startTime) {
		final List<Point2D.Double> myFutures = new ArrayList<>();
		double maxVel = 8d;
		float increment = 1f;
		if(startTime!=self.getTime()) {
			maxVel = 7d;
			increment = 1f;
		}
		//if(this.activeSecondID == wave.hashCode()) {
		//	maxVel = 6d;
		//}
		
		Point2D.Double nextPos = new Point2D.Double(startPos.x,startPos.y);
	
		boolean stopped = Double.compare(Math.abs(startVelocity), 0) <= 0;
		
//	if(stopped)
			myFutures.add(nextPos);

		for(int absDir = sign(startVelocity), actDir = sign(startVelocity), accel = absDir * actDir, a = 0; a < 2; absDir *= -1, a ++,  actDir = sign(startVelocity)) {
			nextPos.x = startPos.x;
			nextPos.y = startPos.y;
			double absAngleToRot = Math.atan2(wave.sourceX() - nextPos.x, wave.sourceY() - nextPos.y);
			

			double bearingToRot = Utils.normalRelativeAngle(absAngleToRot - startHeading);
			
		
		//	int inspireDir = (dist(nextPos.x,nextPos.y,wave.sourceX(),wave.sourceY()) < self.getBattleFieldWidth()/2d ? -1 : 1);

			int eSide = sign(bearingToRot);
			int absLatDir = absDir * eSide;//sign(bearingToRot);
			int actLatDir = actDir * eSide;
		
			double desiredHeading = wallSmooth(nextPos, absLatDir,
					absAngleToRot - Math.PI/2d * eSide - (Math.abs(startVelocity)/4d *(//+ absLatDir * Rules.MAX_TURN_RATE_RADIANS * 0.6d * (healthDisadvantage+3d))
							+ absLatDir * (Math.PI*0.2d*(1d-enemyPos.distance(startPos)/ClownCar.MAX_DIST))))
				,absDir);
				
			/*int goDir = 1;
			if(Math.abs(Utils.normalRelativeAngle(desiredHeading - startHeading)) > Math.PI/2d) {
				goDir = -1;
			}*/
			
			double predVelocity = accel < 0 ? startVelocity +2d*absDir : startVelocity + absDir;
			predVelocity = limit(-maxVel, predVelocity, maxVel);
			float counter = 0;
			long predTime = startTime + (long) counter;
			nextPos = project(nextPos,predVelocity,desiredHeading);
		
			
			myFutures.add(nextPos);
			
			 while(!wave.breaks(nextPos, predTime)) {
				stopped = Double.compare(predVelocity, 0) == 0;

				absAngleToRot = Math.atan2(wave.sourceX() - nextPos.x, wave.sourceY() - nextPos.y);
				bearingToRot = Utils.normalRelativeAngle(absAngleToRot - desiredHeading);
			
				eSide = sign(bearingToRot);
				absLatDir = absDir*eSide;
				actDir = stopped ? actDir : sign(predVelocity);
				actLatDir = actDir * eSide;
				accel = absDir * actDir;
		
				absLatDir = absDir*eSide;//sign(bearingToRot);
				
				//desiredHeading = wallSmooth(nextPos, actLatDir
				//		, absAngleToRot - Math.PI/2d  * sign(bearingToRot) *  actDir + (stopped ? 0d : - actLatDir * Rules.MAX_TURN_RATE_RADIANS  * 0.3d * (healthDisadvantage+2.7d))
				//			- actLatDir * (0.2d*Math.PI* (1d-enemyPos.distance(nextPos)/ClownCar.MAX_DIST)), actDir);
				desiredHeading = wallSmooth(nextPos, absLatDir,
					absAngleToRot - Math.PI/2d * eSide - (Math.abs(predVelocity)/4d*( //+ absDir * Rules.MAX_TURN_RATE_RADIANS * 0.6d * (healthDisadvantage+3d))
							+  absLatDir * (Math.PI*0.2d*(1d-enemyPos.distance(startPos)/ClownCar.MAX_DIST))))
				,absDir);
				predVelocity += accel < 0 ? 2*absDir : absDir;
				predVelocity = limit(-maxVel, predVelocity, maxVel);
				
				nextPos = (project(nextPos,predVelocity,desiredHeading));
				
//add precise prediction / real spot pair to list

				myFutures.add(nextPos);
				//predTime ++;
				counter += increment;
				predTime = startTime + (long)counter;
			}
		}
		
		return myFutures;
	}
	
	private Point2D.Double getLeastDangerousIsh(final InFire surfWave, final float[] key) {
		double minDensity = Double.MAX_VALUE;
		final Point2D.Double lD = new Point2D.Double(myCenter.x,myCenter.y);
		for(final Point2D.Double future : futures) {
			final double[] stats = predictToWith(future, surfWave);
			Point2D.Double realFuture = new Point2D.Double(stats[0],stats[1]);
			double density = 0d;
				boolean contained = false;
			for(final BullegonLegs hider : shades) {
				if(hider.parent == surfWave.hashCode()
					&& hider.contains(new Rectangle2D.Double(realFuture.x-HALF_SHADE_REQ,realFuture.y-HALF_SHADE_REQ,2d*HALF_SHADE_REQ,2d*HALF_SHADE_REQ))) {
					if(!hider.golden) {
						goldenShades ++;
					}
					hider.golden = true;
					contained = true;
					break;
				}
			}
			double botWidthAngle = 27d/dist(surfWave.sourceX(), surfWave.sourceY(), realFuture.x, realFuture.y);//.distance(future);
			final double projectAngle = Math.atan2(realFuture.x-surfWave.sourceX(), realFuture.y-surfWave.sourceY());
			 if (!contained ) {
			for(final float[] nay : neighbors) {
				double neighborFactor = offsetMap.get(nay); //  20 - 3 * bPower
				double aimAngle = neighborFactor * surfWave.mEA *surfWave.dir + surfWave.groundHeading;
				double diff = Utils.normalRelativeAngle(aimAngle-projectAngle)/botWidthAngle;
				//I'm using a different method here than used for sorting
				density += Math.exp(-0.5d*diff*diff) ;/// manDistBetween(nay, key);
			}
			}
			for(final float[] flatty : flatteners) {
				double neighborFactor = flattenerMap.get(flatty);
				double aimAngle = neighborFactor*surfWave.mEA*surfWave.dir + surfWave.groundHeading;
				double diff = Utils.normalRelativeAngle(aimAngle-projectAngle)/botWidthAngle;
				density += Math.exp(-0.5d*diff*diff) * flattenerWeight;	
			}
			if(density < minDensity) {
				minDensity = density;
				lD.x = future.x;
				lD.y = future.y;
			}
			future.x = realFuture.x;
			future.y = realFuture.y;
		}
		return lD;
	}
	
	private Point2D.Double getLeastDangerousIsh(final InFire surfWave, final float[] key, final InFire secondWave) {
		beganDualWave = true;
		double minDensity = Double.MAX_VALUE;
		final Point2D.Double lD = new Point2D.Double(myCenter.x,myCenter.y);
			final float[] mikey2 = new float[]{secondWave.accelSegD, secondWave.distSegD, secondWave.velSegD,
				secondWave.wallSpaceSegD, secondWave.revWallSpaceSegD, secondWave.headSegD, secondWave.bPowerSegD};
		for(final Point2D.Double future : futures) {
			final double[] nextStats =  predictToWith(future, surfWave);
			double density = 0d;
			Point2D.Double realFuture = new Point2D.Double(nextStats[0],nextStats[1]);
				boolean contained = false;
		for(final BullegonLegs hider : shades) {
				if(hider.parent == surfWave.hashCode()
					&& hider.contains(new Rectangle2D.Double(realFuture.x-HALF_SHADE_REQ,realFuture.y-HALF_SHADE_REQ,2d*HALF_SHADE_REQ,2d*HALF_SHADE_REQ))) {
					if(!hider.golden) {
						goldenShades ++;
					}
					hider.golden = true;
					contained = true;
					break;
				}
			}

			double botWidthAngle  = 27d/dist(surfWave.sourceX(), surfWave.sourceY(), realFuture.x, realFuture.y);//.distance(realFuture);
			final double projectAngle = Math.atan2(realFuture.x-surfWave.sourceX(), realFuture.y-surfWave.sourceY());
			
			 if (!contained ) {
			for(final float[] nay : neighbors) {
				double neighborFactor = offsetMap.get(nay); //  20 - 3 * bPower
				double aimAngle = neighborFactor * surfWave.mEA*surfWave.dir + surfWave.groundHeading;
				double diff = Utils.normalRelativeAngle(aimAngle-projectAngle)/botWidthAngle;
				//I'm using a different method here than used for sorting
				density += Math.exp(-0.5d*diff*diff) / manDistBetween(nay, key);
			}
			}
			if(flattenerWeight > 0) {
			for(final float[] flatty : flatteners) {
				double neighborFactor = flattenerMap.get(flatty);
				double aimAngle = neighborFactor * surfWave.mEA*surfWave.dir + surfWave.groundHeading;
				double diff = Utils.normalRelativeAngle(aimAngle-projectAngle)/botWidthAngle;
				density += Math.exp(-0.5d*diff*diff) / manDistBetween(flatty, key);
			}
			}
			
			if(density > minDensity) {
				future.x = realFuture.x;
				future.y = realFuture.y;
				continue;
			}
			
			
			final List<Point2D.Double>  megaFutures = surfFromTill(new Point2D.Double(nextStats[0],nextStats[1]), secondWave, nextStats[2], nextStats[3], (long) nextStats[4]);
			double minSecondDensity = Double.MAX_VALUE;
			//powerFrac = (secondWave.velocity-20d)/-3d/((surfWave.velocity-20d)/-3d);
			final double secondWaveScalar = (secondWave.velocity-20d)/-3d/((surfWave.velocity-20d)/-3d) / Math.max(1d, 2d*Math.log1p(surfWave.timeTillBreak(myCenter, self.getTime())));
			for(final Point2D.Double megaFut : megaFutures) {
				botWidthAngle  = 27d/dist(surfWave.sourceX(), surfWave.sourceY(), megaFut.x, megaFut.y);
				double localDensity = 0d;
				botWidthAngle = 36d/dist(secondWave.sourceX(),secondWave.sourceY(), megaFut.x, megaFut.y);
				final double myProjection = Math.atan2(megaFut.x-surfWave.sourceX(), megaFut.y-surfWave.sourceY());
				for(final float[] nay : secondNeighbors) {
					double neighborFactor = offsetMap.get(nay);
					double aimAngle = neighborFactor * secondWave.mEA*secondWave.dir + secondWave.groundHeading;
					double diff = Utils.normalRelativeAngle(aimAngle-myProjection)/botWidthAngle;
					localDensity += Math.exp(-0.5*diff*diff) / manDistBetween(nay, mikey2);
				}
				if(flattenerWeight > 0d) {
					for(final float[] flatty : secondFlatteners) {
						double neighborFactor = flattenerMap.get(flatty);
						double aimAngle = neighborFactor * secondWave.mEA*secondWave.dir + secondWave.groundHeading;
						double diff = Utils.normalRelativeAngle(aimAngle-myProjection)/botWidthAngle;
						localDensity += Math.exp(-0.5*diff*diff) / manDistBetween(flatty, mikey2) * flattenerWeight;
					}
				}
				minSecondDensity = Math.min(minSecondDensity, localDensity * secondWaveScalar);
			}
			density += minSecondDensity;
			
			if(density < minDensity) {
				secondFutures.clear();
				secondFutures.addAll(megaFutures);
				minDensity = density;
				lD.x = future.x;
				lD.y = future.y;
			}
			
			future.x = realFuture.x;
			future.y = realFuture.y;
								
		}
		return lD;
	}

	private Point2D.Double getLeastDangerous(final InFire surfWave, final float[] key) {
		//neighbors.clear();
		//flatteners.clear();
		secondNeighbors.clear();
		secondFlatteners.clear();
		//ACCEL_BIN = 0, DIST_BIN = 1, VEL_BIN = 2, WALL_BIN = 3, REV_WALL_BIN = 4, HEAD_BIN = 5, BULLET_BIN = 6;
		//final float[] key = new float[]{surfWave.accelSegD, surfWave.distSegD, surfWave.velSegD,
		//	surfWave.wallSpaceSegD, surfWave.revWallSpaceSegD, surfWave.headSegD, surfWave.bPowerSegD};
			
		//final double stDev = 2d*Math.atan(18d/dist(myCenter.x,myCenter.y,surfWave.sourceX(),surfWave.sourceY()))/1.7d;//(Math.atan(18d/dist(surfWave.sourceX(),surfWave.sourceY(),self.getX(),self.getY())));
		
		final AtomicInteger nayFut = new AtomicInteger(-1), nayCounter = new AtomicInteger(-1);

		final double[] futDangers = new double[futures.size()];

		//final int nayCounts = futures.size()/Math.min(offsetMap.size(), NUM_NEIGHBORS);
		
		final long currTime = self.getTime();
		
		for(final Point2D.Double future : futures) {
			final double[] dat = predictToWith(future, surfWave);
			future.x = dat[0];
			future.y = dat[1];	
		}
		

		/*offsetMap.keySet().stream().sorted((d1,d2)
				-> Double.compare(manDistBetween(d1,key),manDistBetween(d2,key))
				).limit(NUM_NEIGHBORS).flatMapToDouble(n -> {
					final double bor = offsetMap.get(n); return futures.stream().parallel()//.map(f->predictToWith(f, surfWave))
					.mapToDouble(f->{ if(shades.stream().filter(
							s->s.parent == surfWave.hashCode() && s.contains(new Rectangle2D.Double(f.x-5d,f.y-5d,10d,10d))).findAny().isPresent()) return 0d;
							final double offsetFactor = surfWave.getOffsetFactor(f); return Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev)));
				});
			}).forEach(n1f1 -> futDangers[nayFut.incrementAndGet()%futDangers.length] += n1f1);
					;*/
		//if(!neighbors.isEmpty() && !flatteners.isEmpty()) {
		final double weightFinaly = flattenerWeight;
		if(!flatteners.isEmpty() && flattenerWeight > 0d) {
			if(!neighbors.isEmpty()) {
		zip(neighbors.stream(),
			flatteners.stream(), (n1, n2) -> new float[][]{n1,n2}).flatMapToDouble(n-> {
					final int nayCount = nayCounter.incrementAndGet();
					final double bor1 = offsetMap.get(n[0]); final double bor2 = flattenerMap.get(n[1]); return futures.stream()
					.mapToDouble(f->{
							final double offsetFactor = surfWave.getOffsetFactor(f);
							final double dist;
							final double stDev = 2d*Math.atan(18d/(dist=dist(f.x,f.y,surfWave.sourceX(),surfWave.sourceY()))) / surfWave.mEA;
							final double distanceMain = eucishDistBetween(n[0], key), distanceFlat = eucishDistBetween(n[1], key);
							if(shades.stream().filter(
								s->s.parent == surfWave.hashCode() && s.contains(new Rectangle2D.Double(f.x-HALF_SHADE_REQ,f.y-HALF_SHADE_REQ,24.2d*HALF_SHADE_REQ,2d*HALF_SHADE_REQ))).findAny().isPresent())
									return weightFinaly * Math.exp(-0.5d*((offsetFactor-bor2)*(offsetFactor-bor2)/(stDev*stDev))) / distanceFlat / dist;
							final double runner = 1d; /*Math.pow(0.95d, nayCount);*/return runner*(Math.exp(-0.5d*((offsetFactor-bor1)*(offsetFactor-bor1)/(stDev*stDev))) / distanceMain / dist
									+ runner * weightFinaly * Math.exp(-0.5d*((offsetFactor-bor2)*(offsetFactor-bor2)/(stDev*stDev)))) / distanceFlat / dist;
				});
			}).forEach(n1f1 -> futDangers[nayFut.incrementAndGet()%futDangers.length] += n1f1);
					;
			} else {
				flatteners.stream().flatMapToDouble(n-> {
					final int nayCount = nayCounter.incrementAndGet();
					final double bor1 = flattenerMap.get(n); return futures.stream()
					.mapToDouble(f->{
							double dist;
							final double offsetFactor = surfWave.getOffsetFactor(f);
							final double stDev = 2d*Math.atan(18d/(dist=dist(f.x,f.y,surfWave.sourceX(),surfWave.sourceY()))) / surfWave.mEA;
							final double distanceMain = eucishDistBetween(n, key);
							if(shades.stream().filter(
								s->s.parent == surfWave.hashCode() && s.contains(new Rectangle2D.Double(f.x-HALF_SHADE_REQ,f.y-HALF_SHADE_REQ,24.2d*HALF_SHADE_REQ,2d*HALF_SHADE_REQ))).findAny().isPresent())
									return 0d;
							final double runner = 1d; /*Math.pow(0.95d, nayCount);*/return runner*Math.exp(-0.5d*((offsetFactor-bor1)*(offsetFactor-bor1)/(stDev*stDev))) / distanceMain/ dist;
				});
			}).forEach(n1f1 -> futDangers[nayFut.incrementAndGet()%futDangers.length] += n1f1);
					;
			}
		} else {
			neighbors.stream().flatMapToDouble(n-> {
					final int nayCount = nayCounter.incrementAndGet();
					final double bor1 = offsetMap.get(n); return futures.stream()
					.mapToDouble(f->{
							double dist;
							final double offsetFactor = surfWave.getOffsetFactor(f);
							final double stDev = 2d*Math.atan(18d/(dist=dist(f.x,f.y,surfWave.sourceX(),surfWave.sourceY()))) / surfWave.mEA;
							final double distanceMain = eucishDistBetween(n, key);
							if(shades.stream().filter(
								s->s.parent == surfWave.hashCode() && s.contains(new Rectangle2D.Double(f.x-HALF_SHADE_REQ,f.y-HALF_SHADE_REQ,24.2d*HALF_SHADE_REQ,2d*HALF_SHADE_REQ))).findAny().isPresent())
									return 0d;
							final double runner = 1d; /*Math.pow(0.95d, nayCount);*/return runner*Math.exp(-0.5d*((offsetFactor-bor1)*(offsetFactor-bor1)/(stDev*stDev))) / distanceMain / dist;
				});
			}).forEach(n1f1 -> futDangers[nayFut.incrementAndGet()%futDangers.length] += n1f1);
					;
		}
		//}

		final double minDanger = Double.POSITIVE_INFINITY;
		 
		int masterDex = 0;
		for(int fud = 0; fud < futDangers.length; fud ++) {
			if(futDangers[fud] < futDangers[masterDex]) {
				masterDex = fud;
			}
		}
		
		return backups.get(masterDex);
	}



	private Point2D.Double getLeastDangerous(final InFire surfWave, final InFire secondWave) {
		beganDualWave = true;
		neighbors.clear();
		flatteners.clear();
		secondNeighbors.clear();
		secondFlatteners.clear();
		secondFutures.clear();
		//ACCEL_BIN = 0, DIST_BIN = 1, VEL_BIN = 2, WALL_BIN = 3, REV_WALL_BIN = 4, HEAD_BIN = 5, BULLET_BIN = 6;
		final float[] key = new float[]{surfWave.accelSegD, surfWave.distSegD, surfWave.velSegD,
			surfWave.wallSpaceSegD, surfWave.revWallSpaceSegD, surfWave.headSegD, surfWave.bPowerSegD};
			
		final float[] futKey = new float[]{secondWave.accelSegD, secondWave.distSegD, secondWave.velSegD,
			secondWave.wallSpaceSegD, secondWave.revWallSpaceSegD, secondWave.headSegD, secondWave.bPowerSegD};
			
		//final double stDev = 2d*Math.atan(18d/dist(myCenter.x,myCenter.y,surfWave.sourceX(),surfWave.sourceY()))/1.7d;//(Math.atan(18d/dist(surfWave.sourceX(),surfWave.sourceY(),self.getX(),self.getY())));
		
		
		//final int nayCount = futures.size()/Math.min(offsetMap.size(), NUM_NEIGHBORS);
		
		final long currTime = self.getTime();
		
//		final AtomDouble minSoFar = new AtomDouble(Double.MAX_VALUE), minBin = new AtomDouble(0);
		final List<float[]> futNeighbors = offsetMap.keySet().stream().sorted((d1,d2)->Double.compare(eucishDistBetween(d1,futKey),eucishDistBetween(d2,futKey))).limit(NUM_NEIGHBORS).collect(Collectors.toList());
		
		neighbors.addAll(offsetMap.keySet().stream().sorted((d1,d2)-> Double.compare(eucishDistBetween(d1,key),eucishDistBetween(d2,key))).limit(NUM_NEIGHBORS).collect(Collectors.toList()));

		/*offsetMap.keySet().stream().sorted((d1,d2)
				-> Double.compare(manDistBetween(d1,key),manDistBetween(d2,key))
				).limit(NUM_NEIGHBORS).flatMapToDouble(n -> {
					final double bor = offsetMap.get(n); return futures.stream().parallel()//.map(f->predictToWith(f, surfWave))
					.mapToDouble(f->{ if(shades.stream().filter(
							s->s.parent == surfWave.hashCode() && s.contains(new Rectangle2D.Double(f.x-5d,f.y-5d,10d,10d))).findAny().isPresent()) return 0d;
							final double offsetFactor = surfWave.getOffsetFactor(f); return Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev)));
				});
			}).forEach(n1f1 -> futDangers[nayFut.incrementAndGet()%futDangers.length] += n1f1);

					;*/

	//	final AtomicInteger nayFut = new AtomicInteger(-1), futCounter = new AtomicInteger(-1), nayCounter = new AtomicInteger(-1), secNayCounter = new AtomicInteger(-1), secFutCounter = new AtomicInteger(-1);

//		final double[] futDangers = new double[futures.size()];
		
		final double secondWaveScalar = 1d / Math.max(1d, 3.5d*Math.log1p(surfWave.timeTillBreak(myCenter, currTime)/*- 0.35d*/));
		
		flatteners.addAll(flattenerMap.keySet().stream().sorted((d1,d2)-> Double.compare(eucishDistBetween(d1,key),eucishDistBetween(d2,key))).limit(NUM_NEIGHBORS).collect(Collectors.toList()));
		secondFlatteners.addAll(flattenerMap.keySet().stream().sorted((d1,d2)-> Double.compare(eucishDistBetween(d1,futKey),eucishDistBetween(d2,futKey))).limit(NUM_NEIGHBORS).collect(Collectors.toList()));
		
		
	//	final double[] futFutureDangers = new double[futures.size()];
		
		
		double minDanger = Double.POSITIVE_INFINITY;
		double minFutureDanger = 0d, powerFrac = 1d;
		Point2D.Double target = new Point2D.Double(0,0);
		
		List<Point2D.Double> mySecondFutures;
		
		for(final Point2D.Double future : futures) {
			final double[] futureStats = predictToWith(future, surfWave);
			final Point2D.Double realFuture = new Point2D.Double(futureStats[0],futureStats[1]);
			double dist = 1d/dist(realFuture.x, realFuture.y, surfWave.sourceX(), surfWave.sourceY());
			double stDev = 2d*Math.atan(18d/dist(realFuture.x,realFuture.y,surfWave.sourceX(),surfWave.sourceY())) / surfWave.mEA;
			double danger = 0d;
			boolean contained = false;
			for(final BullegonLegs hider : shades) {
				if(hider.parent == surfWave.hashCode()
					&& hider.contains(realFuture) && hider.contains(new Rectangle2D.Double(realFuture.x-HALF_SHADE_REQ,realFuture.y-HALF_SHADE_REQ,2d*HALF_SHADE_REQ,2d*HALF_SHADE_REQ))) {
					if(!hider.golden) {
						goldenShades ++;
					}
					hider.golden = true;
					contained = true;
					break;
				}
			}
			final double offsetFactor = surfWave.getOffsetFactor(realFuture);
			
			if(!contained) {
				
				double runner = 1d;
				for(final float[] neigh : neighbors) {
					final double distance = eucishDistBetween(neigh, key);
					final double bor = offsetMap.get(neigh);
					danger += Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev))) * runner /distance;
		//			runner *= 0.97d;
				}
			}
				double runner = 1d;
				double flattenDanger = 0d;
				for(final float[] flatty : flatteners) {
					final double distance = eucishDistBetween(flatty, key);
					final double bor = flattenerMap.get(flatty);
					flattenDanger += Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev))) * runner / distance;
					//runner *= 0.93d;
				}
				//REMEMBER YOU SCALED THIS FOR THE ADDITIONAL WAVE;
				danger += /*(1d + secondWaveScalar)*/flattenerWeight * flattenDanger * dist;
			
			if(danger >= minDanger) {
				continue;
			}
			minFutureDanger = 0d;
			mySecondFutures =  surfFromTill(realFuture, secondWave, futureStats[2], futureStats[3], (long)futureStats[4]);
			for(final Point2D.Double megaFuture : mySecondFutures) {
				contained = false;
				dist = 1d/dist(megaFuture.x, megaFuture.y, surfWave.sourceX(), surfWave.sourceY());
					for(final BullegonLegs hider : shades) {
				if(hider.parent == secondWave.hashCode()
					&& hider.contains(megaFuture) && hider.contains(new Rectangle2D.Double(megaFuture.x-9d,megaFuture.y-9d,18d,18d))) {
					//if(!hider.golden) {
					//	goldenShades ++;
					//}
					//hider.golden = true;
					contained = true;
					break;
				}
			}
					stDev = 2d*Math.atan(18d/dist(megaFuture.x,megaFuture.y,surfWave.sourceX(),surfWave.sourceY())) / surfWave.mEA;
						double dangerTwo = 0d;
						double offsetFactorFut = secondWave.getOffsetFactor(megaFuture);
						
			if(!contained) {
						runner = 1d;
						for(final float[] horses : futNeighbors) {
							final double distance = eucishDistBetween(horses, futKey);
							final double bor = offsetMap.get(horses);
							dangerTwo += Math.exp(-0.5d*((offsetFactorFut-bor)*(offsetFactorFut-bor)/(stDev*stDev)))*runner * dist/ distance;
							//runner *= 0.97d;
						}
			}
						runner = 1d;
						double flattenDangerFut = 0d;
						for(final float[] horses : secondFlatteners) {
							final double distance = eucishDistBetween(horses, futKey);
							final double bor = flattenerMap.get(horses);
							flattenDangerFut += Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev)))*runner * dist /distance;
							//runner *= 0.93d;
						}
						dangerTwo += flattenDangerFut * flattenerWeight;
						//danger += dangerTwo;
						if(dangerTwo < minFutureDanger) {
							
							minFutureDanger = danger;
						}
					//}
					
				
				powerFrac = (secondWave.velocity-20d)/-3d/((surfWave.velocity-20d)/-3d);
			}
			danger += minFutureDanger * powerFrac * secondWaveScalar;
			if(danger < minDanger) {
				secondFutures.clear();
				//if(mySecondFutures!=null) {
					secondFutures.addAll(mySecondFutures);
				//}
				minDanger = danger;
				target.x = future.x;
				target.y = future.y;
			}
			future.setLocation(realFuture);
			mySecondFutures.clear();
		}
		
		
		/*final double[] futureFutureDangers = futures.stream().map(f->predictToWith(f, surfWave)).map(
				f->surfFromTill(new Point2D.Double(f[0],f[1]),secondWave,f[2],f[3],(long)f[4]))
					.mapToDouble(iF ->iF.stream().flatMapToDouble(eF->
						{final double futOffset = secondWave.getOffsetFactor(eF);
							final AtomicInteger nCounter = new AtomicInteger(-1);
							return futNeighbors.stream().mapToDouble(fn->{
								final double runner = Math.pow(0.95d, nCounter.incrementAndGet());
								final double memory = offsetMap.get(fn); return runner * Math.exp(-0.5d*((futOffset-memory)*(futOffset-memory)/(stDev*stDev))) * secondWaveScalar;
						});
					})).toArray();*/
					
		//System.out.println(Arrays.toString(futureFutureDangers));
		
				//	System.out.println(Arrays.toString(futureFutureDangers));
/*


.map(
							f->surfFromTill(new Point2D.Double(f[0],f[1]),secondWave,f[2],f[3],(long)f[4]).stream().mapToDouble(
								superf->{
									final double runner = Math.pow(0.95d, nayCount);
									final double offsetFactor = secondWave.getOffsetFactor(superf); return runner*(Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev))));
								}));
			}).toArray();		*/



		
		//for(final Point2D.Double future : futures) {
		//	final double[] dat = predictToWith(future, surfWave);
		//	future.x = dat[0];
		//	future.y = dat[1];	
		//}

/*		final double weightFinaly = flattenerWeight;
		zip(offsetMap.keySet().stream().sorted((d1,d2)
				-> Double.compare(manDistBetween(d1,key),manDistBetween(d2,key))
				).limit(NUM_NEIGHBORS),
			flattenerMap.keySet().stream().sorted((d1,d2)
				-> Double.compare(manDistBetween(d1,key),manDistBetween(d2,key))
				).limit(NUM_NEIGHBORS), (n1, n2) -> new float[][]{n1,n2}).flatMapToDouble(n-> {
					final int nayCount = nayCounter.incrementAndGet();
					final double bor1 = offsetMap.get(n[0]); final double bor2 = flattenerMap.get(n[1]); return futures.stream()
					.mapToDouble(f->{ if(shades.stream().filter(
							s->s.parent == surfWave.hashCode() && s.contains(new Rectangle2D.Double(f.x-4.5d,f.y-4.5d,9d,9d))).findAny().isPresent())
								return 0d;
							final double runner = Math.pow(0.95d, nayCount);
							final double offsetFactor = surfWave.getOffsetFactor(f); return nayCount * (Math.exp(-0.5d*((offsetFactor-bor1)*(offsetFactor-bor1)/(stDev*stDev)))
									+ weightFinaly * Math.exp(-0.5d*((offsetFactor-bor2)*(offsetFactor-bor2)/(stDev*stDev))));// + futureFutureDangers[fut];
				});
			}).forEach(n1f1 -> futDangers[nayFut.incrementAndGet()%futDangers.length] += n1f1);
					;
		
		
		//final double[] futureFutureDangers =
		backups.stream().map(f->surfToWith(f, surfWave))
						.map(f->surfFromTill(new Point2D.Double(f[0],f[1]),secondWave,f[2],f[3],(long)f[4]))
							.forEach(pointList->{
								final int fut = secFutCounter.incrementAndGet();
								for(final Point2D.Double p : pointList) {
									final AtomicInteger nayTracker = new AtomicInteger(-1);
									double totDanger = futDangers[fut];
									if(totDanger > minSoFar.get()) {
										continue;
									}
									for(final float[] n : futNeighbors) {
										final double bor = offsetMap.get(n);
										final double offsetFactor = secondWave.getOffsetFactor(p);
										final double runner = Math.pow(0.95d, nayTracker.incrementAndGet());
										totDanger += runner * Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev))) *0.5d* secondWaveScalar;
									}
									futDangers[fut] += totDanger;
									//System.out.println(minSoFar.get());
									if(futDangers[fut] < minSoFar.get()) {
										secondFutures.clear();
										secondFutures.addAll(pointList);
										minSoFar.set(totDanger);
										minBin.set(fut);
									}
								}
							});
		*/

	//	final double minDanger = Double.POSITIVE_INFINITY;
		 
		//int masterDex = 0;
		//for(int fud = 0; fud < futDangers.length; fud ++) {
	//		if(futDangers[fud] < futDangers[masterDex]) {
	///			masterDex = fud;
	//		}
	//	}
		
		return target;// futures.get((int)minBin.get());
		
}
/*


		neighbors.addAll(offsetMap.keySet().stream().sorted((d1,d2)
				-> Double.compare(manDistBetween(d1,key),manDistBetween(d2,key))
			).limit(NUM_NEIGHBORS).collect(Collectors.toList()));
		if(Double.compare(flattenerWeight, 0) > 0) {
			flatteners.addAll(flattenerMap.keySet().stream().sorted((d1,d2)
				-> Double.compare(manDistBetween(d1,key),manDistBetween(d2,key))
			).limit(NUM_FLATTENER_NEIGHBORS).collect(Collectors.toList()));
		} //else {
			///flatteners = new ArrayList<>();
		//}
		//secondWave = null;
		surfTwo = surfTwo || (Double.compare(eRangeAcc, 0.071) > 0 && eShots > 28);
		if(secondWave != null && surfTwo) {
			if(updated = !swappedDualSurf) {
				System.arraycopy(dualSurfColor, 0, activeColorBuffer, 0, dualSurfColor.length);
				swappedDualSurf = true;
			}
			final float[] key2 = new float[]
				{
			secondWave.accelSegD, secondWave.distSegD, secondWave.velSegD, secondWave.wallSpaceSegD,
			secondWave.revWallSpaceSegD, secondWave.headSegD, secondWave.bPowerSegD
				};
			secondNeighbors.addAll(
				offsetMap.keySet().stream().sorted((d1,d2)
					-> Double.compare(manDistBetween(d1,key2),manDistBetween(d2,key2))
				).limit(SECOND_WAVE_NEIGHBORS).collect(Collectors.toList())
			);
			if(Double.compare(flattenerWeight,0)>0) {
				secondFlatteners.addAll(flattenerMap.keySet().stream().sorted((d1,d2)
					-> Double.compare(manDistBetween(d1,key2),manDistBetween(d2,key2))
				).limit(NUM_FLATTENER_NEIGHBORS).collect(Collectors.toList()));
			}
		} else if(updated = swappedDualSurf) {
				System.arraycopy(baseColor, 0, activeColorBuffer, 0, baseColor.length);
				swappedDualSurf = false;
			}
		
		double minDanger = Double.POSITIVE_INFINITY;
		double minFutureDanger = 0d, powerFrac = 1d;
		Point2D.Double target = new Point2D.Double(0,0);
		
		List<Point2D.Double> mySecondFutures = null;
		for(final Point2D.Double future : futures) {
			final Point2D.Double realFuture = predictToWith(future, surfWave);
			
			double danger = 0d;
			boolean contained = false;
			for(final BullegonLegs hider : shades) {
				if(hider.parent == surfWave.hashCode()
					&& hider.contains(realFuture) && hider.contains(new Rectangle2D.Double(realFuture.x-5.18d,realFuture.y-5.18d,10.36d,10.36d))) {
					if(!hider.golden) {
						goldenShades ++;
					}
					hider.golden = true;
					contained = true;
					break;
				}
			}
			if(!contained) {
				final double offsetFactor = surfWave.getOffsetFactor(realFuture);// limit(-1d, Utils.normalRelativeAngle(Math.atan2(realFuture.x - surfWave.sourceX(), realFuture.y - surfWave.sourceY()) - surfWave.groundHeading) / surfWave.mEA, 1d) * surfWave.dir;
				
				double runner = 1d;
				for(final float[] neigh : neighbors) {
					final double bor = offsetMap.get(neigh);
					danger += Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev))) * runner;
					runner *= 0.97d;
				}
				runner = 1d;
				double flattenDanger = 0d;
				for(final float[] flatty : flatteners) {
					final double bor = flattenerMap.get(flatty);
					flattenDanger += Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev))) * runner;
					runner *= 0.93d;
				}
				danger += flattenerWeight * flattenDanger;// * (1d - dist(realFuture.x,realFuture.y,surfWave.sourceX(),surfWave.sourceY())/2000d);
			}
			if(danger >= minDanger) {
				continue;
			}
			if(secondWave != null && surfTwo) {
				//mySecondFutures.clear();
				//mySecondFutures.addAll(surfFromTill(realFuture, secondWave, waveBreakHeading, waveBreakVelocity, firstWaveBreak));
				minFutureDanger = 0d;
				mySecondFutures =  surfFromTill(realFuture, secondWave, waveBreakHeading, waveBreakVelocity, firstWaveBreak);
				for(final Point2D.Double megaFuture : mySecondFutures) {
					/*boolean contained = false;
					for(final BullegonLegs hider :shades) {
						if(hider.parent == secondWave.hashCode()
							&& hider.contains(megaFuture)
								&& hider.contains(new Rectangle2D.Double(megaFuture.x-5d,megaFuture.y-5d,10d,10d))) {
							contained = true;
							break;
						}
					}
					if(!contained) {*/
	/*					double dangerTwo = 0d;
						double offsetFactor = secondWave.getOffsetFactor(megaFuture);
						double runner = 1d;
						for(final float[] horses : secondNeighbors) {
							final double bor = offsetMap.get(horses);
							dangerTwo += Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev)))*runner;
							runner *= 0.97d;
						}
						runner = 1d;
						double flattenDanger = 0d;
						for(final float[] horses : secondFlatteners) {
							final double bor = flattenerMap.get(horses);
							flattenDanger += Math.exp(-0.5d*((offsetFactor-bor)*(offsetFactor-bor)/(stDev*stDev)))*runner;
							runner *= 0.93d;
						}
						dangerTwo += flattenDanger * flattenerWeight;
						if(danger < minFutureDanger) {
							
							minFutureDanger = danger;
						}
					//}
					
				}
				powerFrac = (secondWave.velocity-20d)/-3d/((surfWave.velocity-20d)/-3d);
			}
			
			danger += powerFrac*minFutureDanger / Math.max(1d, 2.5d*Math.log1p(firstWaveBreak - self.getTime() - 0.35d));
			if(danger < minDanger) {
				secondFutures.clear();
				if(mySecondFutures!=null) {
					secondFutures.addAll(mySecondFutures);
				}
				minDanger = danger;
				target.x = future.x;
				target.y = future.y;
			}
			future.setLocation(realFuture);
		}
		return target;
	}
	
*/

	private double[] surfToWith(final Point2D.Double target, final InFire wave) {
		//final List<Point2D.Double> myFutures = new ArrayList<>();
		double maxVel = 8d;
		//if(this.activeSecondID == wave.hashCode()) {
		//	maxVel = 6d;
		//}
		
		Point2D.Double nextPos = new Point2D.Double(myCenter.x,myCenter.y);
	
		boolean stopped = Double.compare(self.getVelocity(), 2) <= 0;
		
		//if(stopped || Math.abs(startVelocity) <= 2d)
			//myFutures.add(nextPos);
			
		double predVelocity = self.getVelocity();
		double predHeading = self.getHeadingRadians();
		long predTime = self.getTime();

		for(int absDir = 1; absDir > -2; absDir -=2) {
			nextPos.x = myCenter.x;
			nextPos.y = myCenter.y;
			double absAngleToRot = Math.atan2(wave.sourceX() - nextPos.x, wave.sourceY() - nextPos.y);
			

			double bearingToRot = Utils.normalRelativeAngle(absAngleToRot - predHeading);
		

			int absLatDir = absDir * sign(bearingToRot);//sign(bearingToRot);
		
			double desiredHeading = wallSmooth(nextPos, absLatDir,
					absAngleToRot - Math.PI/2d  * sign(bearingToRot) + (stopped  ? 0d : - absLatDir * Rules.MAX_TURN_RATE_RADIANS * 1.8d * (healthDisadvantage+2.7d))
				,absDir);
			
			predVelocity = predVelocity*absDir < 0 ? predVelocity +2d*absDir : predVelocity + absDir;
			predVelocity = limit(-maxVel, predVelocity, maxVel);
			predTime = self.getTime() + 1;
			nextPos = project(nextPos,predVelocity,desiredHeading);
			
			predHeading = desiredHeading;
		
			
			//myFutures.add(nextPos);
			
			 while(!wave.breaks(nextPos, predTime)) {
				stopped = Double.compare(predVelocity, 0) == 0;

				absAngleToRot = Math.atan2(wave.sourceX() - nextPos.x, wave.sourceY() - nextPos.y);
				bearingToRot = Utils.normalRelativeAngle(absAngleToRot - desiredHeading);
		
				absLatDir = absDir * sign(bearingToRot);//sign(bearingToRot);
				
				desiredHeading = wallSmooth(nextPos, sign(predVelocity*bearingToRot), absAngleToRot - Math.PI/2d  * sign(bearingToRot)  + (stopped ? 0d : - absLatDir * Rules.MAX_TURN_RATE_RADIANS * 1.8d * (healthDisadvantage+2.7d)), sign(predVelocity));
				predVelocity += predVelocity * absDir < 0 ? 2*absDir : absDir;
				predVelocity = limit(-maxVel, predVelocity, maxVel);
				
				nextPos = (project(nextPos,predVelocity,desiredHeading));
				
				predHeading = desiredHeading;
				
//add precise prediction / real spot pair to list

				//myFutures.add(nextPos);
				predTime ++;
			}
		}
		
		return new double[]{nextPos.x,nextPos.y, predHeading, predVelocity,predTime};
		

	}


	private double[] predictToWith(final Point2D.Double target, final InFire surfWave) {
		final long currTime = self.getTime();
		
		final Point2D.Double realFuture = new Point2D.Double(myCenter.x, myCenter.y);
		
		double predHeading = self.getHeadingRadians();
		double predVelocity = self.getVelocity();
		
		
		int counter = 0;
		do {
			double distToTarget = realFuture.distance(target);
			double absAngleToTarget = Utils.normalAbsoluteAngle(Math.atan2(target.x-realFuture.x,target.y-realFuture.y));
			double turnAmount = Utils.normalRelativeAngle(absAngleToTarget - predHeading);
			if(Math.abs(turnAmount) > Math.PI/2d) {
				turnAmount = Utils.normalRelativeAngle(Math.PI-turnAmount)*-1;
				distToTarget *= -1;
			}

			final double hardTurnLimit = Rules.getTurnRateRadians(Math.abs(predVelocity));
			final double turnMax = turnAmount;
			turnAmount = limit(-hardTurnLimit, turnAmount, hardTurnLimit);
			
			predHeading = Utils.normalAbsoluteAngle(predHeading + turnAmount);
			
			predVelocity = limit(-8d,getNewVelocity2(predVelocity, distToTarget),8d);
		
			realFuture.setLocation(project(realFuture,predVelocity,predHeading));
			
			counter ++;
		} while(!surfWave.breaks(realFuture, currTime + counter));
		
		return new double[]{realFuture.x,realFuture.y,predHeading,predVelocity,counter+currTime};	
	}



//////////////////////////////////////////////////////////////////////////////////
//Thanks Voidous

	private double getNewVelocity(double velocity, double distance) {
		// If the distance is negative, then change it to be positive and change the sign of the input velocity and the result
		if (distance < 0) {
			return -getNewVelocity(-velocity, -distance);
		}

		double newVelocity;

		// Get the speed, which is always positive (because it is a scalar)
		final double speed = Math.abs(velocity); 

		// Check if we are decelerating, i.e. if the velocity is negative.
		// Note that if the speed is too high due to a new max. velocity, we must also decelerate.\
		if (velocity < 0) {
			// If the velocity is negative, we are decelerating
			newVelocity = speed - Rules.DECELERATION;

			// Check if we are going from deceleration into acceleration
			if (newVelocity < 0) {
				// If we have decelerated to velocity = 0, then the remaining time must be used for acceleration
				double decelTime = speed / Rules.DECELERATION;
				double accelTime = (1 - decelTime);

				// New velocity (v) = d / t, where time = 1 (i.e. 1 turn). Hence, v = d / 1 => v = d
				// However, the new velocity must be limited by the max. velocity
				//newVelocity = Math.min(maxVel,
				//	Math.min(Rules.DECELERATION * decelTime *decelTime + Rules.ACCELERATION * accelTime * accelTime,
				//		distance));
				newVelocity = Math.min(Rules.ACCELERATION * accelTime, distance);

				// Note: We change the sign here due to the sign check later when returning the result
				velocity *= -1;
			}
		} else {
			// Else, we are not decelerating, but might need to start doing so due to the remaining distance

			// Deceleration time (t) is calculated by: v = a * t => t = v / a
			//final double decelTime = speed / Rules.DECELERATION;

			// Deceleration time (d) is calculated by: d = 1/2 a * t^2 + v0 * t + t
			// Adding the extra 't' (in the end) is special for Robocode, and v0 is the starting velocity = 0
			//final double decelDist = 0.5 * Rules.DECELERATION * decelTime * decelTime + decelTime;

			final double decelDist = decelDistance(speed);
			
			final long decelTime = Math.round( // VOIDIOUS: for rounding errors? maybe unnecessary
                Math.ceil((speed - Rules.DECELERATION) / Rules.DECELERATION));

			// The maximum distance coverable with an equivalent decelTime
            final double decelTimeMaxDist = ((decelTime + 1) / 2.0) * decelTime // sum of 1..decelTime
                * Rules.DECELERATION;

            if (distance <= Rules.DECELERATION) {
                // If we can cover remaining distance and then completely stop,
                // set speed = distance
                newVelocity = Math.max(speed - Rules.DECELERATION, distance);
            } else if (distance <= decelTimeMaxDist) {
                // If we can cover distance in decelTime, split any extra
                // distance (between decelDist and distance) over decelTime
                // ticks
                newVelocity = speed - Rules.DECELERATION +
                    ((distance - decelDist) / decelTime);
            } else {
                // If we need more than decelTime ticks, try to spread the
                // extra distance over (decelTime + 1) ticks. This will just max
                // the acceleration if it needs to (ie, if we need more ticks).
                // VOIDIOUS: I think this part would break if Rules.ACCELERATION
                //           were set above Rules.DECELERATION; we might need an
                //           extra case or something. Doh. =(
                newVelocity = Math.min(speed + Rules.ACCELERATION,
                    (decelTime * Rules.DECELERATION) +
                        ((distance - decelTimeMaxDist) / Math.max(decelTime + 1, 2d)));
            }
        }

        // VOIDIOUS: I think it makes more sense to do this here; no need to decelerate maximally
        //           if you don't need to to accomodate a new setMaxVelocity.
        newVelocity = Math.max(speed - Rules.DECELERATION,
            Math.min(speed + Rules.ACCELERATION,
                Math.min(newVelocity, getMaxVelocity(distance))));

        // Return the new velocity with the correct sign. We have been working with the speed, which is always positive
        return (velocity < 0) ? -newVelocity : newVelocity;
	}
	
	private static final double decelDistance(double speed){
        double distance = 0;
        while(speed > 0){
            speed = Math.max(0,speed - Rules.DECELERATION);
            distance += speed;
        }
        return distance;
    }
	
	double getMaxVelocity(double distance)
	{
    	long decelTime = decelTime(distance);
   		double decelDist = (decelTime / 2.0) * (decelTime-1) // sum of 0..(decelTime-1)
        	* Rules.DECELERATION;
        
    	return ((decelTime - 1) * Rules.DECELERATION) +
        	((distance - decelDist) / decelTime);
	}

	long decelTime(double distance) {
    	long x = 1;
    	do {
        	// (square(x) + x) / 2) = 1, 3, 6, 10, 15...
        	if (distance <= ((square(x) + x) / 2) * Rules.DECELERATION) {
            	return x;
        	}
        	x++;
    	} while (true);
	}

	long square(long i) {
    	return i * i;
	}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
	

	private static double getNewVelocity2(double velocity, double distance) {
        if (distance < 0)
            // If the distance is negative, then change it to be positive
            // and change the sign of the input velocity and the result
            return -getNewVelocity2(-velocity, -distance);

        final double goalVel;
        if(distance == Double.POSITIVE_INFINITY)
            goalVel = 8d;
        else
            goalVel = Math.min(getMaxVelocity2(distance),
                8d);

        if(velocity >= 0)
            return Math.max(velocity - Rules.DECELERATION,
                Math.min(goalVel, velocity + Rules.ACCELERATION));
        //else
        return Math.max(velocity - Rules.ACCELERATION,
            Math.min(goalVel, velocity + maxDecel(-velocity)));
    }

    final static double getMaxVelocity2(double distance) {
        final double decelTime =  Math.max(1,Math.ceil(
            //sum of 0... decelTime, solving for decelTime using quadratic formula
            (Math.sqrt((4*2/Rules.DECELERATION)*distance + 1) - 1)/2));

        final double decelDist = (decelTime / 2.0) * (decelTime-1) // sum of 0..(decelTime-1)
            * Rules.DECELERATION;

        return ((decelTime - 1) * Rules.DECELERATION) +
            ((distance - decelDist) / decelTime);
    }
    
    private static double maxDecel(double speed) {
        double decelTime = speed / Rules.DECELERATION;
        double accelTime = (1 - decelTime);

        return Math.min(1, decelTime) * Rules.DECELERATION +
               Math.max(0, accelTime) * Rules.ACCELERATION;
    }



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

	private void driveTo(final Point2D.Double desiredLoc) {
		final double distToTarget = desiredLoc.distance(myCenter);
		final double absAngleToTarget = Utils.normalAbsoluteAngle(Math.atan2(desiredLoc.x - myCenter.x, desiredLoc.y - myCenter.y));
		double turnAmount = Utils.normalRelativeAngle(absAngleToTarget - self.getHeadingRadians());
		
		double goDir = 1;
		if(Math.abs(turnAmount) > Math.PI/2d) {
			turnAmount = Utils.normalRelativeAngle(Math.PI-turnAmount);
			goDir = -1;
		}
		double	goMult =  distToTarget;//this.beganDualWave ? (distToTarget) : 2d * distToTarget;
		self.setAhead(goMult*goDir/*(1d-Double.compare(Math.abs(self.getTurnRemainingRadians()),0))*/);
		self.setTurnRightRadians(turnAmount*Double.compare(distToTarget,0)*goDir);
	}







	private void flatten(final InFire w) {
			final float[] key = new float[]
		{
			self.getRoundNum(), self.getTime(), w.accelSegD, w.distSegD, w.velSegD, w.wallSpaceSegD, w.revWallSpaceSegD, w.headSegD, w.bPowerSegD
		};
		//final double resultBearingOffset = Utils.normalRelativeAngle(
		//	Math.atan2(self.getX() - w.sourceX(), self.getY() - w.sourceY()) - w.groundHeading
		//);
		final double offsetFactor = w.getOffsetFactor(myCenter);//limit(-1d,resultBearingOffset / w.mEA, 1d) * w.dir;
		flattenerMap.put(key, offsetFactor);
	}
	





	private List<BullegonLegs> findHallowedGround(final List<Bullet> apexes) {
			
		final long currTime = self.getTime();
		final List<BullegonLegs> newHoles = new ArrayList<>(0);

		for(final InFire ceptor : waves) {
			if(!ceptor.isReal) {
				continue;
			}
			final double distTravelled = ceptor.velocity*(currTime - ceptor.fireTime);
			
			final Ellipse2D[] g = new Ellipse2D[]{
	new Ellipse2D.Double((ceptor.sourceX() - distTravelled+0.0d*ceptor.velocity), (ceptor.sourceY() - distTravelled + 0.0d*ceptor.velocity),2*(distTravelled - 0.0d*ceptor.velocity), 2*(distTravelled - 0.0d*ceptor.velocity)),
	new Ellipse2D.Double((ceptor.sourceX() - distTravelled-1.049d*ceptor.velocity), ceptor.sourceY() - distTravelled - 1.049d*ceptor.velocity, 2*(distTravelled + 1.049d*ceptor.velocity), 2*(distTravelled+1.049d*ceptor.velocity))
			};
			
			final double distance = ceptor.sourceDist(new Point2D.Double(self.getX(),self.getY()));	
		outer:
		for(final Bullet steeple : apexes) {
			final int[] xPoints = new int[4];
			final int[] yPoints = new int[4];
			final int[] xy1 = project(steeple.getX(), steeple.getY(), -0.d*steeple.getVelocity(), steeple.getHeadingRadians());
			xPoints[0] = xy1[0]; yPoints[0] = xy1[1];
			final int[] xy2 = project(steeple.getX(), steeple.getY(), 1.d*steeple.getVelocity(), steeple.getHeadingRadians());
			xPoints[1] = xy2[0]; yPoints[1] = xy2[1];
			
			final Rectangle2D bulletTrail = new Line2D.Double(xPoints[0],yPoints[0],xPoints[1],yPoints[1]).getBounds2D();
			
			if((!g[0].intersects(bulletTrail)) && g[1].intersects(bulletTrail)) {
				final int[] xy3 = project(ceptor.sourceX(), ceptor.sourceY(), 1.85d*distance, Math.atan2(xPoints[1] - ceptor.sourceX(), yPoints[1] - ceptor.sourceY()));
				xPoints[2] = xy3[0]; yPoints[2] = xy3[1];
				final int[] xy4 = project(ceptor.sourceX(), ceptor.sourceY(), 1.85d*distance, Math.atan2(xPoints[0] - ceptor.sourceX(), yPoints[0] - ceptor.sourceY()));
				xPoints[3] = xy4[0]; yPoints[3] = xy4[1];
				newHoles.add(new BullegonLegs(ceptor.hashCode(),xPoints,yPoints));
			}
		}
		}
		
		return newHoles;
	}
	




	private double wallSmooth(final Point2D.Double pos, final int latDir, double head, final int velDir) {
		int counter = 0;
		final double inc = Math.PI/64d;
		final double wallStick = ClownCar.surfTains(pos) ? 43d : 200d;
		while(!ClownCar.surfTains(project(pos, wallStick*velDir, head)) && counter < 128) {
			head += inc * latDir;
			counter ++;
		}
		return head;
	}
	




	private void storeWave(final InFire w, final double targetX, final double targetY, final boolean hitUs) {
		if(w == null) {
			System.out.println("Dropped waved, sir");
			return;
		}
		if(hitUs && dist(w.sourceX(),w.sourceY(), targetX, targetY) > 350) {
			eRangeHits ++;
		}
		flatten(w);
		//ACCEL_BIN = 0, DIST_BIN = 1, VEL_BIN = 2, WALL_BIN = 3, REV_WALL_BIN = 4, HEAD_BIN = 5, BULLET_BIN = 6
		final float[] key = new float[]
		{
			self.getRoundNum(), self.getTime(), w.accelSegD, w.distSegD, w.velSegD, w.wallSpaceSegD, w.revWallSpaceSegD, w.headSegD, w.bPowerSegD
		};
		//final double resultBearingOffset = Utils.normalRelativeAngle(
		//	Math.atan2(targetX - w.sourceX(), targetY - w.sourceY()) - w.groundHeading
		//);
		final double offsetFactor = w.getOffsetFactor(new Point2D.Double(targetX, targetY));//limit(-1d,resultBearingOffset / w.mEA, 1d) * w.dir;
		offsetMap.put(key, offsetFactor);
	}



	
	public void storeBullet(final Bullet b, final boolean hitUs) {
		double x = myCenter.x, y = myCenter.y;
		//double x = b.getX(), y = b.getY();
		eHits ++;
		if(!hitUs) {
			eHits --;
			x = b.getX(); y = b.getY();
		}
		final InFire incident;
		storeWave(incident = getWave(b), x, y, hitUs);
		if(incident != null)
			incident.setInactive();
	}
	


	public void expectDamage(final double damage) {
		this.expectedDamage += damage;
	}




	private InFire getWave(final Bullet b) {
		final long currTime = self.getTime();
		InFire incident = null;
		for(final InFire wave : waves) {
			if(wave.isReal && Math.abs(wave.velocity - b.getVelocity()) < .1d && Math.abs(dist(b.getX(),b.getY(),wave.sourceX(),wave.sourceY()) - wave.velocity * (currTime - 1 - wave.fireTime)) <= 1.5d*b.getVelocity()) {
				incident = wave;
				break;
			}
		}

		return incident;
	}






	private void cull(final List<BullegonLegs> shades) {
		final Iterator<BullegonLegs> bullerator = shades.iterator();
		while(bullerator.hasNext()) {
			final BullegonLegs shade= bullerator.next();
			if(!(waves.stream().filter(w -> w.hashCode() == shade.parent).findAny().isPresent())) {
				bullerator.remove();
			}
		}
	}
	
	double manDistBetween(final float[] entry, final float[] key) {
		return 3d*Math.abs(entry[ACCEL_BIN+2]-key[ACCEL_BIN]) + Math.abs(entry[DIST_BIN+2]-key[DIST_BIN])
			+ Math.abs(entry[VEL_BIN+2]-key[VEL_BIN]) + Math.abs(entry[WALL_BIN+2]-key[WALL_BIN])
				+ Math.abs(entry[REV_WALL_BIN+2]-key[REV_WALL_BIN]) + Math.abs(entry[HEAD_BIN+2]-key[HEAD_BIN])
		+ Math.abs(entry[BULLET_BIN+2]-key[BULLET_BIN]);
	}
	
	double eucishDistBetween(final float[] entry, final float[] key) {
		double sum = 0d;
		for(int i = 0; i < key.length; i ++) {
			sum += (key[i]-entry[i+2])*(key[i]-entry[i+2]);
		}
		return sum;
	}
	
	private int[] project(final double sourceX, final double sourceY, final double distance, final double angle) {
		return new int[]{
			(int)(sourceX + distance * Math.sin(angle)),
			(int)(sourceY + distance * Math.cos(angle))
		};
	}

	private Point2D.Double project(final Point2D.Double source, final double distance, final double angle) {
		return new Point2D.Double(source.x + distance*Math.sin(angle), source.y + distance*Math.cos(angle));
	}
	
	double dist(final double x1, final double y1, final double x2, final double y2) {
		return Math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	}
	
	int sign(final double value) {
		return value < 0 ? -1 : 1;
	}
	
	double limit(final double low, final double mid, final double high) {
		return Math.max(low, Math.min(mid,high));
	}
	
	public void onPaint(final Graphics2D g) {
		final long currTime = self.getTime();
		for(final BullegonLegs b : shades) {
			
			if(b.parent == activeID) {
		g.setColor(!b.golden ? Color.MAGENTA : Color.YELLOW.darker().darker());
			} else {
				g.setColor(Color.GREEN.darker());
			}
				g.draw(b);
		}
		for(final InFire w : waves) {
			if(!w.isReal) continue;
			final double code = w.hashCode();
			g.setColor(w.checkHit(myCenter, currTime) ? Color.ORANGE.darker() :
					w.dir == 1 ? code == this.activeSecondID ? Color.RED :
							code == this.activeID ? Color.MAGENTA :
								Color.WHITE :
							code == this.activeID ? Color.GREEN :
							this.activeSecondID == code ? Color.BLUE.brighter() :
								Color.YELLOW);
			final double distTravelled = w.velocity*(currTime - w.fireTime);
			g.draw(new Ellipse2D.Double(w.sourceX() - distTravelled, w.sourceY() - distTravelled, 2*distTravelled, 2*distTravelled)); 
		}
		g.setColor(Color.MAGENTA.darker());
		for(final Point2D.Double spot : futures) {
			g.fill(new Ellipse2D.Double(spot.x-1,spot.y-1, 2, 2));
		}
		g.setColor(Color.GREEN.brighter());
		for(final Point2D.Double spot : secondFutures) {
			g.fill(new Ellipse2D.Double(spot.x-1,spot.y-1,2,2));
		}
	}
	
	public void endRound() {
		this.isRoundEnded = true;
	}

	public void reset() {
		this.waves.clear();
		this.shades.clear();
		this.futures.clear();
		this.backups.clear();
		this.secondFutures.clear();
		neighbors.clear();
		flatteners.clear();
		secondNeighbors.clear();
		secondFlatteners.clear();
		this.activeID = 0;
		this.activeSecondID = 0;
		
		if(offsetMap.size() > MAX_MAP_SIZE) {
			final List<float[]> oldKeys = offsetMap.keySet().stream().sorted(
				(d1, d2) -> d1[0] == d2[0] ? Double.compare(d1[1],d2[1]) : Double.compare(d1[0],d2[0])
			).limit(offsetMap.size()- SHRINK_TARGET).collect(Collectors.toList());
			System.out.println("Clearing " + (oldKeys.size()) + " surf datums");
			for(final float[] oldKey : oldKeys) {
				offsetMap.remove(oldKey);
			}
		}
		
		if(flattenerMap.size() > MAX_FLATTEN_SIZE) {
			final List<float[]> oldKeys =
			flattenerMap.keySet().stream().sorted(
				(d1, d2) -> /*d1[0] == d2[0] ? Double.compare(d1[1],d2[1]) :*/ Double.compare(d1[0],d2[0])
			).limit(flattenerMap.size() - FLATTENER_SHRINK_TARGET).collect(Collectors.toList());
			System.out.println("Clearing " + oldKeys.size() + " flattening logs");
			for(final float[] oldKey : oldKeys) {
				flattenerMap.remove(oldKey);
			}
		}
		System.out.println("Real enemy acc (idgafwyt): %" + (Math.round(eAcc*10000d)/100d));
		System.out.println("Real enemy range acc     : %" + (Math.round(eRangeAcc*10000d)/100d));
		System.out.println("Arbitrary flattener weight: %" + (Math.round(flattenerWeight/(Math.min((double)NUM_NEIGHBORS, (double)offsetMap.size()+1d)/(double)NUM_FLATTENER_NEIGHBORS)*10000d)/100d));
		System.out.println("Secret Sauce: " + (beganDualWave ? "active\nGolden Shades Found " + goldenShades : "inactive"));
		System.out.println("Enemy tried to sneak : " + sneakBullets + " bullets past");
		
		this.isRoundEnded = false;
	}

}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


class InFire {
	private final Point2D.Double source;
	final double groundHeading, mEA, velocity;
	final long fireTime;
	
	final float distSegD, velSegD, accelSegD, wallSpaceSegD, revWallSpaceSegD, headSegD, bPowerSegD;
	
	final int dir;
	
	final boolean isReal, isRange;
	
	private boolean isActive;
	
	InFire(final double sourceX, final double sourceY, final double groundHeading,
			final double velocity, long fireTime,
				final int dir, final float distSegD,
					final float velSegD, final float accelSegD, final float wallSpaceSegD, final float revWallSpaceSegD, final float headSegD,
			final boolean isReal, final float bPowerSeg, final boolean isRange) {
		this.source = new Point2D.Double(sourceX, sourceY);
		this.mEA = Math.asin(8d/velocity);
		this.velocity = velocity;
		this.groundHeading = groundHeading;
		this.fireTime = fireTime;
		this.dir = dir;
		this.isRange = isRange;
		
		this.distSegD = distSegD;
		this.velSegD = velSegD;
		this.accelSegD = accelSegD;
		this.wallSpaceSegD = wallSpaceSegD;
		this.revWallSpaceSegD = revWallSpaceSegD;
		this.headSegD = headSegD;
		this.isActive = true;
		this.isReal = isReal;
		this.bPowerSegD = bPowerSeg;
	}
	
	boolean checkHit(final Point2D.Double self, long currentTime) {
		//return source.distance(enemy) <= (currentTime - fireTime) * velocity;
		final double distTravelled = (currentTime - fireTime + 1) * velocity;
		final Rectangle2D.Double bot = new Rectangle2D.Double(self.x-18d,self.y-18d,36,36);
		return new Ellipse2D.Double(source.x-distTravelled, source.y -distTravelled,2*distTravelled,2*distTravelled)
				.intersects(bot) && !new Ellipse2D.Double(source.x-distTravelled+velocity, source.y -distTravelled+velocity,2*(distTravelled-velocity),2*(distTravelled-velocity))
				.contains(bot);
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
		if(o instanceof InFire) {
			final InFire w = (InFire)o;
			return fireTime == w.fireTime;
		} return false;
	}
}


	
class BullegonLegs extends Polygon {
	final int parent;
	boolean golden;
	BullegonLegs(final int parent, final int[] xPoints, final int[] yPoints) {
		super(xPoints, yPoints, xPoints.length);
		this.parent = parent;
	}
}

class AtomDouble {
	private double value;
	public AtomDouble(final double value) {
		this.value = value;
	}
	double get() {
		return value;
	}
	void set(final double value) {
		this.value = value;
	}
}

