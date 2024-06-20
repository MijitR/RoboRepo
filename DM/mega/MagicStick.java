package DM.mega;

import robocode.*;
import robocode.util.Utils;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.Color;
import java.util.List;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.HashMap;
import java.util.stream.Stream;
import java.util.stream.Collectors;
import java.util.Arrays;
/**
 * MagicStick - a class by Damij
 */
public class MagicStick
{
	//KEEP BINS ODD BUT HEAD_SEGS EVEN <<--? why?
	static final int BINS = 111, ACCEL_SEGS = 8, DIST_SEGS = 8, VEL_SEGS = 8, WALL_DIST_SEGS = 8, HEAD_SEGS = 8, B_POWER_SEGS = 8,
		ACCEL_BIN=0,DIST_BIN=1,VEL_BIN=2,WALL_BIN=3,REV_WALL_BIN=4,HEAD_BIN=5,BULLET_BIN=6;

	private final HashMap<float[], Double> neighborMap;
	
	private final List<StickyGoo> launches; 
	private final List<Future> _futures;
	
	final double[] dangerZones;
	
	final AdvancedRobot self;
	final Point2D.Double enemyPos, myCenter;
	
	final double distSegSize = 1000d/(double)DIST_SEGS, wallDistSegSize = 1000d/(double)WALL_DIST_SEGS,
			WALL_STICK = (8d+6d+4d+2d), B_POWER_SEG_SIZE = 3d/(double)B_POWER_SEGS, ACCEL_SEG_SIZE = 3d/(double)ACCEL_SEGS;
	
	final int MAX_MAP_SIZE = 635, SHRINK_TARGET = 634, NUM_NEIGHBORS = 7;//7;
	
	private float[] currKey;

	double bPower, velocity, oldVelocity, healthDisadvantage, knnAcc = 0.9d, shotPower,
			knnConfidence, eDistance, eBearing, absBearing, maxOffset, minOffset, maxCrimp, minCrimp;
	long timeToShoot, passCount;
	int eDir, eLatDir, shots, hits;
	

	float currDistSeg, currWallSeg, revWallSeg, currVelSeg, accelSeg, headSeg, bPowerSeg,
		lastWDSeg, lastRevWDSeg, lastDSeg, lastVSeg,lastASeg,lastHSeg,lastBPSeg;
		
	
	boolean isMC;
	
	public MagicStick(final AdvancedRobot self, final boolean isMC) {
		this.self = self;
		this.isMC = isMC;
		
		this.launches = new ArrayList<>();
		this._futures = new ArrayList<>();
		this.neighborMap = new HashMap<>();
		this.enemyPos = new Point2D.Double(0,0);
		this.myCenter = new Point2D.Double(0,0);
		
		this.dangerZones = new double[BINS];
		
		this.eDir = 1;
		this.eLatDir = 1;
		
		this.bPower = 1.7d;
	}
	
	public Bullet update(final ScannedRobotEvent e, final double absBearing) {
		this.healthDisadvantage = Math.min(3,Math.max(-3,(e.getEnergy() - self.getEnergy())/33d));
		
		Bullet shot = null; 

		final double targetAngleWidth = 2d*Math.atan(18d/e.getDistance());
		
		this.absBearing = absBearing;
		
		myCenter.x = self.getX();
		myCenter.y = self.getY();
		
		enemyPos.x = myCenter.x + Math.sin(absBearing) * e.getDistance();
		enemyPos.y = myCenter.y + Math.cos(absBearing) * e.getDistance();
		
		eDir = (velocity = e.getVelocity()) == 0 ? eDir : e.getVelocity() > 0 ? 1 : -1;

		eLatDir = Math.sin(e.getHeadingRadians()-absBearing)*e.getVelocity() == 0 ? eLatDir :
			Math.sin(e.getHeadingRadians()-absBearing)*e.getVelocity() > 0 ? 1 : -1;
			
		eDistance = e.getDistance();
		eBearing = e.getBearingRadians();
		
		if(timeToShoot == self.getTime() && self.getGunTurnRemainingRadians() == 0d
				&& ((bPower < self.getEnergy() && bPower > 0) || isMC) && self.getGunHeat() == 0
			)
			//	&& (Math.abs(self.getVelocity())-Math.abs(oldVelocity)) <= 0)
				
				//&& oldVelocity * velocity < 0)
			

			 {
			shot = self.setFireBullet(bPower);
			shotPower = bPower;
			
			if(shot!= null) {
				launches.add(
					new StickyGoo(self.getX(), self.getY(), absBearing,
						Rules.getBulletSpeed(shotPower), self.getTime(), eLatDir, minOffset, maxOffset, currKey, true, minCrimp, maxCrimp, self.getRoundNum()
					)
				);
			}
		}
		
		bPower = 1.9d -0.16d*(healthDisadvantage+2);//+ 2.15d * (confidence-0.05);
		 if(self.getEnergy() < 3) {
			bPower = 0.1d;
		} else  if(self.getEnergy() < 7) {
			bPower = 0.2d;
		} else  if(self.getEnergy() < 12) {
			bPower = 0.4d;
		} else if(self.getEnergy() < 16) {
			bPower -= 1d;
		}
		
		bPower = limit(0d, bPower, 3d);

		if(e.getDistance() < 255) {
			bPower = (1d - (e.getDistance()-36d)/(255d-36d)) * (3d-bPower) + bPower;
		}
		
		if(e.getEnergy() < (16d + Rules.getBulletDamage(bPower))/2d) {
			bPower = (e.getEnergy() + 2d) / 6d;
			if(e.getEnergy() <= 4) {
				bPower = Math.max(0.1d, e.getEnergy() / 4d);
			}
		}
		
		if(isMC)
			bPower = Math.min(3d, self.getEnergy());
		
		final Iterator<StickyGoo> wavtor = launches.iterator();
		while(wavtor.hasNext()) {
			final StickyGoo storage = wavtor.next();
			if(storage.checkHit(enemyPos, self.getTime()) || storage.breaks(enemyPos, self.getTime())) {
				wavtor.remove();
				storeWave(storage, enemyPos);
				passCount ++;
			}
		}
		
		this.decideSegmentation(e, absBearing, targetAngleWidth);
		
		currKey = new float[]{currDistSeg,currVelSeg,accelSeg,currWallSeg,revWallSeg,headSeg,bPowerSeg};		

		self.setTurnGunLeftRadians(
			Utils.normalRelativeAngle(
				self.getGunHeadingRadians() - absBearing - aim(currKey, targetAngleWidth)// + (Math.random()-0.5d) * targetAngleWidth / 4d
			)
		);
		
		timeToShoot = self.getTime() + 1;

		return shot;
	}
	
	private void storeWave(final StickyGoo wave, final Point2D.Double target) {
		final double resultAngle = Math.atan2(enemyPos.x - wave.sourceX(), enemyPos.y - wave.sourceY());
		final double bearingOffset = Utils.normalRelativeAngle(resultAngle - wave.groundHeading);
		double offset = bearingOffset / wave.mEA * wave.dir;
		//offset = limit(Math.min(wave.minFactor,wave.maxFactor), offset, Math.max(wave.minFactor,wave.maxFactor));
		boolean crimpedBack = Double.compare(Math.min(wave.minCrimp,wave.maxCrimp), wave.minFactor) != 0;
		
			double minF = Math.min(wave.minFactor,wave.maxFactor), maxF = Math.max(wave.minFactor,wave.maxFactor);
			double minmin = Math.min(wave.minCrimp, wave.maxCrimp), maxmax = Math.max(wave.minCrimp, wave.maxCrimp);
		//System.out.println("minF " + minF + " maxF : " + maxF);
		//System.out.println("minmin " + minmin + " maxmax " + maxmax);
		//offset /= offset < Math.min(wave.minCrimp,wave.maxCrimp) ?  Math.min(wave.minFactor,wave.maxFactor) : offset > Math.max(wave.minCrimp,wave.maxCrimp) ? Math.max(wave.minFactor,wave.maxFactor) : 1d;
		//offset *= offset < mimin ? ;// < minmin ? (offset-minmin)*(minF+minmin) + minmin : offset > maxmax ? (offset-maxmax)*(maxF+maxmax) + maxmax : Math.max(minF, offset);
		//offset = crimpedBack || offset >= 0d ? offset : Math.max(Math.min(wave.minFactor,wave.maxFactor), offset);
		//offset = offset < minmin ? minF*(offset-minmin) + minmin : offset > maxmax ? maxF*(offset-maxmax) + maxmax : offset;
		
		offset = offset > maxmax ? (offset-maxmax)/maxF + maxmax : offset < minmin ? (offset-minmin)/minF + minmin : offset;
		
	//	System.out.println("Caught them at : " + offset);
		final double guessFactor = Math.max(-1d, Math.min(1d, offset));
	//	int index = (int) Math.round((BINS-1)/2*(guessFactor+1d));
	//	final double botAngleWidth = 2d*Math.atan(18d/enemyPos.distance(new Point2D.Double(wave.sourceX(),wave.sourceY())));
	//	final double binAngleWidth = (2d*wave.mEA / (double)((BINS)));
	//	final double botBinWidthD =((botAngleWidth / binAngleWidth));
	//	final double stDev = botBinWidthD / 2d;
	//	final int botBinWidth = (int)Math.ceil(botBinWidthD);
		
		shots ++;
	//	System.out.println(neighborMap.containsKey(wave.key));
			neighborMap.put(wave.key, guessFactor);
	//	System.out.println("compared to current: " + myEucDistBetween(wave.key,currKey));
	}
	
	private void decideSegmentation(final ScannedRobotEvent e,
			final double absBearing, final double targetAngleWidth)
	{
		double eGoing = eDir < 0 ? Math.PI + e.getHeadingRadians() : e.getHeadingRadians();
		
		final Point2D.Double[] nexts
				= getMaxPosAndCrimps();
				
		if(eDir < 0) {
			maxOffset = calcOffset(nexts[0], absBearing);
			minOffset = calcOffset(nexts[1], absBearing);
			maxCrimp = calcOffset(nexts[2], absBearing);
			minCrimp = calcOffset(nexts[3], absBearing);
		} else {
			maxOffset = calcOffset(nexts[1], absBearing);
			minOffset = calcOffset(nexts[0], absBearing);
			maxCrimp = calcOffset(nexts[3], absBearing);
			minCrimp = calcOffset(nexts[2], absBearing);
		}
		
		
		currDistSeg = (float)Math.min(DIST_SEGS-1,(e.getDistance()/distSegSize));
		currVelSeg = (float)(Math.abs(e.getVelocity())/(8.1d/VEL_SEGS));
		accelSeg = (float)Math.min((Double.compare(Math.abs(e.getVelocity()), Math.abs(oldVelocity))+1d)/ACCEL_SEG_SIZE, ACCEL_SEGS-1);
		headSeg = (float)Math.min(HEAD_SEGS - 1, (Math.abs(Math.sin(e.getHeadingRadians()-absBearing)) * HEAD_SEGS));
		

		final double wallDistLat = Utils.normalAbsoluteAngle(eGoing) < Math.PI ?
			(self.getBattleFieldWidth()-enemyPos.x) / (Math.cos((Math.PI/2d - eGoing)))
			: enemyPos.x / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirt = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(self.getBattleFieldHeight()-enemyPos.y) / (Math.cos(eGoing))
				: enemyPos.y / (Math.cos(Math.PI - eGoing));
		
		currWallSeg = Math.max(0,Math.min(WALL_DIST_SEGS-1,(float)(Math.min(wallDistLat, wallDistVirt) / wallDistSegSize)));
		
		eGoing += Math.PI;
		eGoing = Utils.normalRelativeAngle(eGoing);
		
		final double wallDistLatRev = Utils.normalAbsoluteAngle(eGoing) < Math.PI ?
			(self.getBattleFieldWidth()-enemyPos.x) / (Math.cos((Math.PI/2d - eGoing)))
			: enemyPos.x / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirtRev = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(self.getBattleFieldHeight()-enemyPos.y) / (Math.cos(eGoing))
				: enemyPos.y / (Math.cos(Math.PI-Utils.normalAbsoluteAngle(eGoing)));
		
		revWallSeg = Math.max(0, Math.min(WALL_DIST_SEGS-1,(float)(Math.min(wallDistLatRev, wallDistVirtRev)/ wallDistSegSize)));
		

		bPowerSeg = (float)(bPower/B_POWER_SEG_SIZE);
			
		oldVelocity = e.getVelocity();
	}
	
	private double aim(final float[] key, final double targetWidth) {
	
		final double stDev = targetWidth;
		final double mEA =  Math.asin(8d/Rules.getBulletSpeed(bPower));
		
		int targetBinWidth = (int)Math.ceil(targetWidth / mEA * ((BINS-1)/2));
		if((targetBinWidth & 1) == 0) {
			targetBinWidth ++;
		}
		final int finalizedBinWidth = targetBinWidth;
			double minF = Math.min(minOffset,maxOffset), maxF = Math.max(minOffset,maxOffset);
			double minmin = Math.min(minCrimp, maxCrimp), maxmax = Math.max(minCrimp, maxCrimp);
			int minFBin = (int)(minF*((BINS-1)/2) + (BINS-1)/2), maxFBin = (int)(maxF*((BINS-1)/2) + (BINS-1)/2);
			//System.out.println("minF " + minF + " maxF : " + maxF);
			//System.out.println("minmin " + minmin + " maxmax " + maxmax);

		Arrays.fill(dangerZones, 0d);
		
		final Stream<float[]> neighbors = getNeighbors(key);
		//neighborMap.keySet().stream().sorted(
		//	(n1,n2)->Double.compare(manDistBetween(n1,key),manDistBetween(n2,key))).limit(NUM_NEIGHBORS)
		//.forEach(n ->{
		neighbors.forEach(n-> {
		//	if(!neighborMap.containsKey(n)) break;
			double bor = neighborMap.get(n);
			double distan = Math.max(0.0000001d,myManDistBetween(n,key));
			//bor = bor < minmin ? Math.max(0.001d,(bor-minmin)/((minF-minmin)))*minF + minmin : bor > maxmax ? Math.max(0.001d,(bor-maxmax)/((maxF-maxmax)))*maxF + maxmax : bor;
			//bor = limit(-maxF,bor,maxF);
		//	bor /= bor < minmin ? minF*(bor-minmin)/(minF-minmin) + minmin : bor > maxmax ? maxF*(bor-maxmax)/(maxF-maxmax) + maxmax : 1d;
		
			bor = bor < minmin ? minF*(bor-minmin) + minmin : bor > maxmax ? maxF*(bor-maxmax) + maxmax : bor;
			
			bor = limit(minF, bor, maxF);
			int borBin =  (int)((bor *(BINS-1d)/2d) + ((BINS-1d)/2d));
			
			for(int i = Math.max(minFBin,borBin - 2*(finalizedBinWidth-1)/2); i <= Math.min(maxFBin,borBin + 2*(finalizedBinWidth-1)/2); i ++) {
			//	dangerZones[i] ++;
					final double factor = (i - (BINS-1d)/2d)/(double)((BINS-1d)/2d);
					//dangerZones[i] += //Math.abs(borBin - i) <= (finalizedBinWidth-1)/2 ? 1d : 0d;
					dangerZones[i] += Math.exp(-0.5d*((factor-bor)*(factor-bor)/(stDev*stDev))) / distan;
			}
			//final int 
			//bor = bor < minmin ? (bor)*minF + minmin : bor > maxmax ? (bor)*maxF + maxmax : Math.max(minF, bor);
			//System.out.println("keyfirst: \n" + Arrays.toString(key));
			//System.out.println(Arrays.toString(n));
			//System.out.println(myEucDistBetween(n,key));
		//	double runner = 1d;
		//	for(int i = 0, b = i; i < BINS-1; i ++, b = i) {
				//for(int j = i-(finalizedBinWidth-1)/2, b = limit(0,j,i);
				//		j < i + (finalizedBinWidth-1)/2;
				//			j ++, b = limit(0,j,BINS-1)) {
					
		//			final double factor = (b - (BINS-1d)/2d)/(double)((BINS-1d)/2d);
					//dangerZones[i] += //Math.abs(borBin - i) <= (finalizedBinWidth-1)/2 ? 1d : 0d;
		//			dangerZones[i] += Math.exp(-0.5d*((factor-bor)*(factor-bor)/(stDev*stDev))) / distan;
					//runner *= 0.9d;
				//}
		//	}
		//});
		} );

		int maxBin = (BINS-1)/2;//9*(BINS-1)/10;
		for(int i = (BINS-1)/2; i < BINS + (BINS-1)/2; i ++) {
			if(dangerZones[i%BINS] > dangerZones[maxBin]) {
				maxBin = i%BINS;
			}
		}
		
		final double factor = (maxBin - (BINS-1)/2d)/((BINS-1d)/2d);

		return factor * mEA * eLatDir;
	}
	
/*	private float[][] getNeighbors(final float[] key) {
		final float[][] myNeighbors = new float[NUM_NEIGHBORS][9];
		final Iterator<float[]> flitt = neighborMap.keySet().iterator();
		
		int neighborsAdded = 0;
		for(int i = 0; i < Math.min(NUM_NEIGHBORS, neighborMap.size()); i ++) {
			//if(flitt.hasNext()) {
				myNeighbors[i] = flitt.next();
				neighborsAdded ++;
			//}
		}
		
		final double[] nearestDistances = new double[NUM_NEIGHBORS];
		nearestDistances[0] = myEucDistBetween(myNeighbors[0], key);
		double longestDist = nearestDistances[0];
		int longestIndex = 0;
		
		for(int x = 1; x < Math.min(NUM_NEIGHBORS, neighborMap.size()); x ++) {
			nearestDistances[x] = myEucDistBetween(myNeighbors[x], key);
			if(nearestDistances[x] > longestDist){
				longestDist = nearestDistances[x];
				longestIndex = x;
			}
		}
		
		while(flitt.hasNext()) {
			final float[] storagePoint = flitt.next();
			final double dist = myEucDistBetween(storagePoint, key);
			if(dist <= longestDist) {
				myNeighbors[longestIndex] = storagePoint;
				nearestDistances[longestIndex] = dist;
				longestDist = Double.NEGATIVE_INFINITY;
				for(int y = 0; y < nearestDistances.length; y ++) {
					if(nearestDistances[y] > longestDist) {
						longestDist = nearestDistances[y];
						longestIndex = y;
					}
				}
			}
		}
		
			System.out.println(Arrays.toString(nearestDistances));
			System.out.println("Longest at " + longestIndex + " is " + longestDist);
		
		return myNeighbors;
	}
*/

	private Stream<float[]> getNeighbors(final float[] key) {
	
		return neighborMap.keySet().stream().sorted((n1,n2)->Double.compare(myEucDistBetween(n1,key),myEucDistBetween(n2,key))).limit(NUM_NEIGHBORS);
		
	}
	
	private Point2D.Double[] getMaxPos() {
		final Point2D.Double nextBack = new Point2D.Double(enemyPos.x,enemyPos.y),
			nextFront = new Point2D.Double(enemyPos.x,enemyPos.y);
		final double waveVelocity = Rules.getBulletSpeed(bPower);
		final double escAngleBonus  = 0d;
		_futures.clear();
		if(Math.abs(velocity) <= 2d) {
			_futures.add(new Future(enemyPos.x,enemyPos.y, 0));
		}
		for(double dir = -1d; dir <= 1d; dir += 2d) {
			Point2D.Double future =
				(Point2D.Double) enemyPos.clone();
			double futureVelocity = velocity;
			double runningDir = eDir;
			double runningLatDir = eLatDir;
			double bareing = Math.atan2(enemyPos.x - myCenter.x, enemyPos.y - myCenter.y);
			double futureHeading = bareing - (Math.PI / 2d) * sign(eBearing);
			futureHeading += dir*eDir< 0d ? Math.PI : 0d;
			for(int futures = 1;
					!new Ellipse2D.Double(myCenter.x-waveVelocity*futures,myCenter.y-waveVelocity*futures,2*waveVelocity*futures,2*waveVelocity*futures)
						.intersects(new Rectangle2D.Double(future.x-18d,future.y-18d,36d,36d)); futures ++) {
				futureVelocity += dir*runningDir > 0 ? dir : 2d*dir;
				futureVelocity = limit(-8d, futureVelocity, 8d);
				runningDir = futureVelocity == 0d ? runningDir : sign(futureVelocity);
				runningLatDir = runningDir*sign(eBearing);
				futureHeading = Math.atan2(future.x-myCenter.x,future.y-myCenter.y) + Math.PI/2d*runningLatDir;
				futureHeading = wallSmooth(future, futureHeading, sign(Utils.normalRelativeAngle(
				futureHeading - absBearing
			)));
				
				future = project(future, futureHeading, Math.abs(futureVelocity));
				_futures.add(new Future(future.x,future.y, dir));
			}
			if(dir == -1d) {
				nextBack.x = future.x;
				nextBack.y = future.y;
			} else {
				nextFront.x = future.x;
				nextFront.y = future.y;
			}
		}
		
		return new Point2D.Double[] {nextBack, nextFront};
	}
	
	private Point2D.Double[] getMaxPosAndCrimps() {
		final Point2D.Double nextBack = new Point2D.Double(enemyPos.x,enemyPos.y),
			nextFront = new Point2D.Double(enemyPos.x,enemyPos.y);
		final double waveVelocity = Rules.getBulletSpeed(bPower);
		final double escAngleBonus  = 0d;
		
		final Point2D.Double frontCrimp = new Point2D.Double(enemyPos.x,enemyPos.y),
			backCrimp = new Point2D.Double(enemyPos.x,enemyPos.y);
		_futures.clear();
		if(Math.abs(velocity) <= 2d) {
			_futures.add(new Future(enemyPos.x,enemyPos.y, 0));
		}
		for(double dir = -1d; dir <= 1d; dir += 2d) {
			Point2D.Double future =
				(Point2D.Double) enemyPos.clone();
			double futureVelocity = velocity;
			double bareing = Math.atan2(enemyPos.x - myCenter.x, enemyPos.y - myCenter.y);
			double runningDir = eDir;
			double runningLatDir = eLatDir;
			double futureHeading = bareing + (Math.PI / 2d) * runningLatDir;
			futureHeading += runningDir < 0 ? Math.PI : 0d;
			boolean hitWall = false;
			for(int futures = 1;
					futures < 1000d / 11d && !new Ellipse2D.Double(myCenter.x-waveVelocity*futures,myCenter.y-waveVelocity*futures,2*waveVelocity*futures,2*waveVelocity*futures)
						.intersects(new Rectangle2D.Double(future.x-18d,future.y-18d,36d,36d)); futures ++) {
				//futureVelocity += dir * eLatDir > 0 ? sign(futureVelocity)*dir*eLatDir : 2d*sign(futureVelocity)*dir*eLatDir;
				futureVelocity += dir*futureVelocity > 0 ? dir : 2*dir;
				futureVelocity = limit(-8d, futureVelocity, 8d);
				runningDir = futureVelocity == 0d ? runningDir : sign(futureVelocity);
				runningLatDir = runningDir*sign(Utils.normalRelativeAngle(futureHeading - bareing));
				futureHeading = Utils.normalRelativeAngle(bareing + Math.PI/2d*runningLatDir);
				futureHeading += runningDir < 0 ? Math.PI : 0d;
				final double smoothedHeading = wallSmooth(future, futureHeading, dir*sign(Utils.normalRelativeAngle(futureHeading - bareing)), (int)dir);
				if(!hitWall && Math.abs(futureHeading - smoothedHeading) <= Math.PI/45d) {
					if(dir < 0) { backCrimp.x = future.x; backCrimp.y = future.y; }
					else { frontCrimp.x = future.x; frontCrimp.y = future.y; }
				} else {
				//	System.out.println(hitWall + " " + dir + " : " + futureHeading + " " + smoothedHeading);
					hitWall = true;
				}
			//	System.out.println(futureVelocity + " dir: " + dir + " eLatDir: " + eLatDir);
				future = project(future, smoothedHeading, futureVelocity);
				bareing = Math.atan2(future.x - myCenter.x, future.y - myCenter.y);
				_futures.add(new Future(future.x,future.y, dir));
			}
			if(dir == -1d) {
				nextBack.x = future.x;
				nextBack.y = future.y;
				//if(hitWall)
				//	System.out.println("Crimp at back: " + backCrimp);
			} else {
				nextFront.x = future.x;
				nextFront.y = future.y;
				//if(hitWall)
				//	System.out.println("Crimp at front: " + frontCrimp);
			}
		}
		
		return new Point2D.Double[] {nextBack, nextFront, backCrimp, frontCrimp};
	}
	
	private double wallSmooth(final Point2D.Double posi,
		double heading, final double dir)
	{
		while(
			!ClownCar.surfTains(
				project(posi,heading,WALL_STICK)
			)
		) {
			heading += 0.01d * dir;
		}
		return heading;
	}
	
	private double wallSmooth(final Point2D.Double posi,
		double heading, final double latDir, final int velDir)
	{
		int tries = 0;
		while(
			!ClownCar.surfTains(
				project(posi,heading,WALL_STICK*velDir)
			) && tries < 314
		) {
			heading += 0.01d * latDir; tries ++;
		}
		return heading;
	}
	
	private Point2D.Double project(final Point2D.Double pos,
		final double heading, final double dist)
	{
		return  new Point2D.Double(
			pos.x + Math.sin(heading) * dist,
			pos.y + Math.cos(heading) * dist
		);
	}
	
	private double calcOffset(final Point2D.Double pos, final double absBearing) {
		final double mEA = Math.asin(8d/Rules.getBulletSpeed(bPower));
		final double absAngleTo = Math.atan2(pos.x-myCenter.x,pos.y-myCenter.y);
		return limit(-1d, Utils.normalRelativeAngle(absAngleTo - absBearing)/mEA, 1d) * eLatDir;
	}
	
	private float manDistBetween(final float[] entry, final float[] key) {
		return Math.abs(entry[ACCEL_BIN+2]-key[ACCEL_BIN]) + Math.abs(entry[DIST_BIN+2]-key[DIST_BIN])
			+ Math.abs(entry[VEL_BIN+2]-key[VEL_BIN]) + Math.abs(entry[WALL_BIN+2]-key[WALL_BIN])
				+ Math.abs(entry[REV_WALL_BIN+2]-key[REV_WALL_BIN]) + Math.abs(entry[HEAD_BIN+2]-key[HEAD_BIN])
		+ Math.abs(entry[BULLET_BIN+2]-key[BULLET_BIN]);
	}
	
	private float eucishDistBetween(final float[] entry, final float[] key) {
		float sum = (entry[ACCEL_BIN+2] + key[ACCEL_BIN])*(entry[ACCEL_BIN+2] + key[ACCEL_BIN]);
		for(int i = 1; i < key.length; i ++) {
			sum += (key[i] - entry[i+2])*(key[i] - entry[i+2]);
		}
		return sum;
	}
	
	private float myManDistBetween(final float[] entry, final float[] key) {
		float sum = 0f;
		for(int i = 0; i < entry.length; i ++) {
			sum += Math.abs(entry[i] - key[i]);
		}
		return sum;
	}
	
	private float myEucDistBetween(final float[] entry, final float[] key) {
		float sum = 0f;
		for(int i = 0; i < Math.max(entry.length,key.length); i ++) {
			sum += (key[i]-entry[i])*(key[i]-entry[i]);
		}
		return sum;
	}
	
	void rackOneUp() {
		hits ++;
	}
	
	double limit(final double low, final double mid, final double high) {
		return Math.max(low, Math.min(mid,high));
	}
	
	int limit(final int low, final int mid, final int high) {
		return Math.max(low, Math.min(mid,high));
	}
	
	double sign(final double value) {
		return value >= 0 ? 1d : -1d;
	}

	public void onPaint(final Graphics2D g) {
		double maxDensity = 0.1d;
		for(int i = 0; i < BINS; i ++) {
			if(dangerZones[i] > maxDensity) {
				maxDensity = dangerZones[i];
			}
		}
		final double boxWidth = ClownCar.DISPLAY_WIDTH / (double) BINS;
		g.setColor(Color.BLUE.brighter().brighter());
		for(int i = 0; i < BINS; i ++) {
			g.fill(new Rectangle2D.Double(ClownCar.DISPLAY_X+i*(boxWidth),
						ClownCar.DISPLAY_Y, boxWidth,
					ClownCar.DISPLAY_HEIGHT * (dangerZones[i]/maxDensity)
				)
			);
		}
		g.setColor(Color.YELLOW);
		g.draw(new Rectangle2D.Double(ClownCar.DISPLAY_X, ClownCar.DISPLAY_Y,
				ClownCar.DISPLAY_WIDTH, ClownCar.DISPLAY_HEIGHT
		));
		g.draw(new Line2D.Double(ClownCar.DISPLAY_X + ClownCar.DISPLAY_WIDTH/2d,
				ClownCar.DISPLAY_Y, ClownCar.DISPLAY_X + ClownCar.DISPLAY_WIDTH/2d,
					ClownCar.DISPLAY_Y + ClownCar.DISPLAY_HEIGHT
		));
		g.setColor(Color.PINK.darker());
		for(final Future fut : _futures) {
			g.fill(new Ellipse2D.Double(fut.x-1d,fut.y-1d,2d,2d));
		}
	}
	
	public void reset() {
		launches.clear();
		_futures.clear();
		if(neighborMap.size() > MAX_MAP_SIZE) {
			final List<float[]> oldKeys = neighborMap.keySet().stream()/*.sorted((seg1,seg2)->{
				final int roundComp = Double.compare(seg1[0],seg2[0]);
					if (roundComp == 0) {
							return Double.compare(seg1[1],seg2[1]);
					} return roundComp;
			})*/.limit(neighborMap.size()-SHRINK_TARGET).collect(Collectors.toList());
			System.out.println("Clearing " + oldKeys.size() + " gun datums");
			for(final float[] ok : oldKeys) {
				neighborMap.remove(ok);
			}
		}
		System.out.println("My effective acc : %" + (Math.round((double)hits/Math.max(1d,shots)*10000d)/100d));
	}

}


class StickyGoo {
	private final Point2D.Double source;
	final double groundHeading, mEA, velocity, minFactor, maxFactor, minCrimp, maxCrimp;
	final long fireTime;
	final int dir;
	//final float distSeg, velSeg, accelSeg, wallSpaceSeg, revWallSeg, headSeg, bPowerSeg;
	
	final float[] key;
	
	final boolean isReal;
	
	private boolean isActive;
	
	StickyGoo(final double sourceX, final double sourceY, final double groundHeading,
			final double velocity, final long fireTime,
				final int dir, final double minFactor, final double maxFactor, final float[] key,
			final boolean isReal, final double minCrimp, final double maxCrimp, final float roundNum) {
		this.source = new Point2D.Double(sourceX, sourceY);
		this.mEA = Math.asin(8d/velocity);
		this.velocity = velocity;
		this.groundHeading = groundHeading;
		this.fireTime = fireTime;
		this.dir = dir;
		/*this.distSeg = distSeg;
		this.velSeg = velSeg;
		this.accelSeg = accelSeg;
		this.wallSpaceSeg = wallSpaceSeg;
		this.revWallSeg = revWallSeg;
		this.headSeg = headSeg;
		this.bPowerSeg = bPowerSeg;*/
		this.minFactor = minFactor;
		this.maxFactor = maxFactor;
		this.minCrimp = minCrimp;
		this.maxCrimp = maxCrimp;
		

		this.key = key;// new float[]{roundNum, (float)fireTime, key[0], key[1], key[2], key[3], key[4], key[5], key[6]};//new float[]{roundNum, (float)fireTime, distSeg,velSeg,accelSeg,wallSpaceSeg,revWallSeg,headSeg,bPowerSeg};
		
		this.isActive = true;
		this.isReal = isReal;
	}
	
	boolean checkHit(final Point2D.Double enemy, long currentTime) {
		final double flightDist =  (currentTime - fireTime) * velocity;
		return new Ellipse2D.Double(source.x - flightDist, source.y - flightDist, 2*flightDist, 2*flightDist).intersects(
				new Rectangle2D.Double(enemy.x-18,enemy.y-18,36,36));//source.distance(enemy) <= (currentTime - fireTime) * velocity;
	}
	
	boolean breaks(final Point2D.Double enemy, long currentTime) {
		return enemy.distance(source) <= velocity * (currentTime - fireTime);
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
	
	@Override
	public int hashCode() {
		return source.hashCode() ^ Double.valueOf(groundHeading).hashCode() ^ Double.valueOf(mEA).hashCode() ^ Double.valueOf(velocity).hashCode() ^ Arrays.hashCode(key);
	}
	
	@Override
	public boolean equals(final Object o) {
		if(o instanceof HammerToss) {
			final HammerToss w = (HammerToss)o;
			return this.hashCode() == w.hashCode();
		} return false;
	}
}

class Future extends Point2D.Double {
	private final double dir;
	Future(final double x, final double y, final double dir) {
		super(x,y);
		this.dir = dir;
	}
	double dir() {
		return dir;
	}
}
