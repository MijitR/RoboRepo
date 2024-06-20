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
 * MyClass - a class by Damij
 */
public class BanHammer
{

	//KEEP BINS ODD BUT HEAD_SEGS EVEN
	static final int BINS = 83, ACCEL_SEGS = 3, DIST_SEGS = 6, VEL_SEGS = 8, WALL_DIST_SEGS = 5, HEAD_SEGS = 5, B_POWER_SEGS = 3,
		ACCEL_BIN=0,DIST_BIN=1,VEL_BIN=2,WALL_BIN=3,REV_WALL_BIN=4,HEAD_BIN=5,BULLET_BIN=6;
		
	private static float[][][][][][][][] stats = new float[ACCEL_SEGS][DIST_SEGS][VEL_SEGS][WALL_DIST_SEGS][WALL_DIST_SEGS][HEAD_SEGS][B_POWER_SEGS][BINS];
	//Used for Polyps
	private static float[][][][][][][] weights = new float[2][2][2][2][2][2][2];
	
	static {
		for(int a = 0; a < weights.length; a ++) {
			for(int d = 0; d < weights[a].length; d ++) {
				for(int v = 0; v < weights[a][d].length; v ++) {
					for(int w = 0; w < weights[a][d][v].length; w ++) {
						for(int wb = 0; wb < weights[a][d][v][w].length; wb ++) {
							for(int h = 0; h < weights[a][d][v][w][wb].length; h ++) {
								for(int b = 0; b < weights[a][d][v][w][wb][h].length; b ++) {
									weights[a][d][v][w][wb][h][b] = //(float)Math.exp(-0.2d*(a+d+v+w+wb+h+b));
													Math.min(1f,(float)((0.6d/Math.log(a+d+v+w+wb+h+b+2) - 0.1d)/2d));
								}
							}
						}
					}
				}
			}
		}
	}
		
	private final HashMap<float[], Double> flatMap;
	
	final List<HammerToss> waves;
	
	final AdvancedRobot self;
	final Point2D.Double enemyPos, myCenter, myOldCenter;
	
	private final List<Future> _futures;
	
	final double distSegSize = 1000d/(double)DIST_SEGS, wallDistSegSize = 1000d/(double)WALL_DIST_SEGS, WALL_STICK = (8d), bPowerSegSize = 3.01d/B_POWER_SEGS;
	
	final int MAX_MAP_SIZE = 345, SHRINK_TARGET = 324, NUM_NEIGHBORS = 17;

	final double[] gfBuffer = new double[2], boundingFactors = new double[2], crimpOffsets;	

	double[] guessFactors;

	double bPower, oldVelocity, healthDisadvantage, mainAcc = 0d, flatAcc = 0d, shotPower,
			knnConfidence, vcsConfidence, confidence, realAcc, realHits, realShots, eVelocity,
			maxOffset, minOffset, maxCrimp, minCrimp, lastAbsBearing;
	long timeToShoot, passCount;
	int eDir, eLatDir, mainGunUses, flattenerUses, shots, mainHits, flatHits, bhbs, lastELatDir;
	

	float currDistSeg, currWallSeg, revWallSeg, currVelSeg, accelSeg, headSeg, bPowerSeg,
		lastWDSeg, lastRevWDSeg, lastDSeg, lastVSeg,lastASeg,lastHSeg,lastBPSeg;
		
	
	boolean usedFlattenerGun, isMC;
	
	public BanHammer(final AdvancedRobot self, final boolean isMC) {
		this.self = self;
		
		this.waves = new ArrayList<>();
		this.flatMap = new HashMap<>();
		this.enemyPos = new Point2D.Double(self.getBattleFieldWidth()/2d,
				self.getBattleFieldHeight()/2d);
		this.myCenter = new Point2D.Double(enemyPos.x, enemyPos.y);
		this.myOldCenter = new Point2D.Double(0,0);
		
		this._futures = new ArrayList<>();
		
		this.shotPower = 0.1d;
				
		this.eDir = 1;
		this.isMC = isMC;
		
		this.crimpOffsets = new double[4];
	}

	public Bullet update(final ScannedRobotEvent e, final double absBearing) {
		this.healthDisadvantage = Math.min(3,Math.max(-3,(e.getEnergy() - self.getEnergy())/25d));
		
		final Bullet shot;

		final double targetAngleWidth = 36d/e.getDistance();
		
		myCenter.x = self.getX();// + Math.sin(self.getHeadingRadians()) * self.getVelocity();
		myCenter.y = self.getY();// + Math.cos(self.getHeadingRadians()) * self.getVelocity();
		
		enemyPos.x = myCenter.x + Math.sin(absBearing) * e.getDistance();// + Math.sin(e.getHeadingRadians()) * e.getVelocity();
		enemyPos.y = myCenter.y + Math.cos(absBearing) * e.getDistance();// + Math.cos(e.getHeadingRadians()) * e.getVelocity();

		final Iterator<HammerToss> wavtor = waves.iterator();
		while(wavtor.hasNext()) {
			final HammerToss storage;
			if((storage=wavtor.next()).checkHit(enemyPos, self.getTime())) {
				wavtor.remove();
				storeWave(storage, enemyPos);
				passCount ++;
			}
		}	//Math.sin(e.getHeadingRadians()-absBearing)*eDir > 0 ? 1 : -1;
		
		
		
		eDir = e.getVelocity() == 0 ? eDir : e.getVelocity() > 0 ? 1 : -1;

		double bearingDiff, latTemp;
		eLatDir = (latTemp=Math.sin(-(absBearing-e.getHeadingRadians()))*e.getVelocity()) == 0 ? eLatDir : latTemp > 0 ? 1 : -1;
		//eLatDir = (bearingDiff=Utils.normalRelativeAngle(absBearing - lastAbsBearing))== 0 ? eLatDir : bearingDiff > 0 ? 1 : -1;//Math.sin(e.getHeadingRadians()-absBearing)*(eVelocity=e.getVelocity()) == 0 ? eLatDir :
		//System.out.println("ELatDir: " + eLatDir);
		guessFactors = decideGun(e, targetAngleWidth);		


		if(timeToShoot == self.getTime() && self.getGunTurnRemainingRadians() == 0d && ((bPower < self.getEnergy() && bPower > 0) || isMC) && self.getGunHeat() == 0
			//		&& Math.cos((self.getHeadingRadians() + (sign(self.getVelocity() > 0 ? 0d : Math.PI)))-absBearing)>=0

			) {
			shot = self.setFireBullet(bPower);
			shotPower = bPower;
			
			if(shot!= null) {
				//final double eDeltaX = Math.sin(e.getHeadingRadians()) * e.getVelocity(), eDeltaY = Math.cos(e.getHeadingRadians()) * e.getVelocity(),
				//	deltaX = Math.sin(self.getHeadingRadians()) * self.getVelocity(), deltaY = Math.cos(self.getHeadingRadians()) * self.getVelocity();
				//System.out.println("Shot at " + (usedFlattenerGun ? guessFactors[1] : guessFactors[0]));
				waves.add(new HammerToss(myCenter.x, myCenter.y, lastAbsBearing,//Math.atan2(enemyPos.x-eDeltaX+deltaX-myCenter.x,enemyPos.y-eDeltaY+deltaY-myCenter.y),
					Rules.getBulletSpeed(shotPower), self.getTime(), lastELatDir, lastDSeg, lastVSeg, lastASeg, lastWDSeg, lastRevWDSeg, lastHSeg, lastBPSeg, true, gfBuffer[0], gfBuffer[1],
					crimpOffsets[1], crimpOffsets[3], crimpOffsets[0], crimpOffsets[2]));
			
				if(usedFlattenerGun) {
					flattenerUses ++;
				} else {
					mainGunUses ++;
				}
			
			}
		} else {
			shot = null;
		}
		
		bPower = 1.5d - 0.1d*(healthDisadvantage+2);//+ 2.15d * (confidence-0.05);
		
		if(e.getDistance() < 536) {
			bPower = (1d - (e.getDistance()-36d)/500d) * (3d-bPower) + bPower;
		}
		 if(self.getEnergy() < 3) {
			bPower = 0.1d;
		} else  if(self.getEnergy() < 7) {
			bPower = Math.min(bPower,0.2d);
		} else  if(self.getEnergy() < 12) {
			bPower = Math.min(0.4d, bPower);
		} else if(self.getEnergy() < 16) {
			bPower -= 1d;
		}
		
		if(e.getEnergy() <= Rules.getBulletDamage(bPower)) {
			bPower = (e.getEnergy() + 2d) / 6d;
			if(e.getEnergy() <= 4) {
				bPower = Math.max(0.1d, e.getEnergy() / 4d);
			}
		}
		
		//bPower = Math.ceil(bPower*10d)/10d;
			
		bPower = limit(0d, Math.min(bPower, self.getEnergy()-0.1), 3d);	

		if(isMC)
			bPower = Math.min(self.getEnergy(),3d);
		
		confidence = vcsConfidence;
		if(flatAcc > mainAcc) {
			confidence = knnConfidence;
		}	
			
		this.lastAbsBearing = absBearing;
		this.lastELatDir = eLatDir;
		
		double guessFactor = guessFactors[0];
		usedFlattenerGun = false;
		//if(flatAcc > mainAcc) {
		//if(flatAcc > mainAcc) {
		if(boundingFactors[1]/flatAcc < boundingFactors[0]/mainAcc) {
			usedFlattenerGun = true;
			guessFactor = guessFactors[1];
		}
		
		final double angleOffset = eLatDir * guessFactor * Math.asin(8d/(20d-3d*bPower));
		final double angleBonus = ((float)bhbs)/(Math.max(1f,shots)) > 0.5f ? (Math.random()-0.5d) * targetAngleWidth * 0.2d : 0d;		

		self.setTurnGunLeftRadians(
			Utils.normalRelativeAngle(
				self.getGunHeadingRadians() - absBearing - angleOffset + angleBonus
			)
		);
		
		timeToShoot = self.getTime() + 1;
		
		gfBuffer[0] = guessFactors[0];
		gfBuffer[1] = guessFactors[1];
		
		return shot;
	}
	
	private double[] decideGun(final ScannedRobotEvent e, final double targetAngleWidth) {
	
			this.lastDSeg = currDistSeg;
			this.lastVSeg = currVelSeg;
			this.lastASeg = accelSeg;
			this.lastWDSeg = currWallSeg;
			this.lastRevWDSeg = revWallSeg;
			this.lastHSeg = headSeg;
			this.lastBPSeg = bPowerSeg;
	
		final double eDeltaX = Math.sin(e.getHeadingRadians()) * e.getVelocity(), eDeltaY = Math.cos(e.getHeadingRadians()) * e.getVelocity(),
					deltaX = Math.sin(self.getHeadingRadians()) * self.getVelocity(), deltaY = Math.cos(self.getHeadingRadians()) * self.getVelocity();
					
		final double absBearing = Math.atan2(enemyPos.x-myCenter.x, enemyPos.y-myCenter.y);
		
		double eGoing = eDir < 0 ? Math.PI + e.getHeadingRadians() : e.getHeadingRadians();
		
		final Point2D.Double[] nexts
				= getMaxPosAndCrimps();

		this.crimpOffsets[0] = minOffset;
		this.crimpOffsets[1] = minCrimp;
		this.crimpOffsets[2] = maxOffset;
		this.crimpOffsets[3] = maxCrimp;
				
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
		accelSeg = Math.min(Double.compare(Math.abs(e.getVelocity()), Math.abs(oldVelocity)) + 1, ACCEL_SEGS-1);
		headSeg = (float)Math.min(HEAD_SEGS - 1, (Math.abs(Math.sin(e.getHeadingRadians()-absBearing)) * HEAD_SEGS));
		

		final double wallDistLat = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (self.getBattleFieldWidth()-enemyPos.x) / (Math.cos((Math.PI/2d - eGoing)))
			: enemyPos.x / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirt = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(self.getBattleFieldHeight()-enemyPos.y) / (Math.cos(eGoing)) : enemyPos.y / (Math.cos(Math.PI - eGoing));
		
		currWallSeg = Math.max(0,Math.min(WALL_DIST_SEGS-1,(float)(Math.min(wallDistLat, wallDistVirt) / wallDistSegSize)));
		
		eGoing += Math.PI;
		eGoing = Utils.normalRelativeAngle(eGoing);
		
		final double wallDistLatRev = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (self.getBattleFieldWidth()-enemyPos.x) / (Math.cos((Math.PI/2d - eGoing)))
			: enemyPos.x / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirtRev = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(self.getBattleFieldHeight()-enemyPos.y) / (Math.cos(eGoing)) : enemyPos.y / (Math.cos(Math.PI-Utils.normalAbsoluteAngle(eGoing)));
		
		revWallSeg = Math.max(0, Math.min(WALL_DIST_SEGS-1,(float)(Math.min(wallDistLatRev, wallDistVirtRev)/ wallDistSegSize)));
		

		bPowerSeg = (float)(bPower/bPowerSegSize);
			
		oldVelocity = e.getVelocity();

		//ACCEL_BIN=0,DIST_BIN=1,VEL_BIN=2,WALL_BIN=3,HEAD_BIN=4,BULLET_BIN=5;
		final float[] currSegment = stats[(int)accelSeg][(int)currDistSeg][(int)currVelSeg][(int)currWallSeg][(int)revWallSeg][(int)headSeg][(int)bPowerSeg];
		
		int targetBinWidth = (int)(((targetAngleWidth)/(0.5d*Math.asin(8/Rules.getBulletSpeed(bPower))))*(((double)BINS-1)/2));
		
		if((targetBinWidth&1)==0) {
			targetBinWidth ++;
		}
		
		//System.out.println("Dist Seg: " + currDistSeg + "\nVelSeg: " + currVelSeg + "\nAccelSeg: " + accelSeg + "\nHeadSeg: " + headSeg + "\nBPowerSeg: " + bPowerSeg);
		//System.out.println("Wall Seg: " + currWallSeg + " and " + revWallSeg);
		
		//System.out.println("Widt " + targetBinWidth);
		
		int bestMainIndexCenter = 13*(BINS-1)/32 + (BINS-1)/2;
		double maxDanger = currSegment[bestMainIndexCenter], totalDanger = 0d;
		//for(int i = 0; i < BINS; i ++) {
	//		totalDanger += currSegment[i];
		//}
		
		for(int i = 0; i < BINS; i ++) {
			double dangerAt = currSegment[i];
								//0d;
			//for(int j = i-(targetBinWidth-1)/2; j <= i + (targetBinWidth-1)/2; j ++) {
			//	dangerAt += currSegment[limit(0,j,BINS-1)];
			//}
			totalDanger += dangerAt;
			if(dangerAt > maxDanger) {
				maxDanger = dangerAt;
				bestMainIndexCenter = i;
			}
		}
		vcsConfidence = maxDanger / Math.max(0.01d,totalDanger);
		double guessFactorMain = ((double) (bestMainIndexCenter - (BINS-1)/2)) / (double) ((BINS-1)/2);
		//bor = bor < minmin ? minF*(bor-minmin) + minmin : bor > maxmax ? maxF*(bor-maxmax) + maxmax : bor;
		guessFactorMain = guessFactorMain < minCrimp ? minOffset*(guessFactorMain-minCrimp) + minCrimp : guessFactorMain > maxCrimp ? maxOffset*(guessFactorMain-maxCrimp) + maxCrimp : guessFactorMain;
	
		final float[] currKey = new float[]{accelSeg, currDistSeg, currVelSeg, currWallSeg, revWallSeg, headSeg, bPowerSeg};
		final List<float[]> neighbors = flatMap.keySet().stream().sorted((k1,k2)->Double.compare(myManDistBetween(k1,currKey),myManDistBetween(k2,currKey))).limit(NUM_NEIGHBORS).collect(Collectors.toList());
		final double stDev = targetAngleWidth/(2d*Math.asin(8d/Rules.getBulletSpeed(bPower)));
		
		//System.out.println("VCS Conf: " + vcsConfidence);

		double maxDensity = 0;
		double guessFactorFlattener = 0d;
		totalDanger = NUM_NEIGHBORS;
		//for(int i = 0; i < BINS; i ++) {
		for(final float[] test : neighbors) {
			double density = 0;
			double runner = 1d;
			final double binFactor = flatMap.get(test);
			for(final float[] neigh : neighbors) {
				final double memory = flatMap.get(neigh);
			//	final double binFactor = f((double)(i-(BINS-1)/2)) / (double) ((BINS-1)/2);
				density += Math.exp(-0.5d*((binFactor-memory)*(binFactor-memory)/(stDev*stDev)));
				//for(int j = i-(targetBinWidth-1)/2; j <= i + (targetBinWidth-1)/2; j ++) {
				//	final double testFactor = ((double) (j - (BINS-1)/2)) / (double) ((BINS-1)/2);
				//	density += Math.exp(-0.5d*((testFactor-memory)*(testFactor-memory)/(stDev*stDev))) * runner;
				//}
//				runner *= 0.9d;
			}
			if(density > maxDensity) {
				maxDensity = density;
				guessFactorFlattener =  binFactor;//((double) (i - (BINS-1)/2)) / (double) ((BINS-1)/2);//testFactor;
			}
		}
		
		guessFactorFlattener = guessFactorFlattener < minCrimp ? minOffset*(guessFactorFlattener-minCrimp) + minCrimp : guessFactorFlattener > maxCrimp ? maxOffset*(guessFactorFlattener-maxCrimp) + maxCrimp : guessFactorFlattener;
		knnConfidence = maxDensity / Math.max(0.00001d,totalDanger);
		
		//System.out.println("KNN Conf: " + knnConfidence);
		//For updating the paint method
	//	if(self.getTime() % 9 == 0) {
			
	//	}
	
		
		
		return new double[]{guessFactorMain, guessFactorFlattener};
	}
	
	private void storeWave(final HammerToss wave, final Point2D.Double target) {
		final double botAngleWidth = Math.asin(18d/enemyPos.distance(new Point2D.Double(wave.sourceX(),wave.sourceY())))/wave.mEA;//36d/enemyPos.distance(new Point2D.Double(wave.sourceX(),wave.sourceY()));//2d*Math.asin(18d/enemyPos.distance(new Point2D.Double(wave.sourceX(),wave.sourceY())));
		final double binAngleWidth = (2d*wave.mEA / (double)((BINS)));
		final double resultAngle = Math.atan2(enemyPos.x - wave.sourceX(), enemyPos.y - wave.sourceY());
		final double bearingOffset = Utils.normalRelativeAngle(resultAngle - wave.groundHeading);
		double guessFactor = Math.max(-1d, Math.min(1d, bearingOffset / wave.mEA)) * wave.dir;
/*		
		shots ++;
*/		
/*		if(Math.abs(Utils.normalRelativeAngle(wave.mainGunFactor * wave.mEA * wave.dir - guessFactor * wave.mEA * wave.dir)) < botAngleWidth / 2d) {
			mainHits ++;
		}
		if(Math.abs(Utils.normalRelativeAngle(wave.flatGunFactor * wave.mEA * wave.dir - guessFactor * wave.mEA * wave.dir)) < botAngleWidth / 2d) {
			flatHits ++;
		}
		
		final double mainBoundingFactor = Math.log1p(Math.abs(guessFactor-wave.mainGunFactor)),
			flatBoundingFactor = Math.log1p(Math.abs(guessFactor-wave.flatGunFactor));
*/
			
		guessFactor = guessFactor > wave.maxCrimp ? (guessFactor-wave.maxCrimp)/wave.maxFactor + wave.maxCrimp
					: guessFactor < wave.minCrimp ? (guessFactor-wave.minCrimp)/wave.minFactor + wave.minCrimp : guessFactor;
		int index = (int) Math.round((BINS-1)/2*(guessFactor+1d));
		final double botBinWidthD =((botAngleWidth / binAngleWidth));
		final double stDev = botBinWidthD;
		final int botBinWidth = (int)Math.ceil(botBinWidthD);
		
		if(Math.abs(Utils.normalRelativeAngle(wave.mainGunFactor * wave.mEA * wave.dir - guessFactor * wave.mEA * wave.dir)) < botAngleWidth / 2d) {
			mainHits ++;
		}
		if(Math.abs(Utils.normalRelativeAngle(wave.flatGunFactor * wave.mEA * wave.dir - guessFactor * wave.mEA * wave.dir)) < botAngleWidth / 2d) {
			flatHits ++;
		}
		
		final double mainBoundingFactor = /*Math.log1p*/(Math.abs(guessFactor-wave.mainGunFactor)),
			flatBoundingFactor = /*Math.log1p*/(Math.abs(guessFactor-wave.flatGunFactor));
	
			
		boundingFactors[0] = (boundingFactors[0]*Math.min(shots,25) + mainBoundingFactor * 1) / (Math.min(shots+1,26) + 1);
		boundingFactors[1] = (boundingFactors[1]*Math.min(shots,25) + flatBoundingFactor * 1) / (Math.min(shots+1,26) + 1);
		
		//if(wave.isReal) {
			
		/*	double total = 0d;
			for(int i = 0; i < BINS; i ++) {
				total += stats[wave.accelSeg][wave.distSeg][wave.velSeg][wave.wallSpaceSeg][wave.revWallSeg][wave.headSeg][wave.bPowerSeg][i];
			}
		*/	
			
		shots ++;
			this.mainAcc = (this.mainAcc * Math.min(shots-1,75d) + (mainHits / (double) shots)) / Math.min(shots,76d);
			this.flatAcc = (this.flatAcc * Math.min(shots-1,75d) + (flatHits / (double) shots)) / Math.min(shots,76d);
			
			for(int i = Math.max(0,index-(2*botBinWidth-1)); i < Math.min(BINS,index+(2*botBinWidth-1)+1); i ++) {
				
				//stats[wave.accelSeg][wave.distSeg][wave.velSeg][wave.wallSpaceSeg][wave.revWallSeg][wave.headSeg][wave.bPowerSeg][i] /= 0.2*total;
				float main = (float)Math.exp(-.5d*((i-index)*(i-index)/(stDev*stDev)));
				for(int a = Math.max(0, (int)wave.accelSeg-1); a < Math.min(ACCEL_SEGS,(int)wave.accelSeg+2); a ++) {
					final int a1 =  Math.abs((int)wave.accelSeg-a);
					for(int d = Math.max(0, (int)wave.distSeg-1); d < Math.min(DIST_SEGS,(int)wave.distSeg+2); d ++) {
						final int d1 = (Math.abs(d-(int)wave.distSeg));
						for(int v = Math.max(0, (int)wave.velSeg-1); v < Math.min(VEL_SEGS,(int)wave.velSeg+2); v ++) {
							final int v1 = (Math.abs(v-(int)wave.velSeg));
							for(int w = Math.max(0, (int)wave.wallSpaceSeg-1); w < Math.min(WALL_DIST_SEGS,(int)wave.wallSpaceSeg+2); w ++) {
								final int w1 = (Math.abs(w-(int)wave.wallSpaceSeg));
								for(int wr = Math.max(0, (int)wave.revWallSeg-1); wr < Math.min(WALL_DIST_SEGS,(int)wave.revWallSeg+2); wr++) {
									final int w2 = Math.abs(wr-(int)wave.revWallSeg);
									for(int h = Math.max(0, (int)wave.headSeg-1); h < Math.min(HEAD_SEGS,(int)wave.headSeg+2); h ++) {
										final int h1 = (Math.abs(h-(int)wave.headSeg));
										for(int b = Math.max(0, (int)wave.bPowerSeg-1); b < Math.min(B_POWER_SEGS,(int)wave.bPowerSeg+2); b ++) {
											final int b1 = Math.abs(b-(int)wave.bPowerSeg);
											//stats[a][d][v][w][wr][h][b][i] = 
												//(stats[a][d][v][w][wr][h][b][i] * Math.min(shots,MAX_MAP_SIZE) + (float)(weights[a1][d1][v1][w1][w2][h1][b1] * main) * 1f) / (Math.min(shots,(float)MAX_MAP_SIZE) + 1f);
											stats[a][d][v][w][wr][h][b][i] += (float)(weights[a1][d1][v1][w1][w2][h1][b1] * main);
										}
									}
								}
							}
						}
					}
				}
				//stats[(int)wave.accelSeg][(int)wave.distSeg][(int)wave.velSeg][(int)wave.wallSpaceSeg][(int)wave.revWallSeg][(int)wave.headSeg][(int)wave.bPowerSeg][i] =
				//	(stats[(int)wave.accelSeg][(int)wave.distSeg][(int)wave.velSeg][(int)wave.wallSpaceSeg][(int)wave.revWallSeg][(int)wave.headSeg][(int)wave.bPowerSeg][i] * Math.min(shots,100f)
				//		+ main * 1f) / (float) (Math.min(shots,100f) + 1);
			}
		//} else {
			flatMap.put(new float[]{wave.accelSeg,wave.distSeg,wave.velSeg,wave.wallSpaceSeg,wave.revWallSeg,wave.headSeg,wave.bPowerSeg}, guessFactor);
			
		//}
	}
	
	public void rackOneUp() {
		this.realHits ++;
	}
	
	public void countaShield() {
		this.bhbs ++;
	}
	
	float manDistBetween(final float[] entry, final float[] key) {
		return Math.abs(entry[ACCEL_BIN+2]-key[ACCEL_BIN]) + Math.abs(entry[DIST_BIN+2]-key[DIST_BIN])
			+ Math.abs(entry[VEL_BIN+2]-key[VEL_BIN]) + Math.abs(entry[WALL_BIN+2]-key[WALL_BIN])
				+ Math.abs(entry[REV_WALL_BIN+2]-key[REV_WALL_BIN]) + Math.abs(entry[HEAD_BIN+2]-key[HEAD_BIN])
		+ Math.abs(entry[BULLET_BIN+2]-key[BULLET_BIN]);
	}
	
	float myManDistBetween(final float[] entry, final float[] key) {
		float dist = 0f;
		for(int i = 0; i < entry.length; i ++) {
			dist += Math.abs(entry[i]-key[i]);
		}
		return dist;
	}
	
	double limit(final double low, final double mid, final double high) {
		return Math.max(low, Math.min(mid,high));
	}
	
	int limit(final int low, final int mid, final int high) {
		return Math.max(low, Math.min(mid,high));
	}
	
	double sign(final double value) {
		return Double.compare(value,0d);
	}
	
	private double calcOffset(final Point2D.Double pos, final double absBearing) {
		final double mEA = Math.asin(8d/Rules.getBulletSpeed(bPower));
		final double absAngleTo = Math.atan2(pos.x-myCenter.x,pos.y-myCenter.y);
		return limit(-1d, Utils.normalRelativeAngle(absAngleTo - absBearing)/mEA, 1d) * eLatDir;
	}
	
	
	
	private Point2D.Double[] getMaxPosAndCrimps() {
		final Point2D.Double nextBack = new Point2D.Double(enemyPos.x,enemyPos.y),
			nextFront = new Point2D.Double(enemyPos.x,enemyPos.y);
		final double waveVelocity = Rules.getBulletSpeed(bPower);
		final double escAngleBonus  = 0d;
		
		final Point2D.Double myNextPos = project(myCenter, self.getHeadingRadians(), self.getVelocity());
		
		final Point2D.Double frontCrimp = new Point2D.Double(enemyPos.x,enemyPos.y),
			backCrimp = new Point2D.Double(enemyPos.x,enemyPos.y);
		_futures.clear();
		if(Math.abs(eVelocity) <= 2d) {
			_futures.add(new Future(enemyPos.x,enemyPos.y, 0));
		}
		for(double dir = -1d; dir <= 1d; dir += 2d) {
			Point2D.Double future =
				(Point2D.Double) enemyPos.clone();
			double futureVelocity = eVelocity;
			double bareing = Math.atan2(enemyPos.x - myCenter.x, enemyPos.y - myCenter.y);
			double runningDir = eDir, futDub = 0d;
			double runningLatDir = eLatDir;
			double futureHeading = bareing + (Math.PI / 2d) * runningLatDir;
			futureHeading += runningDir < 0 ? Math.PI : 0d;
			boolean hitWall = false;
			for(int futures = 0;
					futures < 1000d && !new Ellipse2D.Double(myNextPos.x-waveVelocity*futures,myNextPos.y-waveVelocity*futures,2*waveVelocity*futures,2*waveVelocity*futures)
						.intersects(new Rectangle2D.Double(future.x-13d,future.y-13d,26d,26d)); futDub += 0.9d /*hitWall ? Math.max(0.5d, Math.min(0.98,0.5d+Math.abs(Math.abs(futureVelocity))/16d)) : 1d*/, futures = (int)futDub) {
				//futureVelocity += dir * eLatDir > 0 ? sign(futureVelocity)*dir*eLatDir : 2d*sign(futureVelocity)*dir*eLatDir;
				futureVelocity += dir*futureVelocity > 0 ? dir : 2*dir;
				futureVelocity = limit(-8d, futureVelocity, 8d);
				runningDir = futureVelocity == 0d ? runningDir : sign(futureVelocity);
				runningLatDir = runningDir*sign(Utils.normalRelativeAngle(futureHeading - bareing));
				futureHeading = Utils.normalRelativeAngle(bareing + Math.PI/2d*runningLatDir);
				futureHeading += runningDir < 0 ? Math.PI : 0d;
				final double smoothedHeading = wallSmooth(future, futureHeading, dir*sign(Utils.normalRelativeAngle(futureHeading - bareing)), (int)dir);
				if(!hitWall && Math.abs(Utils.normalRelativeAngle(futureHeading - smoothedHeading)) <= Math.PI/35d) {
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
		double heading, final double latDir, final int velDir)
	{
		int tries = 0;
		while(
			!Bezier.playTains(
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

	private Point2D.Double project(final double x, final double y,
		final double heading, final double dist)
	{
		return  new Point2D.Double(
			x + Math.sin(heading) * dist,
			y + Math.cos(heading) * dist
		);
	}

	public void onPaint(final Graphics2D g) {
			final long currTime = self.getTime();
		for(final HammerToss w : waves) {
			if(w.isReal) {
				g.setColor(Color.BLUE);
				final double distTravelled = w.velocity*(currTime - w.fireTime);
				g.draw(new Ellipse2D.Double(w.sourceX() - distTravelled, w.sourceY() - distTravelled, 2*distTravelled, 2*distTravelled));
				final Point2D.Double targetMain = project(w.sourceX(), w.sourceY(), w.mainGunFactor * w.mEA * w.dir + w.groundHeading, distTravelled),
					targetKNN = project(w.sourceX(), w.sourceY(), w.flatGunFactor * w.mEA * w.dir + w.groundHeading, distTravelled),
					tailMain = project(w.sourceX(), w.sourceY(), w.mainGunFactor * w.mEA * w.dir + w.groundHeading, distTravelled - w.velocity),
					tailKNN = project(w.sourceX(), w.sourceY(), w.flatGunFactor * w.mEA * w.dir + w.groundHeading, distTravelled - w.velocity);
				g.setColor(Color.ORANGE.darker());
				g.draw(new Line2D.Double(tailMain.x,tailMain.y,targetMain.x,targetMain.y));
				g.setColor(Color.GREEN.darker());
				g.draw(new Line2D.Double(tailKNN.x, tailKNN.y, targetKNN.x, targetKNN.y));
			}
		}
		final float[] segmentValues = stats[(int)this.lastASeg][(int)this.lastDSeg][(int)this.lastVSeg][(int)this.lastWDSeg][(int)this.lastRevWDSeg][(int)this.lastHSeg][(int)this.lastBPSeg];
		final double boxWidth = Bezier.DISPLAY_WIDTH / (double) BINS;
		
		g.setColor(Color.MAGENTA);
		for(final Future fut : _futures) {
			g.draw(new Ellipse2D.Double(fut.x-1,fut.y-1,2,2));
		}
		
		double maxValue = 0;
		for(int i = 0; i < BINS; i ++) {
			if(segmentValues[i]>maxValue) {
				maxValue = segmentValues[i];
			}
		}
		if(!(Double.isNaN(maxValue) || Double.isInfinite(maxValue) || Double.compare(maxValue,0) == 0)) {
		
		g.setColor(new Color((int)limit(0,this.vcsConfidence*255,255),128,128));
		for(int i = 0; i < BINS; i ++) {
			g.fill(new Rectangle2D.Double(Bezier.DISPLAY_X+i*(boxWidth), Bezier.DISPLAY_Y, boxWidth, segmentValues[i]/maxValue*Bezier.DISPLAY_HEIGHT));
		}
		}
		g.setColor(Color.YELLOW.brighter());
		g.draw(new Rectangle2D.Double(Bezier.DISPLAY_X,Bezier.DISPLAY_Y,Bezier.DISPLAY_WIDTH,Bezier.DISPLAY_HEIGHT));
		g.draw(new Line2D.Double((Bezier.DISPLAY_X+Bezier.DISPLAY_WIDTH)/2d,Bezier.DISPLAY_Y,(Bezier.DISPLAY_X+Bezier.DISPLAY_WIDTH)/2d,Bezier.DISPLAY_Y+Bezier.DISPLAY_HEIGHT));
	}
	
	public void reset() {
		waves.clear();
		if(flatMap.size() > MAX_MAP_SIZE) {
			System.out.println("Clearing " + (flatMap.size()-SHRINK_TARGET) + " gun datums");
			final List<float[]> oldKeys = flatMap.keySet().stream()/*.sorted((seg1,seg2)->{final int roundComp = Double.compare(seg1[0],seg2[0]); if (roundComp == 0) {return Double.compare(seg1[1],seg2[1]);} return roundComp;})*/.limit(flatMap.size()-SHRINK_TARGET).collect(Collectors.toList());
			for(final float[] ok : oldKeys) {
				flatMap.remove(ok);
			}
		}
		
	//flatMap.clear();
		System.out.println("VCS Gun uses     : " + mainGunUses + " acc: %" + (Math.round(mainAcc*10000d)/100d));
		System.out.println("KNN Gun uses     : " + flattenerUses + " acc %" + (Math.round(flatAcc*10000d)/100d));
		System.out.println("My Real acc      :  %" + (Math.round(realHits/Double.max(1d,shots)*10000d)/100d));
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

class HammerToss {
	private final Point2D.Double source;
	final double groundHeading, mEA, velocity, mainGunFactor, flatGunFactor,
		minCrimp, maxCrimp, minFactor, maxFactor;
	final long fireTime;
	final int dir;
	final float distSeg, velSeg, accelSeg, wallSpaceSeg, revWallSeg, headSeg, bPowerSeg;
	
	final boolean isReal;
	
	private boolean isActive;
	
	HammerToss(final double sourceX, final double sourceY, final double groundHeading,
			final double velocity, final long fireTime,
				final int dir, final float distSeg,
					final float velSeg, final float accelSeg, final float wallSpaceSeg, final float revWallSeg, final float headSeg, final float bPowerSeg,
			final boolean isReal, final double mainGunFactor, final double flatGunFactor, final double minCrimp,
					final double maxCrimp, final double minFactor, final double maxFactor) {
		this.source = new Point2D.Double(sourceX, sourceY);
		this.mEA = Math.asin(8d/velocity);
		this.velocity = velocity;
		this.groundHeading = groundHeading;
		this.fireTime = fireTime;
		this.dir = dir;
		this.distSeg = distSeg;
		this.velSeg = velSeg;
		this.accelSeg = accelSeg;
		this.wallSpaceSeg = wallSpaceSeg;
		this.revWallSeg = revWallSeg;
		this.headSeg = headSeg;
		this.bPowerSeg = bPowerSeg;
		
		this.minCrimp = minCrimp;
		this.maxCrimp = maxCrimp;
		this.minFactor = minFactor;
		this.maxFactor = maxFactor;
		
		this.isActive = true;
		this.isReal = isReal;
		
		this.mainGunFactor = mainGunFactor;
		this.flatGunFactor = flatGunFactor;
	}
	
	boolean checkHit(final Point2D.Double enemy, long currentTime) {
		final double flightDist =  (currentTime - fireTime) * velocity;
		return new Ellipse2D.Double(source.x - flightDist, source.y - flightDist, 2*flightDist, 2*flightDist).intersects(
				new Rectangle2D.Double(enemy.x-18,enemy.y-18,36,36));//source.distance(enemy) <= (currentTime - fireTime) * velocity;
	}
	
	boolean breaks(final Point2D.Double enemy, final long currentTime) {
		return (currentTime - fireTime) * velocity >= enemy.distance(source);
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
		return source.hashCode() ^ Double.valueOf(groundHeading).hashCode() ^ Double.valueOf(mEA).hashCode() ^ Double.valueOf(velocity).hashCode() ^ ((int)fireTime) ^ dir ^ Double.valueOf(wallSpaceSeg).hashCode() ^ Double.valueOf(revWallSeg).hashCode();
	}
	
	@Override
	public boolean equals(final Object o) {
		if(o instanceof HammerToss) {
			final HammerToss w = (HammerToss)o;
			return fireTime == w.fireTime && Math.abs(velocity-w.velocity) <= 0.2d
				&& distSeg == w.distSeg;
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