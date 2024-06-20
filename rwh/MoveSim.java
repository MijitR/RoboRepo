package rwh;
import rwh.util.Rut;
import robocode.*;
import robocode.util.Utils;
import java.util.List;
import java.util.ArrayList;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.Color;
import java.util.ArrayList;
import java.lang.reflect.Array;
import java.awt.geom.Ellipse2D;
import java.util.Iterator;
import java.awt.geom.RoundRectangle2D;
import java.util.Arrays;
import java.awt.geom.Rectangle2D;
import java.util.HashMap;
import java.util.stream.Stream;
import java.util.stream.Collectors;

/**
 * MyClass - a class by Damij
 */
public class MoveSim
{
	
	//Using fill-paths, this must be >= 3
	//For current uses, must be = 1
	public static final int HALF_PATH_COUNT = 1;
	
	public static final int BINS = 255;
	
	public static final int NUM_NEIGHB0RS = 34;
	
	public static final int MAX_MAP_SIZE = 1400;

	private final Self self;
	
	private final HashMap<float[], Double> offsetMap;
	
	private final List<Wave> _waves;
	
	private final List<FutureImage>[] paths;
	
	private final List<Point2D> secondPaths;
	
	private final Enemy enemy;
	
	private final Point2D target, center;
	
	private final RoundRectangle2D arena;
	
	private final float[][][][][][] vcs;
	
	private Wave surfWave, secondWave;
	
	private float[][] neighbors, secondNeighbors;
	
	private double lastVelocity, velocity, lastDistance, distanceToEnemy, myEscape, desiredDist;
	
	private int dir = 1, latDir = 1, accel, surfID, desDir, secondSurfID, eShots, eHits;
	
	private boolean roundEnded = false, useFlat = false;
	
	//WAVE DATA
	private final Point2D lastCenter, shotCenter;
	private double lastAbsAngle, shotDistance, shotVel, shotNearWall, shotRevWall;
	private int lastDir = 1, shotDir;
	
	//SURF DATA
	private double wallSpace, revWallSpace, lastWallSpace, lastRevWallSpace;
	private int lastAccel, 
		shotAccel, distSeg, velSeg, nearWallSeg, revNearWallSeg,
			lastDSeg, lastVSeg, lastWSeg, lastRWSeg,
			lastLastDSeg, lastLastVSeg, lastLastWSeg, lastLastRWSeg;
	
	public MoveSim(final Self captain, final Enemy e) {
		this.self = captain;
		paths = (List<FutureImage>[]) new List[2*HALF_PATH_COUNT];
		for(int i = 0; i < paths.length; i ++) {
			paths[i] = new ArrayList<>();
		}
		secondPaths = new ArrayList<>();
		this.enemy = e;
		this._waves = new ArrayList<>();
		this.lastCenter = new Point2D.Double(captain.getX(),captain.getY());
		this.target = new Point2D.Double(captain.getX(), captain.getY());
		this.shotCenter = new Point2D.Double(captain.getX(), captain.getY());
		this.arena = new RoundRectangle2D.Double
			(45d,45d,Redacted.PLAY_WIDTH-90d,
			Redacted.PLAY_HEIGHT-90d, 95d, 125d
		);
		this.center = new Point2D.Double(captain.getX(),captain.getY());
		
		this.vcs = new float[3][5][8][3][2][BINS];
		this.offsetMap = new HashMap<>();
		this.neighbors = new float[][]{};
		
		this.desiredDist = 275d;
	}
	
	public void updateStatus(final StatusEvent ev) {
		if(roundEnded && self.getTime() < 30) {
			cleanup();
			roundEnded = false;
		}
		final Iterator<Wave> waverator = _waves.iterator();
		while(waverator.hasNext()) {
			final Wave w = waverator.next();
			if(w.contains(self.getX(),self.getY(),self.getTime())) {
				waverator.remove();
			}
			else if(w.breaks(center,self.getTime()) && w.isActive()) {
				w.deactivate();
				w.storeWave(center, w.angBotWidth(new Point2D.Double(self.getX(),self.getY())), 0.618f, 1.5f);
				//final float[] key = w.key();
				//key[key.length-1] += 0.001f;
				//offsetMap.put(key, w.getFactor(center));
			}
		}
		


		this.shotDir = lastDir;
		this.shotDistance = lastDistance;
		this.shotAccel = lastAccel;
		this.shotVel = lastVelocity;
		this.shotNearWall = lastWallSpace;
		this.shotRevWall = lastRevWallSpace;
		this.shotCenter.setLocation(lastCenter);
		
		this.lastDir = dir;
		lastAccel = accel;
		lastCenter.setLocation(center);
		this.lastDistance = distanceToEnemy;
		this.lastAbsAngle = Utils.normalRelativeAngle(enemy.absBearingTo() + Math.PI);
		this.lastVelocity = velocity;
		this.lastDir = dir;
		this.lastWallSpace = wallSpace;
		this.lastRevWallSpace = revWallSpace;	
		
		accel = Double.compare(Math.abs(self.getVelocity()),Math.abs(lastVelocity));
				
		center.setLocation(self.getX(), self.getY());
		this.distanceToEnemy = enemy.distance();
		this.velocity = self.getVelocity();

		dir = Math.abs(velocity) < 0.07d ?
				dir : (int) Math.signum(velocity);		
		latDir = dir * enemy.sideOfUs();

		setSegmentation();
	}	
	
	public Action sim(final List<Bullet> _shaders) {
	
		//if(roundEnded && self.getTime() < 30) {
	//		cleanup();
	//		roundEnded = false;
	//	}
		
		
		
		if(enemy.shot()) {
			final Point2D shotPos;
			_waves.add(new Wave(true, delayedKey(), shotPos = enemy.exposeShotPos(), Math.atan2(shotCenter.getX()-shotPos.getX(),shotCenter.getY()-shotPos.getY()),
				Rules.getBulletSpeed(enemy.shotPower()), self.getTime()-1,
					shotDir*(enemy.bearing()<0?-1:1),

			vcs[shotAccel+1][lastLastVSeg][lastLastDSeg][lastLastWSeg][lastLastRWSeg]

				)
			);
			eShots ++;
		} else if(enemy.shotPower() > enemy.energy()) {
			final Point2D shotPos;
			_waves.add(new Wave(false, delayedKey(), shotPos = enemy.exposeShotPos(), Math.atan2(shotCenter.getX()-shotPos.getX(),shotCenter.getY()-shotPos.getY()),
				Rules.getBulletSpeed(enemy.shotPower()), self.getTime()-1,
					shotDir*(enemy.bearing()<0?-1:1),

			vcs[shotAccel+1][lastLastVSeg][lastLastDSeg][lastLastWSeg][lastLastRWSeg]

				)
			);
		}
	
		/*dir = Math.abs(self.getVelocity()) <= 0.5d ?
				dir : (int) Math.signum(self.getVelocity());
		
		lastAccel = accel;
		accel = Double.compare(Math.abs(self.getVelocity()),Math.abs(lastVelocity));
				
		latDir = dir * enemy.sideOfUs();
		
		lastCenter.setLocation(center);
		center.setLocation(self.getX(), self.getY());
		
		this.lastDistance = distanceToEnemy;
		this.distanceToEnemy = enemy.distance();
		
		setSegmentation();*/
		
		
		getSurfWave();
		
		final Action toTake;
		if(surfWave != null) {
			//clearPaths();
			//carvePaths();
			//findSafest();
			toTake = driveTo();
		} else {
			toTake = emptySurf();
		}

		return toTake;
		
	}
	
	private void setSegmentation() {
		double eGoing = self.getHeadingRadians();
		final double wallDistLat = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (Redacted.PLAY_WIDTH-center.getX()) / (Math.cos((Math.PI/2d - eGoing)))
			: center.getX() / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirt = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(Redacted.PLAY_HEIGHT-center.getY()) / (Math.cos(eGoing)) : center.getY() / (Math.cos(Math.PI - eGoing));
		wallSpace = Math.max(0,Math.min(1,(float)(Math.min(wallDistLat, wallDistVirt) / Math.max(Redacted.PLAY_WIDTH, Redacted.PLAY_HEIGHT))));
		eGoing += Math.PI;
		eGoing = Utils.normalRelativeAngle(eGoing);
		final double wallDistLatRev = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (Redacted.PLAY_WIDTH-center.getX()) / (Math.cos((Math.PI/2d - eGoing)))
			: center.getX() / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirtRev = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(Redacted.PLAY_HEIGHT-center.getY()) / (Math.cos(eGoing)) : center.getY() / (Math.cos(Math.PI-Utils.normalAbsoluteAngle(eGoing)));
		revWallSpace = Math.max(0, Math.min(1,(float)(Math.min(wallDistLatRev, wallDistVirtRev) / Math.max(Redacted.PLAY_WIDTH, Redacted.PLAY_HEIGHT))));	

		lastLastDSeg = lastDSeg;
		lastLastVSeg = lastVSeg;
		lastLastWSeg = lastWSeg;
		lastLastRWSeg = lastRWSeg;

		lastDSeg = distSeg;
		lastVSeg = velSeg;
		lastWSeg = nearWallSeg;
		lastRWSeg = revNearWallSeg;

		nearWallSeg = Math.min((int)Math.floor(wallSpace*7d), 2);
		revNearWallSeg = Math.min((int)Math.floor(revWallSpace*9d), 1);
		distSeg = (int)Math.floor(this.distanceToEnemy/Redacted.MAX_DIST * 8d);
		velSeg = (int)((Math.abs(velocity)+1))/2;
		
			//System.out.println("nearWall: " + wallSpace);
			//System.out.println("revWall: " + revWallSpace);
			//System.out.println();
	}
	
	private Action emptySurf() {

		double desiredHeading = Utils.normalAbsoluteAngle(enemy.absBearingTo()-0.5d*Math.PI*enemy.sideOfUs());
		final Point2D head = Rut.project(center, self.getHeadingRadians(), 150*dir);
		final Point2D tail = Rut.project(center, self.getHeadingRadians(), -150*dir);
		
		double dDir = dir, distHead, distTail;
		if((distHead=head.distance(enemy.exposePos()))-(distTail=tail.distance(enemy.exposePos())) > 120d || ((distHead < distTail) && !arena.contains(head))) {
			dDir *= -1d;
			desiredHeading = Utils.normalRelativeAngle(desiredHeading + Math.PI);
		}
		double turnAmount = Utils.normalRelativeAngle(Rut.wallSmooth(arena, center, desiredHeading, dDir, dDir*enemy.sideOfUs())
				- self.getHeadingRadians());
		
		target.setLocation(center);

		return new Action(100d*dDir, turnAmount);
	}
	
	
	private void findSafest() {
		if(paths[0].isEmpty() || paths[1].isEmpty())
			target.setLocation(center);
		else {
			useFlat = useFlat || (eHits > 6 && (float)eHits/Math.max(1f,eShots) > 0.1f);
			float safest = Float.MAX_VALUE;
			//TAKE NOTE ASSHOLE
			
/////////////////////////////////////
			final float[] dangerMap = surfWave.backend();
/////////////////////////////////////
			float maxFlattenerDanger = 0f, maxKNNDanger = 0f;
			Point2D newTarget = center;
			int targetBin = 0, targetPath=-1;
			for(int p = paths.length-1; p >= 0; p --) {
				final List<FutureImage> path = paths[p];
				for(final FutureImage future : path) {
					final double optAngle,  stDev = (surfWave.angBotWidth(future)) *1.05d;
					final int bin = (int)(Math.max(-1d,Math.min(Utils.normalRelativeAngle((optAngle=surfWave.angleFromSource(future)) - surfWave.baseAmp())
							/ surfWave.mEA()*surfWave.latDir(),1d))*(dangerMap.length-1)/2d + (dangerMap.length-1)/2);
					//final int halfBinWidth = (int)Math.ceil(surfWave.angBotWidth(future)*0.5d/surfWave.mEA());
					float flattenDanger = useFlat ? dangerMap[bin] : 0f;
					//for(int b = Math.max(0,bin-halfBinWidth); b <= Math.min(BINS-1, bin+halfBinWidth); b ++) {
					//	flattenDanger += dangerMap[b];
					//}
					maxFlattenerDanger = Math.max(maxFlattenerDanger, flattenDanger);// = (flattenDanger / (2*halfBinWidth+1)));
					float knnDanger = 0f;
					for(final float[] neighbor : neighbors) {
						final double x = (Utils.normalRelativeAngle(optAngle-surfWave.baseAmp()-offsetMap.get(neighbor) * surfWave.mEA() * surfWave.latDir()) / stDev);
						knnDanger += Math.exp(-0.5d*x*x)// / surfWave.distance(future)
; // / Math.max(1f, Rut.manKeyDist(surfWave.key(), neighbor));
					}
					maxKNNDanger = Math.max(maxKNNDanger, knnDanger);
					if(flattenDanger + knnDanger >= safest) {
						continue;
					}
					float secondSafest = Float.MAX_VALUE
							, secondAverage = 0f, secondDangerousest = 0f, secondDanger = 0f;
					final List<Point2D> _secondPositions = new ArrayList<>();
					// if(!offsetMap.isEmpty()) {
					if(this.secondWave != null && !offsetMap.isEmpty()) {
						final long timeOfImpact = surfWave.timeTillImpact(future, self.getTime()) + self.getTime();// + surf.wave.timeTillImpact(ime()) + self.getTime();
						final Tank toy = new Tank(future.getX(),future.getY(),future.velocity(),future.heading(), p);///Utils.normalAbsoluteAngle(secondWave.angleToSource(future) - 0.5d*enemy.sideOfUs()*(Math.PI - 1.618d*Rules.MAX_TURN_RATE_RADIANS*p)), p);
						_secondPositions.addAll(estimatePathsFrom(toy, secondWave, _secondPositions, timeOfImpact));
						for(final Point2D future2 : _secondPositions) {
							final double optAngle2;
							final int bin2 = (int)(Math.max(-1d,Math.min(1d,Utils.normalRelativeAngle((optAngle2=secondWave.angleFromSource(future2))-secondWave.baseAmp())/secondWave.mEA()*secondWave.latDir()))*(dangerMap.length-1)/2d+(dangerMap.length-1)/2);
							float danger2 = 
							0;//		useFlat ? secondWave.backend()[bin2] : 0f;
							final double stDev2 = secondWave.angBotWidth(future2) *1.35d;
							for(final float[] neigh : secondNeighbors) {
								if(neigh == null) continue;
								final double x = Utils.normalRelativeAngle(optAngle2-secondWave.baseAmp()-offsetMap.get(neigh) *secondWave.mEA() * secondWave.latDir()) / stDev2;
								danger2 += Math.exp(-0.5d*x*x); // secondWave.distance(future2)
// / Math.max(1f, Rut.manKeyDist(secondWave.key(), neigh));
							}
							secondSafest = Math.min(secondSafest, danger2);
							secondDangerousest = Math.max(secondDangerousest, danger2);
							secondAverage += danger2;
						}
						secondAverage /= Math.max(_secondPositions.size(), 1f);
						//secondSafest = Math.min(secondSafest, secondAverage);
					} else {
						secondSafest = 0f;
					} //} else {secondSafest = 0f;}
					//System.out.println("SS: " + secondSafest);
					//secondDanger = ((Math.random()<0.5d ? secondSafest : secondDangerousest) +(Math.random() < 0.5d ? secondAverage : secondSafest))/2f;
//					secondDanger = secondSafest;
	//				final float secondSafest = 0f;
		secondDanger = secondSafest;//1f/((2f*secondDangerousest-secondAverage)/2f+2f/secondSafest+0.0001f);//secondDangerousest;//(secondSafest + secondAverage) / 2f;// + secondDangerousest)/3f;// + secondSafest) / 2f;
		
					if(/*0.618d*/(flattenDanger + knnDanger + secondDanger) < safest) {//*Math.max(1f,flattenDanger+knnDanger)*Math.max(1f, secondSafest) < safest) {
						secondPaths.clear();
						secondPaths.addAll(_secondPositions);
						safest = flattenDanger + knnDanger + secondDanger;
						//safest = (flattenDanger + knnDanger + secondSafest)*Math.max(1f,flattenDanger+knnDanger)*Math.max(1f, secondSafest);
						//target.setLocation(future);
						targetBin = bin;
						targetPath = p;
						newTarget = future;
						desDir = p*2-1;
					}
				}
			}
			target.setLocation(newTarget);		
		//System.out.println("MFD: " + maxFlattenerDanger);
		//System.out.println("MKD: " + maxKNNDanger);
		//System.out.println();
			
		}
			
		
	}
	
	private Action driveTo() {
//		return Action.create(center, target, self.getHeadingRadians());
		//I don't want to drive straight there, I want to follow the projection

		myEscape = enemy.distance() < desiredDist ? 3d*Rules.MAX_TURN_RATE_RADIANS*dir : -1.618d*Rules.MAX_TURN_RATE_RADIANS*dir;

		int goDir = 1;
		final double angleToTarget = Math.atan2(target.getX()-center.getX(), target.getY()-center.getY()),
			bearingToTarget = Utils.normalRelativeAngle(angleToTarget - self.getHeadingRadians());
		if(Math.abs(bearingToTarget)>0.5d*Math.PI) {
			goDir *= -1;
			//bearingToTarget = Utils.normalRelativeAngle(bearingtoTarget+Math.PI);
		}

		final double desiredHeading = Rut.wallSmooth(arena, center,
				Utils.normalAbsoluteAngle(surfWave.angleToSource(center) - 0.5d*enemy.sideOfUs()*(Math.PI + myEscape)),
				goDir, goDir*enemy.sideOfUs());				

		double turnAmount = Utils.normalRelativeAngle(desiredHeading - self.getHeadingRadians());
		
		//if(Math.abs(turnAmount) > 0.5d*Math.PI) {
		//	turnAmount = Utils.normalRelativeAngle(turnAmount + Math.PI);
		//	goDir *= -1;
		//}

		return new Action(goDir*target.distance(center), turnAmount);
	}
	
	private void getSurfWave() {
		this.surfWave = null;
		this.secondWave = null;
		int minImpact = Integer.MAX_VALUE;
		int secondImpact = Integer.MAX_VALUE;
		for(final Wave wave : _waves) {
			final int timeTillImpact = wave.timeTillImpact(self.getX(),self.getY(),self.getTime());
			if(wave.isReal() && wave.isActive() && timeTillImpact < minImpact && timeTillImpact > 2) {
				this.surfWave = wave;
				minImpact = timeTillImpact;
			}
			else if(wave.isReal() && wave.isActive() && timeTillImpact < secondImpact && timeTillImpact > 2) {
				this.secondWave = wave;
				secondImpact = timeTillImpact;
			}
		}
		final int tempCode;
		if(surfWave!=null && (tempCode=this.surfWave.hashCode())!=this.surfID) {
			this.surfID = tempCode;
			if(this.secondWave != null)
				this.secondSurfID = secondWave.hashCode();
			if(!offsetMap.isEmpty()){
				setNeighbors();
				clearPaths();
				carvePaths();
				findSafest();
			}
		}
		else if(this.secondWave != null && (this.secondWave.hashCode()!=this.secondSurfID)) {
			if(!offsetMap.isEmpty()) {
				secondSurfID = secondWave.hashCode();
				setNeighbors();
				clearPaths();
				carvePaths();
				findSafest();
			}
		}
	}
	
	private void setNeighbors() {
		this.neighbors = Rut.findKNN(NUM_NEIGHB0RS//(int)Math.max(NUM_NEIGHB0RS, Math.log(offsetMap.size()))
					, surfWave.key(),
			offsetMap.keySet().toArray(new float[0][]));
		if(this.secondWave != null)
			this.secondNeighbors = Rut.findKNN(NUM_NEIGHB0RS//(int)Math.max(NUM_NEIGHB0RS, Math.log(offsetMap.size()))
					, secondWave.key(),
				offsetMap.keySet().toArray(new float[0][]));
	}
	
	private void clearPaths() {
		for(int i = 0; i < paths.length; i ++) {
			paths[i].clear();
		}
	}
	
	private void carvePaths() {
		final Tank toy = new Tank(self.getX(), self.getY(),
			self.getVelocity(), self.getHeadingRadians(), dir);
		final List<Point2D> _spacers = new ArrayList<>();
		final double myX = self.getX(), myY = self.getY(), myVel = self.getVelocity(), myHead = self.getHeadingRadians();
		for(int d = dir, i = 0; i < 2; d -= 2*dir, i ++) {
			//Geenerate spaced points
			double escape = enemy.distance() < desiredDist ? 3d*Rules.MAX_TURN_RATE_RADIANS*toy.dir() : -1.618d*Rules.MAX_TURN_RATE_RADIANS*toy.dir();
			double desiredAngle = Rut.wallSmooth(arena, toy.exposePos(),
				Utils.normalAbsoluteAngle(surfWave.angleToSource(toy.exposePos()) - 0.5d*enemy.sideOfUs()*(Math.PI + escape)),
				d, d*enemy.sideOfUs());
			for(int f = 0; !surfWave.breaks(toy.exposePos(), self.getTime()+f) && f < 205; f ++) {
				toy.step(new Action(d*100d, Utils.normalRelativeAngle(desiredAngle-toy.heading())));
				desiredAngle = Rut.wallSmooth(arena, toy.exposePos(),
			//		toy.heading(),
		Utils.normalAbsoluteAngle(surfWave.angleToSource(toy.exposePos()) - 0.5d*enemy.sideOfUs()*(Math.PI + escape)),
				d, d*enemy.sideOfUs());
				_spacers.add(toy.popPos());
				 escape = enemy.exposePos().distance(toy.exposePos()) < desiredDist ? 3d*Rules.MAX_TURN_RATE_RADIANS*toy.dir() : -1.618d*Rules.MAX_TURN_RATE_RADIANS*toy.dir();
				//paths[(d+1)/2].add(toy.popPos());
			}
		//	paths[(d+1)/2].add(toy.popPos());
			//Solve for stops
			final double endHeading = toy.heading();
			final int endDir = toy.dir();
			final double endVel = toy.velocity();
			for(int p = 0; p < _spacers.size()/*-1*/; p ++) {
				final Point2D possible = _spacers.get(p);
				//if(possible.distance(center) > 18d) {
					toy.reset(myX, myY, myVel, myHead,dir);
					final FutureImage real = endOfPathTo(possible, toy);
					if(real != null)
				
						paths[(d+1)/2].add(real);
//					else
//						paths[(d+1)/2].add(new FutureImage(possible, endVel, endHeading));
				//}
				//else {
				//	paths[(d+1)/2].add(new FutureImage(possible, toy.velocity(), toy.heading()));
				//}
			}
			if(d == dir)
				paths[(d+1)/2].add(new FutureImage(_spacers.get(_spacers.size()-1), endVel, endHeading));
			toy.reset(myX, myY, myVel, myHead,dir);
			_spacers.clear();
		}
	}
	
	private List<Point2D> estimatePathsFrom(final Tank toy, final Wave surf, final List<Point2D> _spacers, final long future) {
		final double startHead = toy.heading(), startVel = toy.velocity(),
			startX = toy.getX(), startY = toy.getY();
		final int  startDir = toy.dir();
		for(int d = dir, i = 0; i < 2; d -= 2*dir, i ++) {
			//Geenerate spaced points
			double escape = enemy.distance() < desiredDist ? 3d*Rules.MAX_TURN_RATE_RADIANS*toy.dir() : -1.618d*Rules.MAX_TURN_RATE_RADIANS*toy.dir();
			double desiredAngle = Rut.wallSmooth(arena, toy.exposePos(),
				Utils.normalAbsoluteAngle(surf.angleToSource(toy.exposePos()) - 0.5d*enemy.sideOfUs()*(Math.PI + escape)),
				d, d*enemy.sideOfUs());
			for(int f = 1; !surf.contacts(toy.exposePos(), future+f) && f < 205; f ++) {
				toy.step(new Action(d*100d, Utils.normalRelativeAngle(desiredAngle-toy.heading())));
				desiredAngle = Rut.wallSmooth(arena, toy.exposePos(),
					Utils.normalAbsoluteAngle(surf.angleToSource(toy.exposePos()) - 0.5d*enemy.sideOfUs()*(Math.PI + escape)),
					d, d*enemy.sideOfUs());
				_spacers.add(toy.popPos());
				
				 escape = enemy.exposePos().distance(toy.exposePos()) < desiredDist ? 3d*Rules.MAX_TURN_RATE_RADIANS*toy.dir() : -1.618d*Rules.MAX_TURN_RATE_RADIANS*toy.dir();
				//paths[(d+1)/2].add(toy.popPos());
			}
			//Solve for stops
			/*for(final Point2D possible : _spacers) {
				toy.reset(startX, startY, startVel, startHead, startDir);
				final Point2D real = endOfPathTo(possible, toy);
				if(real != null)
					paths[(d+1)/2].add(real);
			}*/
		//	if(!_spacers.isEmpty())
		//		_spacers.remove(_spacers.size()-1);
			toy.reset(startX, startY, startVel, startHead, startDir);
			//_spacers.clear();
		}
		return _spacers;
	}
	
	private FutureImage endOfPathTo(final Point2D loc, final Tank toy) {
		//final Tank toy = new Tank(self.getX(), self.getY(),
		//	self.getVelocity(), self.getHeadingRadians(), dir);
		
		int counter = 1;
		boolean hit = false;
		while(toy.exposePos().distance(loc) >= .5d && counter < 200 && !(hit=surfWave.contacts(toy.exposePos(), self.getTime()+counter))) {
			toy.step(Action.create(toy.exposePos(), loc, toy.heading()));
			counter ++;
		}
		//System.out.println("Hit: " + hit);
		return hit ? null :
		//	return
				 new FutureImage(toy.popPos(), toy.velocity(), toy.heading());//toy.popPos();
	}
	
	@Deprecated
	private void fillPaths() {
		final double turnPerPath = Math.PI/(double)(HALF_PATH_COUNT-1);
		final Tank toy = new Tank(self.getX(),self.getY(),
			self.getVelocity(), self.getHeadingRadians(),dir);
		for(int d = 1, h = 0; d > -2; d -= 2, h ++) {
			for(int i = 0; i < HALF_PATH_COUNT; i ++) {
				double distRemaining = 150d*d,
					turnRemaining = -0.5d*Math.PI + i * turnPerPath;
				int counter = 0;
				do {
					final Action a = new Action(distRemaining, turnRemaining);
					turnRemaining = toy.step(a);
					distRemaining -= d * Math.abs(toy.velocity());
					paths[i+h*HALF_PATH_COUNT].add(new FutureImage(toy.exposePos(), toy.velocity(), toy.heading()));//toy.popPos());
					counter ++;
				} while(Math.abs(distRemaining) > 0 && d * distRemaining > 0 && counter < 100);
				toy.reset(
					self.getX(),self.getY(),
					self.getVelocity(),self.getHeadingRadians(),dir
				);
			}
		}
	}
	
	private float[] delayedKey() {
		final float STATIC_MULT = 1f;
		return new float[]{self.getRoundNum(), self.getTime()-1,STATIC_MULT*(shotAccel+1)/2f, 
			STATIC_MULT*((float)Math.abs(shotVel))/8f, 
			STATIC_MULT*(float)(shotDistance/Redacted.MAX_DIST),
			STATIC_MULT*Math.min(0.33333f,(float)shotNearWall)*3f,
			STATIC_MULT*Math.min(0.33333f,(float)shotRevWall)*3f,
		//	STATIC_MULT*(secondWave == null ? 0f : 1f)
			0f
		};
	}
	
	public void handleHit(final Bullet b, final boolean bhb) {
		final Wave incident = getCause(b, bhb);
		if(incident == null) {
			System.out.println("Dropped wave");
			return;
		}
		if(!bhb) eHits ++;
		//incident.storeWave(b.getHeadingRadians(), incident.angBotWidth(center));
		offsetMap.put(incident.key(), incident.getFactor(b.getHeadingRadians()));
		this.surfID = 0;
		getSurfWave();
	}
	
	private Wave getCause(final Bullet b, final boolean bhb) {
		for(final Wave w : _waves) {
			if(w.isReal() && w.matches(b, self.getTime(), bhb)) {
				return w;
			}
		}
		return null;
	}

	public void paint(final Graphics2D g) {
		g.setColor(Color.CYAN);
		for(int i = 0; i < paths.length; i ++) {
			for(final Point2D fut : paths[i]) {
				g.fill(
					new Ellipse2D.Double(
						fut.getX()-1,fut.getY()-1,2,2
					)
				);
			}
		}
		g.setColor(Color.ORANGE.brighter());
		for(final Point2D fut : secondPaths) {
			g.fill(
				new Ellipse2D.Double(
					fut.getX()-1,fut.getY()-1,2,2
				)
			);
		}
		g.setColor(Color.RED.brighter().brighter().brighter().brighter());
		//for(final Wave w : _waves) {
		//	if(w.isReal()) w.paint(g, self.getTime());
		//}
		g.setColor(new Color(231,201,169));
		g.draw(arena);
		g.setColor(new Color(231,201,169));
		g.draw(new Rectangle2D.Double(center.getX()-18d,center.getY()-18d,36d,36d));
		g.setColor(new Color(214,108,201));
		g.draw(new Rectangle2D.Double(target.getX()-18d,target.getY()-18d,36d,36d));
	}
	
	public void endRound() {
		roundEnded = true;
		
		//DON'T FORGET THIS
		//cleanup();
	}
	
	private void cleanup() {
		_waves.clear();
		enemy.setHealth(100d);
		target.setLocation(center);
		surfWave = null;
		secondWave = null;
		this.surfID = 0;
		this.secondSurfID = 0;
		for(int i = 0; i < paths.length; i ++) {
			paths[i].clear();
		}
		secondPaths.clear();
		if(offsetMap.size() > MAX_MAP_SIZE) {
			final List<float[]> oldKeys = offsetMap.keySet().stream().sorted((a,b)->
				Float.compare(a[0],b[0])==0?Float.compare(a[1],b[1]):Float.compare(a[0],b[0])
			).limit(offsetMap.size()-MAX_MAP_SIZE).collect(Collectors.toList());
			for(final float[] oldKey : oldKeys) {
				offsetMap.remove(oldKey);
			}
			System.out.println("Dumped : " + oldKeys.size() + " surfs");
		}
	}

}

class FutureImage extends Point2D.Double {
	private final double velocity, heading;
	FutureImage(final Point2D inPos, final double inVel, final double inHead) {
		super(inPos.getX(), inPos.getY());
		this.velocity = inVel;
		this.heading = inHead;
	}
	public double heading() {
		return heading;
	}
	public double velocity() {
		return velocity;
	}
}

