package DM;
import robocode.*;
import robocode.util.Utils;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.stream.*;
import java.util.List;
import java.util.ArrayList;


// At one time, I had it working

/**
 * DNA - a robot by Damij
 */
public class DNA extends AdvancedRobot
{

	private static Rectangle2D playField;
	static double playHeight, playWidth, maxDist;

	Contrabase base;
	Enemy enemy;
	
	private final int PIXELATION = 3
, POOL_SIZE = 2, SHRINK_TARGET = 8;
	
	final List<Strand> rnaPool = new ArrayList<>();
	
	final List<Point2D> eFutures = new ArrayList<>();

	private final Point2D myCenter = new Point2D.Double(0d,0d);
	
	private double velocity, heading, health;
	/**
	 * run: DNA's default behavior
	 */
	public void run() {
	
		if(playField == null)
			playField = new Rectangle2D.Double(18,18,super.getBattleFieldWidth()-36,super.getBattleFieldHeight()-36);
			
		playHeight = super.getBattleFieldHeight();
		playWidth = super.getBattleFieldWidth();
		
		maxDist = Math.sqrt(playHeight*playHeight + playWidth*playWidth);

		// Initialization of the robot should be put here

		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:

		// setColors(Color.red,Color.blue,Color.green); // body,gun,radar
		
		super.setAdjustRadarForGunTurn(true);
		super.setAdjustRadarForRobotTurn(true);
		super.setAdjustGunForRobotTurn(true);

		// Robot main loop
		while(true) {
			// Replace the next 4 lines with any behavior you would like
			setSelf();
			doGun();
			execute();
		}
	}
	

	public void onStatus(final StatusEvent e) {
		setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(final ScannedRobotEvent e) {
		// Replace the next line with any behavior you would like
		final double absBearing = Utils.normalRelativeAngle
			(getHeadingRadians() + e.getBearingRadians()),
				radarOffset = Utils.normalRelativeAngle(
					getRadarHeadingRadians() - absBearing
				);
		setTurnRadarLeftRadians(1.7d * radarOffset);
		
		Enemy.update(e, absBearing, myCenter);
		
		final double bSpeed = Rules.getBulletSpeed(bPower);

		long timeDelta;
		
		while(rnaPool.size() >= POOL_SIZE /*&& bPower < super.getEnergy()*/) {
			/*final List<Strand> usedEntries =
				rnaPool.stream().sorted((a, b) -> Long.compare(a.age(),b.age())).limit(POOL_SIZE-SHRINK_TARGET)
			.collect(Collectors.toList());
			for(final Strand entry : usedEntries) {
				
				rnaPool.remove(entry);
			}*/
			rnaPool.remove(0);
		}

		final Strand currStrand = new Strand((float)(Utils.normalRelativeAngle(Enemy.heading-absBearing)), (float)Enemy.velocity, (timeDelta=(long)Math.ceil(Enemy.dist/bSpeed)), (float)Enemy.wallSpace, (float)Enemy.revWallSpace);
		if(super.getTime() % PIXELATION == 0 && bPower < super.getEnergy()) {
			rnaPool.add(currStrand);
			for(int i = 0; i < rnaPool.size()-1; i ++) {
				final Strand expectation = Contrabase.process(rnaPool.get(i));
				final Strand influence = Contrabase.trainSample(rnaPool.get(i+1));//new Strand((float)Enemy.heading,(float)Enemy.velocity,rnaBase.time()).sub(currStrand));
			}
		}
		
		lastAbsBearing = Enemy.absBearing;
		
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// Replace the next line with any behavior you would like
		
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
		
	}
	
	void setSelf() {
		this.velocity = super.getVelocity();
		this.heading = super.getHeadingRadians();
		this.health = super.getEnergy();
		myCenter.setLocation(super.getX(), super.getY());
	}
	

	long timeToShoot = 0L, currTime; double bPower = 3d, lastAbsBearing;
	void doGun() {
		currTime = super.getTime();
		
		if(timeToShoot == super.getTime() && super.getGunTurnRemaining() == 0) {
			final Bullet bob = setFireBullet(bPower);
			if(bob!=null) {
				
			}
		}
		
		
		double bSpeed = Rules.getBulletSpeed(bPower);

		long timeDelta;
		//final Strand expectation = Contrabase.process(currStrand);
		
		//System.out.println("Given: " + currStrand);
		//System.out.println("ExpectedDeltas: " + expectation);
		
		Point2D expectedSpot = new Point2D.Double(Enemy.x, Enemy.y), enemyPos = new Point2D.Double(Enemy.x, Enemy.y);
		
		//final Strand rates = expectation.scale(1f/(float)Math.max(1,timeDelta));
		//System.out.println("Per tick: " + rates);
		
		eFutures.clear();
		double wallSpaceLoc = Enemy.wallSpace;
		double revWallSpaceLoc = Enemy.revWallSpace;
		Strand expectation = new Strand((float)(Utils.normalRelativeAngle(Enemy.heading-Enemy.absBearing)), (float)Enemy.velocity, (timeDelta=(long)Math.ceil(Enemy.dist/bSpeed)), (float)Enemy.wallSpace, (float)Enemy.revWallSpace);
		double runningVelocity = Enemy.velocity;
		final Mass enemy = new Mass(Enemy.velocity, Enemy.heading);
		int i = 1;
		do {
		
			double eGoing;
		//for(int i = 1; expectedSpot.distance(myCenter) - bSpeed * i * PIXELATION >= 0 && i <= 50f*timeDelta; i ++) {
			expectation = Contrabase.process(expectation.setTime((long)Math.ceil(expectedSpot.distance(myCenter)/bSpeed)).setWallSpaceAndRev(wallSpaceLoc, revWallSpaceLoc));
			enemy.update(expectation.velocity(), eGoing = expectation.heading() + Math.atan2(expectedSpot.getX()-myCenter.getX(),expectedSpot.getY()-myCenter.getY()));//(Enemy.absBearing + Math.PI/2d * sign(Utils.normalRelativeAngle(Utils.normalRelativeAngle(lastAbsBearing-Enemy.absBearing))));
			expectedSpot = fix(project(expectedSpot, enemy.velocity(), enemy.heading()));
			eFutures.add(expectedSpot);
			//timeDelta = (long)(expectedSpot.distance(myCenter)/bSpeed);
			
			final double x = Enemy.x, y = Enemy.y;

			final double wallDistLat = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (DNA.playWidth-x) / (Math.cos((Math.PI/2d - eGoing)))
				: x / (Math.cos((3d*Math.PI/2d - eGoing)));
			final double wallDistVirt = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
					(DNA.playHeight-y) / (Math.cos(eGoing)) : y / (Math.cos(Math.PI - eGoing));
		
			wallSpaceLoc = Math.max(0,Math.min(1,(float)(Math.min(wallDistLat, wallDistVirt) / DNA.maxDist)));
		
			eGoing += Math.PI;
			eGoing = Utils.normalRelativeAngle(eGoing);
		
			final double wallDistLatRev = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (DNA.playHeight-x) / (Math.cos((Math.PI/2d - eGoing)))
				: x / (Math.cos((3d*Math.PI/2d - eGoing)));
			final double wallDistVirtRev = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
					(DNA.playHeight-y) / (Math.cos(eGoing)) : y / (Math.cos(Math.PI-Utils.normalAbsoluteAngle(eGoing)));
			
			revWallSpaceLoc = Math.max(0, Math.min(1,(float)(Math.min(wallDistLatRev, wallDistVirtRev)/ DNA.maxDist)));			

			i ++;
		} while (expectedSpot.distance(myCenter) - bSpeed * (i-1)  >= 0 && i <= 50f*timeDelta);
		
		final double aimAngle = Math.atan2(expectedSpot.getX()-myCenter.getX(),expectedSpot.getY()-myCenter.getY());
		
		final double gunOffset = Utils.normalRelativeAngle(
			super.getGunHeadingRadians() -
				aimAngle
		);
		super.setTurnGunLeftRadians(gunOffset);		
		
		timeToShoot  = super.getTime() + 1;
	}
	
	Point2D project(final Point2D source, final double dist, final double angle) {
		return new Point2D.Double(source.getX() + dist * Math.sin(angle), source.getY() + dist * Math.cos(angle));
	}
	
	Point2D fix(final Point2D spot) {
		spot.setLocation(limit(18,spot.getX(),super.getBattleFieldWidth()-36), limit(18,spot.getY(),super.getBattleFieldHeight()-36));
		return spot;
	}
	
	static double limit(final double low, final double mid, final double high) {
		return Math.max(low, Math.min(mid,high));
	}
	
	static int sign(final double value) {
		return value >= 0 ? 1 : -1;
	}
	
	@Override
	public void onPaint(final Graphics2D g) {
		g.setColor(Color.PINK);
		for(final Point2D spot : eFutures) {
			g.draw(new Ellipse2D.Double(spot.getX()-1,spot.getY()-1,2,2));
		}
	}
}

class Enemy {
	static double dist, heading, absBearing, x, y, velocity, lastVelocity, accel, lastHealth, health, wallSpace = 0, revWallSpace = 0;
	static void update(final ScannedRobotEvent e, final double absBearing, Point2D source) {
		dist = e.getDistance();
		heading = e.getHeadingRadians();
		Enemy.absBearing = absBearing;
		x = Math.sin(absBearing) * dist + source.getX();
		y = Math.cos(absBearing) * dist + source.getY();
		velocity = e.getVelocity();
		accel = (Math.abs(velocity) - Math.abs(lastVelocity));
		lastVelocity = velocity;
		health = e.getEnergy();
		final boolean shot = lastHealth-health > 0 && lastHealth-health <= 3d;
		lastHealth= health;
		
		double eGoing = heading;

		final double wallDistLat = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (DNA.playWidth-x) / (Math.cos((Math.PI/2d - eGoing)))
			: x / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirt = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(DNA.playHeight-y) / (Math.cos(eGoing)) : y / (Math.cos(Math.PI - eGoing));
		
		wallSpace = Math.max(0,Math.min(1,(float)(Math.min(wallDistLat, wallDistVirt) / DNA.maxDist)));
		
		eGoing += Math.PI;
		eGoing = Utils.normalRelativeAngle(eGoing);
		
		final double wallDistLatRev = Utils.normalAbsoluteAngle(eGoing) < Math.PI ? (DNA.playHeight-x) / (Math.cos((Math.PI/2d - eGoing)))
			: x / (Math.cos((3d*Math.PI/2d - eGoing)));
		final double wallDistVirtRev = Math.abs(Utils.normalRelativeAngle(eGoing)) < Math.PI / 2d  ?
				(DNA.playHeight-y) / (Math.cos(eGoing)) : y / (Math.cos(Math.PI-Utils.normalAbsoluteAngle(eGoing)));
		
		revWallSpace = Math.max(0, Math.min(1,(float)(Math.min(wallDistLatRev, wallDistVirtRev)/ DNA.maxDist)));
	}
}

class Strand {
	private final long time;
	private final float heading, velocity, wallSpace, revWallSpace;
	private int age;
	Strand(final float heading, final float velocity, final long time, final float wallSpace, final float revWallSpace) {
		this.heading = heading;
		this.velocity = velocity;
		this.time = time;
		this.wallSpace = wallSpace;
		this.revWallSpace = revWallSpace;
	}
	
	public float heading() {
		return heading;
	}
	public float velocity() {
		return velocity;
	}
	public Strand scale(final float scalar) {
		return new Strand(scalar*heading, scalar*velocity, time, wallSpace, revWallSpace);
	}
	public Strand fuse_by_mul(final Strand s) {
		return new Strand(heading*s.heading(),velocity*s.velocity(),s.time(), s.wallSpace, s.revWallSpace);
	}
	public Strand sub(final Strand s) {
		return new Strand((float)Utils.normalRelativeAngle(heading - s.heading()), velocity() - s.velocity(),time-s.time(), wallSpace-s.wallSpace, revWallSpace-s.revWallSpace);
	}
	public Strand mul_out() {
		return new Strand((float) Math.acos(heading), (velocity * 8f), time, wallSpace, revWallSpace);
	}
	public Strand simplify() {
		return new Strand((float)Math.cos(heading), velocity / 8f, time, wallSpace, revWallSpace);
	}
	public Strand setWallSpaceAndRev(final double wS, final double rWS) {
		return new Strand(heading, velocity, time, (float)wS, (float)rWS);
	}
	public float sum() {
		return heading + velocity;
	}
	public void olden() {
		this.age++;
	}
	public int age() {
		return age;
	}
	public long time() {
		return time;
	}
	public float wallSpace() {
		return wallSpace;
	}
	public float revWallSpace() {
		return revWallSpace;
	}
	public Strand setTime(final long newTime) {
		return new Strand(heading, velocity, newTime, wallSpace, revWallSpace);
	}
	public boolean breaks(final double dist, final long time) {
		return (time-this.time) * velocity >= dist;
	}
	@Override
	public String toString() {
		return "(Time: " + time + " Head: " + heading + " Vel: " + velocity + ")";
	}
}

class Mass {
	private double velocity, heading;
	Mass(final double velocity, final double heading) {
		this.velocity = velocity;
		this.heading = heading;
	}
	Mass update(double newVelocity, final double newHeading) {
		int dir = DNA.sign(velocity);
		double turnAmount = Utils.normalRelativeAngle(newHeading - heading);
		if(Math.abs(turnAmount) > Math.PI/2d) {
			newVelocity *= -1;
			turnAmount = Utils.normalRelativeAngle(Math.PI+turnAmount);
		}
		final double velDiffer = Math.abs(newVelocity)-Math.abs(velocity);
		velocity += dir * newVelocity >= 0 ? dir : -2d*dir;
		velocity = DNA.limit(-8d, velocity, 8d);
		final double maxTurn = Rules.getTurnRateRadians(Math.abs(velocity));
		heading += DNA.limit(-maxTurn, turnAmount, maxTurn);
		return this;
	}
	double velocity() {
		return this.velocity;
	}
	double heading() { 
		return this.heading;
	}
}

class Contrabase {
	
	static final int H_DEX = 0, V_DEX = 1;
	static final float eta = 0.001618f;	
	static final float[][] state = new float[][]{new float[]{(float)Math.random()-0.5f,(float)Math.random()-0.5f},
			new float[]{(float)Math.random()-0.5f, (float)Math.random()-0.5f},
					new float[]{(float)Math.random()-0.5f,(float)Math.random()-0.5f}}, deltaState = new float[3][2];
	
	static Strand lastInput, lastOutput;
	
	static float bias1 = (float)Math.random()-0.5f, deltaBias1, timeWeight1 = (float)Math.random()-0.5f, deltaTimeWeight1,
		bias2=(float)Math.random()-0.5f, deltaBias2, timeWeight2=(float)Math.random()-0.5f, deltaTimeWeight2,
		bias3=(float)Math.random()-0.5f, deltaBias3, timeWeight3=(float)Math.random()-0.5f, deltaTimeWeight3,
		revWallWeight1=(float)Math.random()-0.5f, deltaRevWallWeight1, revWallWeight2=(float)Math.random()-0.5f, deltaRevWallWeight2,
		dwrtIH, dwrtWV, dwrtWH, dwrtIV, gradHead, gradVel, dwrtWT, dwrtIT, dwrtRWH, dwrtRWV, gradID;
	
	static Strand process(final Strand input) {
		input.olden();
		lastInput = new Strand(input.heading(),input.velocity(),input.time(),input.wallSpace(),input.revWallSpace());
		lastOutput = new Strand(
			headingOperation(input),
			deltaVelocities(input),
			timeOps(input),
			input.wallSpace(),
			input.revWallSpace()
		);
		return lastOutput.mul_out();
	}
	
	static Strand trainSample(final Strand target) {
		
		Strand transfer = lastOutput.sub(target.simplify());
		return train(transfer);
		
	}
	
	static float headingOperation(final Strand input) {
		dwrtIH = state[0][H_DEX];
		dwrtWH = input.heading();
		dwrtRWH = input.revWallSpace();
		final float theta = state[0][H_DEX] * input.heading() + bias1 + state[0][V_DEX] * input.velocity() + timeWeight1 * (input.wallSpace()) + revWallWeight1*(input.revWallSpace());
		gradHead = -(float) Math.sin(theta);
		return (float) Math.cos(theta);
		//final float value = (float) Math.tanh(theta);
		//gradHead = 1f - value*value;
		//return value;
	}
	
	static float deltaVelocities(final Strand input) {
		dwrtIV = state[1][V_DEX];
		dwrtWV = input.velocity();
		dwrtRWV = input.revWallSpace();
		final float theta = state[1][V_DEX] * input.velocity() + bias2  + state[1][H_DEX] * input.heading() + timeWeight2 * (input.wallSpace()) + revWallWeight2*(input.revWallSpace());
			//state[V_DEX] * input.velocity() + bias * 0.33f;// + timeWeight * (currTime - input.time());
		gradVel = -(float) Math.sin(theta);
		return (float) Math.cos(theta);
		//final float value = (float) Math.tanh(theta);
		//gradVel = 1f - value * value;
		//return value;
	}
	
	static long timeOps(final Strand input) {
		dwrtWT = input.wallSpace();
		dwrtIT = timeWeight3;
		final float passVal = state[2][H_DEX] * input.heading() + state[2][V_DEX] * input.velocity() + bias3 + timeWeight3 * (input.time());
		/*gradID = 0.01f;
		if(passVal >= 0) {
			gradID = 1f;
		}
		return (long)(gradID * passVal);/*
		gradID = -(float)  Math.sin(theta);
		return (long) Math.cos(theta);*/
		gradID = 1f;
		return (long)passVal;
	}
	
	static Strand train(final Strand target) {
		final Strand err = lastOutput.sub(target.simplify());
		deltaBias1 = 0.100f * deltaBias1 + (1f-0.100f) * gradHead * err.heading();
		deltaBias2 = 0.100f * deltaBias2 + (1f-0.100f) * gradVel * err.velocity();
		deltaBias3 = 0.100f * deltaBias3 + (1f-0.100f) * gradID * err.time();
		
		deltaState[0][H_DEX] = 0.100f * deltaState[0][H_DEX] + (1f-0.100f) * gradHead * err.heading() * dwrtWH;
		deltaState[0][V_DEX] = 0.100f * deltaState[0][V_DEX] + (1f-0.100f) * gradHead * err.heading() * dwrtWV;
		deltaTimeWeight1 = 0.100f * deltaTimeWeight1 + (1f-0.100f) * gradHead * err.heading() * dwrtWT;
		deltaRevWallWeight1 = 0.100f * deltaRevWallWeight1 + (1f-0.100f) * gradHead * err.heading() * dwrtRWH;
		
		deltaState[1][H_DEX] = 0.100f * deltaState[1][H_DEX] + (1f-0.100f) * gradVel * err.velocity() * dwrtWH;
		deltaState[1][V_DEX] = 0.100f * deltaState[1][V_DEX] + (1f-0.100f) * gradVel * err.velocity() * dwrtWV;
		deltaTimeWeight2 = 0.100f * deltaTimeWeight2 + (1f-0.100f) * gradVel * err.velocity() * dwrtWT;
		deltaRevWallWeight2 = 0.100f * deltaRevWallWeight2 + (1f-0.100f) * gradVel * err.velocity() * dwrtRWV;
		
		deltaState[2][H_DEX] = 0.100f * deltaState[2][H_DEX] + (1f-0.100f) * gradID * err.time() * dwrtWH;
		deltaState[2][V_DEX] = 0.100f * deltaState[2][V_DEX] + (1f-0.100f) * gradID * err.time() * dwrtWV;
		deltaTimeWeight3 = 0.100f * deltaTimeWeight3 + (1f-0.100f) * gradID * err.time() *dwrtWT;
		
		bias1 -= eta * deltaBias1;
		bias2 -= eta * deltaBias2;
		bias3 -= eta * deltaBias3;
		
		state[0][H_DEX] -= eta * deltaState[0][H_DEX];
		state[0][V_DEX] -= eta * deltaState[0][V_DEX];
		timeWeight1 -= eta * deltaTimeWeight1;
		revWallWeight1 -= eta * deltaRevWallWeight1;
		
		state[1][H_DEX] -= eta * deltaState[1][H_DEX];
		state[1][V_DEX] -= eta * deltaState[1][V_DEX];
		timeWeight2 -= eta * deltaTimeWeight2;
		revWallWeight2 -= eta * deltaRevWallWeight2;
		
		state[2][H_DEX] -= eta * deltaState[2][H_DEX];
		state[2][V_DEX] -= eta * deltaState[2][V_DEX];
		timeWeight3 -= eta * deltaTimeWeight3;
		
		return new Strand(gradHead * err.heading(), gradVel * err.velocity(), (long)(gradID * err.time()), err.wallSpace(), err.revWallSpace());
	}

	/*static Strand train(final Strand err) {
		deltaBias = err.sum();
		deltaState[H_DEX] = dwrtWH * gradHead * err.heading();
		deltaState[V_DEX] = dwrtWV * gradVel * err.velocity();
		deltaTimeWeight = dwrtWT * (gradVel + gradHead);
		
		bias -= eta * deltaBias;
		timeWeight -= eta * deltaTimeWeight;
		state[H_DEX] -= eta * deltaState[H_DEX];
		state[V_DEX] -= eta * deltaState[V_DEX];
		
		return new Strand(state[H_DEX], state[V_DEX], err.time());
	}*/
	
}
