package damij;
import robocode.*;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Stream;
import java.awt.geom.Point2D;
import robocode.util.Utils;
import java.awt.geom.Rectangle2D;
import java.awt.Graphics2D;
import java.util.Iterator;
import java.awt.Color;
import java.util.Arrays;

/**
 * Dodger by damij
 */
public class Dodger
{

	public static final int BINS = 47, VEL_SEGS = 5, DIST_SEGS = 5, ACCEL_SEGS = 3, NEAR_WALL_SEGS = 5;
	
	public static final double V_W = 0.9D, D_W = 0.9D, NW_W = .9D;

	private final AdvancedRobot body;
	
	private final Point2D.Double pos, scannedTarget;
	
	private final List<Wave> _eWaves;
	
	private final List<Point2D.Double> _pos;
	
	private final double[][][][][] visitCountStats;
	
	private final double[][][] weights;
	
	private final double MAX_DIST;
	
	private Point2D.Double head, tail;
	
	private Rectangle2D.Double playField;
	
	private double eEnergy, lastEEnergy, absBearing, lastEVel, eVel,
		speed, lastSpeed, heading;
	
	private long scanTime, lastScanTime;
	
	private int velSeg, distSeg, accelSeg, nwSeg, eSide, latDir;
	
	private boolean hasShot, surfed;
	
	public Dodger(final AdvancedRobot self) {
		this.body = self;
		this._eWaves = new ArrayList<>();
		this._pos = new ArrayList<>(20);
		eEnergy = 100d;
		pos = new Point2D.Double(body.getX(),body.getY());
		head = damij.Utils.project(pos,8d,body.getHeadingRadians());
		tail = damij.Utils.project(pos,-8d,body.getHeadingRadians());
		scannedTarget = new Point2D.Double(0,0);
		playField = new Rectangle2D.Double(18,18,
			body.getBattleFieldWidth()-36,
			body.getBattleFieldHeight() - 36
		);
		MAX_DIST = Math.sqrt(
				body.getBattleFieldWidth()*body.getBattleFieldWidth()
				+ body.getBattleFieldHeight()*body.getBattleFieldHeight()
		);
		visitCountStats = new double[VEL_SEGS][DIST_SEGS][ACCEL_SEGS][NEAR_WALL_SEGS][BINS];
		weights = new double[VEL_SEGS][DIST_SEGS][NEAR_WALL_SEGS];
		
		for(int v = 0; v < VEL_SEGS; v ++) {
			for(int d = 0; d < DIST_SEGS; d ++) {
				for(int nW = 0; nW < NEAR_WALL_SEGS; nW ++) {
						final double q = Math.pow(V_W, (v)),
							w = Math.pow(D_W, (d)),
						r = Math.pow(NW_W, (nW));
						weights[v][d][nW] = q*w*r;
				}
					
			}
		}
		
	}
	
	public void handleHitByBullet(final HitByBulletEvent e) {
		final Point2D.Double bPos = new Point2D.Double(e.getBullet().getX(), e.getBullet().getY());
		//for(final Wave w : _eWaves) {
		//	System.out.println(w.power() + " :: " + w.timeTillHit(bPos));
		//}
		final Wave cause = _eWaves.stream().filter(w->Math.abs(w.power()-e.getBullet().getPower())<.1d).sorted((w1,w2)->(
					w1.timeTillHit(bPos)< w2.timeTillHit(bPos) ? -1 : 1)).findFirst().orElse(null);
		if(cause == null) {
			System.out.println("Dropped a wave");
		}
		else {
			storeWave(cause, e.getBullet());
			_eWaves.remove(cause);
		}
	}
	
	public void handleBulletHitBullet(final BulletHitBulletEvent e) {
		final Point2D.Double bPos = new Point2D.Double(e.getHitBullet().getX(), e.getHitBullet().getY());
		final Wave cause = _eWaves.stream().filter(w->Math.abs(w.power()-e.getHitBullet().getPower())<.1d&&w.timeTillHit(bPos)<2).sorted((w1,w2)->(
					w1.timeTillHit(bPos)< w2.timeTillHit(bPos) ? -1 : 1)).findFirst().orElse(null);
		if(cause == null) {
			System.out.println("Dropped a bonus wave");
		}
		else {
			storeWave(cause, e.getHitBullet());
			_eWaves.remove(cause);
		}
	}
	
	public void update(final ScannedRobotEvent e) {
	
		lastScanTime = scanTime;
		scanTime = body.getTime();
		
		lastEEnergy = eEnergy;
		eEnergy = e.getEnergy();
		lastEVel = eVel;
		eVel = e.getVelocity();
		
		eSide = sign(e.getBearing());
		
		absBearing = Utils.normalRelativeAngle(
			body.getHeadingRadians() + e.getBearingRadians()
		);
		hasShot = false;
		if(Math.abs(lastEVel)-Math.abs(eVel)<= 2 &&
				Math.abs(lastEEnergy - eEnergy) <= 3 && lastEEnergy - eEnergy > 0) {
			hasShot = true;
			_eWaves.add(
				new Wave(
					scannedTarget, Math.atan2(pos.x-scannedTarget.x, pos.y-scannedTarget.y),
					lastEEnergy - eEnergy, latDir, velSeg, distSeg, accelSeg, nwSeg, body.getRoundNum()
				)
			);
			_eWaves.get(_eWaves.size()-1).progress(1);
		}

		final double vDiv = 8d/(VEL_SEGS-1d);
		
		velSeg = (int)Math.ceil(Math.abs(body.getVelocity()/vDiv));
		distSeg = (int)Math.floor(e.getDistance()/MAX_DIST*DIST_SEGS);
		accelSeg = Math.abs(lastSpeed-speed) < 0.5 ? 0 :
			Math.abs(lastSpeed)<Math.abs(speed) ? 1 : 2;
		nwSeg = (int)Math.max(
			wallSmooth(pos, heading, eSide)[1],
			wallSmooth(pos, heading+Math.PI, -eSide)[1]
		);

		pos.x = body.getX();
		pos.y = body.getY();
		
		//System.out.println(nwSeg);	

		
		scannedTarget.x = pos.x + Math.sin(absBearing) * e.getDistance();
		scannedTarget.y = pos.y + Math.cos(absBearing) * e.getDistance();
		
		latDir = Math.abs(speed-lastSpeed) < 0.1d ?sign(lastSpeed)*eSide : sign(speed)*eSide;
		
	}
	
	public void cycle() {
		head = damij.Utils.project(pos,8d,body.getHeadingRadians());
		tail = damij.Utils.project(pos,-8d,body.getHeadingRadians());
		
		lastSpeed = body.getVelocity()==0? lastSpeed : speed;
		speed = body.getVelocity();
		
		heading = body.getHeadingRadians();
		
		final Iterator<Wave> wavorator
			= _eWaves.iterator();
		while(wavorator.hasNext()) {
			final Wave w = wavorator.next();
			w.progress(1);
			if(w.hasBroken(new Point2D.Double(body.getX(),body.getY()))||w.roundNumber()!=body.getRoundNum()) {
				wavorator.remove();
			}
		}

		surf();
		/*final Wave toDodge = getMostSurfableWave();
		if(toDodge == null) {
			emptySurf();
		} else {
			surf(toDodge);
		}*/
	}
	 
	private void surf() {
		surfed = false;
		_eWaves.stream()./*filter(w->w.timeTillHit(pos)>=0.0d).*/sorted(
			(w1,w2) -> w1.timeTillHit(pos) < w2.timeTillHit(pos) ? -1 : 1
		).findFirst().ifPresent(w->surf(w));
		if(!surfed) {
			emptySurf();
		}
	}
	
	private Wave getMostSurfableWave() {
		return _eWaves.stream().sorted(
			(w1,w2) -> w1.timeTillHit(pos) < w2.timeTillHit(pos) ? -1 : 1
		).findFirst().orElse(null);
	}
	
	private void emptySurf() {		
		final double desiredForward
			= absBearing - Math.PI/2 * eSide;
			
		final double[] desiredGoForwardTurn = wallSmooth(pos, desiredForward, eSide);
		final double desiredGoForward = desiredGoForwardTurn[0];
		final double[] desiredGoBackTurn = wallSmooth(pos, desiredForward + Math.PI, -eSide);
		final double desiredGoBack = desiredGoBackTurn[0];
		final double desiredGoDir = damij.Utils.project(pos, 1d, desiredGoForward).distance(pos)
								>= damij.Utils.project(pos,1d, desiredGoBack).distance(pos) ?
										1 : -1;
		
		final double absVel = Math.abs(body.getVelocity());
		body.setAhead(desiredGoDir * ((absVel>6?16:(absVel>4?8:(absVel>=3?4:2)))+absVel));
		body.setTurnRightRadians(
			Utils.normalRelativeAngle(
				(desiredGoDir == 1 ? desiredGoForward : desiredGoBack + Math.PI)
					- body.getHeadingRadians()
			)
		);
		//System.out.println(desiredGoDir + " :: " + body.getVelocity());
		//0 = forward, -1 = backward
		/*final int desiredGoDir = head.distance(scannedTarget)>=tail.distance(scannedTarget)?0:-1;
		final double desiredGo = wallSmooth(pos, desiredForward + Math.PI*desiredGoDir, desiredGoDir*eSide);
		
		final double turnAmount = Utils.normalRelativeAngle(desiredGo - body.getHeadingRadians());
		final int goDir = Math.abs(turnAmount) > Math.PI/2d ? -1 : 1;
		body.setTurnRightRadians(
			Utils.normalRelativeAngle(turnAmount + (goDir == -1 ? Math.PI : 0))
		);
		body.setAhead(goDir*120d);*/
	}
	
	private void storeWave(final Wave w, final Bullet bullet) {
		final int endBin = (int)Math.max(Math.min(w.getOffset(new Point2D.Double(bullet.getX(),bullet.getY()))*BINS/2 + BINS/2, BINS-1),0);
		final int aheadBin = getBin(w.getOffset(head));
		final int backBin = getBin(w.getOffset(tail));
		final Point2D.Double source = new Point2D.Double(w.sourceX(),w.sourceY());
		final double botWidth = //Math.abs(Math.atan(8d/source.distance(pos)) / w.mEA()) * BINS/2d;
				Math.max(1d,Math.abs(aheadBin-backBin));
		
		final double sigma = botWidth / 2d;
		final double div = sigma*Math.sqrt(2d*Math.PI);
		
		//double total = 0d;
		for(int b = 0; b < BINS; b ++) {
			visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b]
				+= (Math.exp(-.5d*Math.pow((b-endBin)/sigma,2d)) - .5d) / div;
			//total += visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b];
		}
		
		//for(int b = 0; b < BINS; b ++) {
		//	visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b] /= total;
		//}
		
		//System.out.println(Arrays.toString(visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()]));

		//for(int b = Math.min(aheadBin,backBin)-1; b < Math.max(aheadBin,backBin)+1; b ++) {
		//	visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b] ++;
		//}
		//final int endBin = (int)Math.max(Math.min(w.getOffset(position)*BINS/2 + BINS/2, BINS-1),0);
		/*for(int b = 0; b < BINS; b ++) {
			visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b]
				+= 1 / (1d + Math.abs(b-endBin));
		}*/
		//System.out.println(w.mEA());
		//System.out.println("Hit at : " + (endBin-BINS/2));
		//System.out.println(Arrays.toString(visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()]));
	}
	
	private void surf(final Wave toDodge) {
		surfed = true;
		final double desiredForward = Math.atan2(toDodge.sourceX()-pos.x, toDodge.sourceY()-pos.y)/*absBearing*/ - Math.PI/2 * eSide;		

		_pos.clear();
		final int currentBin = getCurrentBin(toDodge);
		final int aheadBin = predictCollision(toDodge);
		final int backBin = predictBackCollision(toDodge);
		final int leastDangerBin = getLeastDangerBin(toDodge, backBin, aheadBin);
		
		final Point2D.Double source = 
					new Point2D.Double(toDodge.sourceX(),toDodge.sourceY());
		//System.out.println(aheadBin + " -> " + backBin);
		final Point2D.Double desiredSpot = damij.Utils.project(source,
					source.distance(pos), 
					toDodge.heading() + getFactor(leastDangerBin) * toDodge.mEA()
				);

		//what if we use more real go to

		double desiredGoDir = 0;
		/*if(latDir*eSide>0) {
			if(currentBin < leastDangerBin) {
				desiredGoDir = toDodge.curl() * eSide;
			} else {//(currentBin > leastDangerBin) {
				desiredGoDir = -toDodge.curl() * eSide;
			}
		}
		else {*/
			if(currentBin <= leastDangerBin) {
				desiredGoDir = toDodge.curl() * eSide;
			} else if (currentBin > leastDangerBin) {
				desiredGoDir = -toDodge.curl() * eSide;
			}
		//}
		
		final Point2D.Double currentBinPos = damij.Utils.project(source,
					source.distance(pos), 
					toDodge.heading() + getFactor(currentBin) * toDodge.mEA()
				);
		
		final double distToTravel = desiredSpot.distance(currentBinPos);
		
		final double desiredGoForward = wallSmooth(pos,desiredForward,eSide)[0];
		final double desiredGoBack = wallSmooth(pos, desiredForward+Math.PI, -eSide)[0];
		
		final double absVel = Math.abs(body.getVelocity());
		//System.out.println(desiredSpot.distance(pos));
		body.setAhead(desiredGoDir * 2d*distToTravel);//Math.max((distToTravel),/*((absVel>6?16:(absVel>4?8:(absVel>=3?4:2)))+*/absVel));
		body.setTurnRightRadians(Utils.normalRelativeAngle((desiredGoDir==1?desiredGoForward:desiredGoBack+Math.PI) - body.getHeadingRadians()));
	}
	
	private int predictCollision(final Wave w) {
		Point2D.Double nextPos = new Point2D.Double(pos.x,pos.y);
		
		double nextVelocity = speed, nextHeading = heading;
		int futures = 0, nextLatDir = latDir;
		while(!w.willBreak(nextPos,futures)) {
			futures ++;
			nextHeading = wallSmooth(nextPos, nextHeading, eSide)[0];
			nextVelocity += nextLatDir*eSide > 0 ? 1 : 2;
			nextVelocity = Math.max(-8d, Math.min(nextVelocity, 8d));
			nextLatDir = sign(nextVelocity)*eSide;
			nextPos = damij.Utils.project(nextPos, nextVelocity, nextHeading);
			_pos.add(nextPos);
		}

		return getBin(w.getOffset(nextPos));
	}
	
	private int predictBackCollision(final Wave w) {
		Point2D.Double nextPos = new Point2D.Double(pos.x,pos.y);
		
		double nextVelocity = speed, nextHeading = Utils.normalAbsoluteAngle(heading + Math.PI);
		int futures= 0, nextLatDir = latDir;
		while(!w.willBreak(nextPos,futures)) {
			futures ++;
			nextHeading = wallSmooth(nextPos, nextHeading, -eSide)[0];
			nextVelocity += nextLatDir*eSide < 0 ? -1 : -2;
			nextVelocity = Math.max(-8d,Math.min(nextVelocity,8d));
			nextLatDir = sign(nextVelocity)*eSide;
			nextPos = damij.Utils.project(nextPos, -nextVelocity, nextHeading);
			_pos.add(nextPos);
		}
		
		return getBin(w.getOffset(nextPos));
	}
	
	private int getCurrentBin(final Wave toDodge) {
		final double offset = toDodge.getOffset(pos);
		return (int)(offset*BINS/2+BINS/2);
	}
	
	private int getLeastDangerBin(final Wave w, final int minBin, final int maxBin) {
		int minDangerBin = -1;
		double minDanger = Double.POSITIVE_INFINITY;
		for(int b = Math.min(minBin,maxBin); b < Math.min(BINS,Math.max(minBin,maxBin)); b ++) {
			double localDanger = 0d;
			for(int v = 0; v < VEL_SEGS; v ++) {
				for(int d = 0; d < DIST_SEGS; d ++) {
					for(int nw = 0; nw < NEAR_WALL_SEGS; nw ++) {
						localDanger += visitCountStats[v][d][w.accelSeg()][nw][b]
							* weights[Math.abs(v-w.velSeg())][Math.abs(d-w.distSeg())][Math.abs(nw-w.nWallSeg())];
					}
				}
			}
			if(localDanger <= minDanger) {
				minDanger = localDanger;
				minDangerBin = b;
			}
		}
		return minDangerBin;
	}
	
	private int getBin(final double offset) {
		return (int)(offset*BINS/2 + BINS/2);
	}
	
	private double getFactor(final int bin) {
		return ((double)bin - BINS/2D) / (BINS/2D);
	}
	
	private int sign(final double value) {
		return value >= 0d ? 1 : -1;
	}
	
	public void cleanup() {
		eEnergy = 100d;
		lastEEnergy = eEnergy;
		lastScanTime = 0;
		_eWaves.clear();
	}
	
	public void onPaint(final Graphics2D g2) {
		g2.setColor(Color.RED);
		for(final Wave w : _eWaves) {
			w.paint(g2);
		}
		g2.setColor(Color.ORANGE.brighter().brighter().brighter());
		for(final Point2D.Double pos : _pos) {
			g2.fillOval((int)pos.x,(int)pos.y,3,3);
		}
	}

	public double[] wallSmooth(final Point2D.Double pos, double theta, final int dir) {
		int times = 0;
		while(times < 100 && !playField.contains(damij.Utils.project(pos, 120d, theta))) {
			theta += dir * 0.25d / 31.4159d;
			times ++;
		}
		return new double[] {theta, Math.min(times/100d*NEAR_WALL_SEGS, NEAR_WALL_SEGS-1)};
	}

}
