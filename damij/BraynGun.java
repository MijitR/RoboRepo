//acc : 046031001
//rout : 275979076
package damij;
import robocode.*;
import java.util.List;
import java.util.ArrayList;
import robocode.util.Utils;
import java.awt.geom.Point2D;
import java.util.Iterator;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.geom.Rectangle2D;

/**
 * BraynGun - a class by Damij
 */
public class BraynGun
{

	public static final int BINS = 47, VEL_SEGS = 5, DIST_SEGS = 5, ACCEL_SEGS = 3, NEAR_WALL_SEGS = 3;
	
	public static final double V_W = 0.5D, D_W = 0.5D, A_W = .0D, NW_W = 0.5D;
	
	private static final double[][][][][] visitCountStats = new double[VEL_SEGS][DIST_SEGS][ACCEL_SEGS][NEAR_WALL_SEGS][BINS];
	
	private final Rectangle2D.Double playGround;
	
	private final double MAX_DIST;

	private final AdvancedRobot body;
	
	private final List<Wave> _waves;
	
	private final Point2D.Double pos, scannedTarget;
	
	private final double[][][] weights;
	
	private double bPower, mEA, lastESpeed, eSpeed, eHeading;

	private long timeToShoot, scanTime, lastScanTime;
	
	private int eLatDirection;
	
	public BraynGun(final AdvancedRobot body) {
		this.body = body;
		_waves = new ArrayList<>();
		pos = new Point2D.Double(0,0);
		scannedTarget = new Point2D.Double(0,0);
		//visitCountStats = new double[VEL_SEGS][DIST_SEGS][ACCEL_SEGS][NEAR_WALL_SEGS][BINS];
		weights = new double[VEL_SEGS][DIST_SEGS][NEAR_WALL_SEGS];
		
		MAX_DIST = Math.sqrt(
			body.getBattleFieldWidth()*body.getBattleFieldWidth()
			+
			body.getBattleFieldHeight()*body.getBattleFieldHeight()
		);
		
		playGround = new Rectangle2D.Double(0,0,body.getBattleFieldWidth(),body.getBattleFieldHeight());
		
		
		for(int v = 0; v < VEL_SEGS; v ++) {
			for(int d = 0; d < DIST_SEGS; d ++) {
				for(int nW = 0; nW < NEAR_WALL_SEGS; nW ++) {
						final double q = Math.pow(V_W, (v)),
							w = Math.pow(D_W, (d)),
						r = Math.pow(NW_W, (nW));
						weights[v][d][nW] = q*w*r;
						/*weights[v][d][nW] =
							Math.pow(( q
						* w
							* r) , 1d/3d);/// Math.sqrt(q*q+w*w+r*r);
						;*/
						//System.out.println("v -> q " + v + " " + q);
						//weights[v][d][nW] = (q*VEL_SEGS + w*DIST_SEGS + r*NEAR_WALL_SEGS)/Math.sqrt((VEL_SEGS*VEL_SEGS+DIST_SEGS*DIST_SEGS+NEAR_WALL_SEGS*NEAR_WALL_SEGS));
						//System.out.println("Weights: " + weights[v][d][nW]);
				}
					
			}
		}
		
		eLatDirection = 1;
	}
	
	void update(final ScannedRobotEvent e) {
	
		eHeading = e.getHeadingRadians();
	
		double bPowerTemp = Math.max((1d-e.getDistance()/MAX_DIST) * 3d,1.4d);
	
		if(e.getEnergy() < /*8.2*/12) {
			if(e.getEnergy() <= 4) {
				bPowerTemp = e.getEnergy() / 4d;
			} else {
				bPowerTemp = (e.getEnergy() + 2d) / 6d;
			}
		}
		
		bPower = Math.max(.1d,bPowerTemp);///*Math.min(bPowerTemp, */body.getEnergy());
		bPower = bPower >= body.getEnergy() ? 0 : bPower;

		final double bSpeed;

		mEA = Math.asin(8d/(bSpeed=(20d-3d*bPower)));	

		pos.x = body.getX();
		pos.y = body.getY();
	
		final double absBearing =
			Utils.normalRelativeAngle(body.getHeadingRadians() + e.getBearingRadians());		

		scannedTarget.x = pos.x + Math.sin(absBearing) * e.getDistance();
		scannedTarget.y = pos.y + Math.cos(absBearing) * e.getDistance();
		
		final double themToUs =
			Utils.normalAbsoluteAngle(absBearing-Math.PI);
			
		final boolean isWePort
			= Utils.normalRelativeAngle(e.getHeadingRadians() - themToUs) >= 0;

		eLatDirection = e.getVelocity()==0 ?
			eLatDirection : sign(e.getVelocity()) * (isWePort?-1:1);
			
		lastESpeed = eSpeed;
		eSpeed = e.getVelocity();
		
		lastScanTime = scanTime;
		scanTime = body.getTime();

		final Iterator<Wave> waveorator
			= _waves.iterator();
		while(waveorator.hasNext()) {
			final Wave w = waveorator.next();
			w.progress((int)(scanTime - lastScanTime));
			if(w.hasBroken(scannedTarget)||w.roundNumber()!=body.getRoundNum()) {
				storeWave(w);
				waveorator.remove();
			}
		}
		
		Point2D.Double predFront = damij.Utils.project(scannedTarget, Math.max(125d,e.getDistance()/bSpeed*8d), e.getHeadingRadians());
		for(int i = 0; i < 7; i ++) {
			predFront = damij.Utils.project(scannedTarget, predFront.distance(pos)/bSpeed*8d, e.getHeadingRadians());
		}
		
		Point2D.Double predBack = damij.Utils.project(scannedTarget, Math.max(125d,e.getDistance()/bSpeed*8d), e.getHeadingRadians() + Math.PI);
		for(int i = 0; i < 7; i ++) {
			predBack = damij.Utils.project(scannedTarget, predBack.distance(pos)/bSpeed*8d, e.getHeadingRadians() + Math.PI);
		}
		//final Point2D.Double predFront = damij.Utils.project(scannedTarget, 125d, e.getHeadingRadians());
		//final Point2D.Double predBack = damij.Utils.project(scannedTarget, -125d, e.getHeadingRadians());
		final boolean predFrontInBounds = playGround.contains(
				predFront
			);
		final boolean predBackInBounds = playGround.contains(predBack);

		Point2D.Double pred = sign(eSpeed)*sign(Math.abs(eSpeed)-Math.abs(lastESpeed))>0?predFront:predBack;

		if(!predFrontInBounds) {
			pred = predFront;
		}
		else if(!predBackInBounds) {
			pred = predBack;
		}
		

		final boolean predInBounds;

		if(playGround.contains(pred)) {
			if(sign(eSpeed)>0) {
				predInBounds = predBackInBounds;
				pred = predBack;
			} else {
				predInBounds = predFrontInBounds;
				pred = predFront;
			}	
		} else {
			predInBounds = false;
		}
		//playGround.contains(pred) || (predFrontInBounds && predBackInBounds);
		
		final double width = body.getBattleFieldWidth();
		final double height = body.getBattleFieldHeight();
		
		final double outAmount;
		
		if(!predInBounds) {
		
			//final double outX = (Math.min(pred.x, -pred.x+width)) / 180d;
			//final double outY = (Math.min(pred.y, -pred.y+height)) / 180d;
			
			final double outX = pred.x < 0 ? -pred.x : Math.max((pred.x-width), 0d);
				//(Math.min(pred.x, -pred.x+width)) / pred.distance(pos);
			final double outY = pred.y < 0 ? -pred.y : Math.max((pred.y-height), 0d);
				//(Math.min(pred.y, -pred.y+height)) / pred.distance(pos);
			
			outAmount = //Math.abs(predBack.distance(pos)-predFront.distance(pos))/e.getDistance()*Math.sqrt((outX*outX+outY*outY))/bSpeed;
				//1d/bSpeed;//
				Math.sqrt((outX*outX+outY*outY)) / (pred.distance(scannedTarget));// : outY / Math.sqrt((outX*outX+outY*outY));
			
		}
		
		else {
			outAmount = 0d;
		}
		//System.out.println(outAmount);

		//final double outX = Math.abs(Math.min(body.getBattleFieldWidth()-pred.x, pred.x)) / pred.distance(pos);
		//final double outY = Math.abs(Math.min(body.getBattleFieldHeight()-pred.y, pred.y))) / pred.distance(pos);
		
		final double vDiv = 8d/(VEL_SEGS-1d);
		
		final int velSeg = (int)(Math.ceil(Math.abs(e.getVelocity()/vDiv)));
		final int distSeg = (int)Math.floor(e.getDistance()/MAX_DIST*DIST_SEGS);
		final int accelSeg = Math.abs(lastESpeed)==Math.abs(eSpeed) ? 0 :
					Math.abs(lastESpeed)<Math.abs(eSpeed) ? 1 : 2;
		final int nwSeg = predInBounds ? 0 : (int) Math.min(NEAR_WALL_SEGS-1, Math.ceil(outAmount* NEAR_WALL_SEGS));
		//final int nwSeg = predInBounds ? 0 : (int)Math.min(NEAR_WALL_SEGS-1, Math.ceil((outX+outY) * NEAR_WALL_SEGS));

		//System.out.println(nwSeg);

		if(timeToShoot == body.getTime() && bPower > 0) {
			final Bullet b = body.setFireBullet(bPower);
			if(b!=null) {
				_waves.add(
					new Wave(
						pos, absBearing, bPower, eLatDirection, velSeg, distSeg, accelSeg, nwSeg, body.getRoundNum()
					)
				);
			} else {
				//System.out.println("null bullet");
			}
		}
		
		/*
		 * 
		 * 
		 * 
		 * 
		 * 
		 */
		
		//if(body.getGunTurnRemaining() == 0
		//		) {//&& body.getEnergy() >= bPower+.1d) {
		//	boolean shot = false;
		if(body.getGunTurnRemaining()==0) {
		//		shot = true;
			timeToShoot = body.getTime() + 1;
		//	}
		}
		
		final double angleToShoot 
			= determineAimingAngle(absBearing, velSeg, distSeg, accelSeg, nwSeg);
			
		final double gunOffset =
			Utils.normalRelativeAngle(
				body.getGunHeadingRadians()
				- angleToShoot
			);

		//if(!shot)
		body.setTurnGunLeftRadians(gunOffset);
		
	}
	
	private double determineAimingAngle(final double absBearing, final int velSeg, final int distSeg, final int accelSeg, final int nwSeg) {
		double maxBin = BINS/2;
		double maxVisit = Double.NEGATIVE_INFINITY;

		for(int b = 0; b < BINS; b ++) {
			double weightSum = 0;
			for(int v = 0; v < VEL_SEGS; v ++) {
				for(int d = 0; d < DIST_SEGS; d ++) {
					for(int nW = 0; nW < NEAR_WALL_SEGS; nW ++) {
						
								weightSum += visitCountStats[v][d][accelSeg][nW][b]
									* weights[Math.abs(v-velSeg)][Math.abs(d-distSeg)][Math.abs(nwSeg-nW)];
						/*	* Math.pow(V_W, Math.abs(v-velSeg))
							* Math.pow(D_W, Math.abs(d-distSeg))
							* Math.pow(NW_W, Math.abs(nwSeg - nW));*/
							;
					}
						
				}
			}
			if(weightSum >= maxVisit) {
				maxVisit = weightSum;
				maxBin = b;
			}
		}

		//System.out.println("Aiming at bin: " + maxBin);
		/*for(int b = 0; b < BINS; b ++) {
			if(visitCountStats[velSeg][distSeg][accelSeg][b] >= maxVisit) {
				maxVisit = visitCountStats[velSeg][distSeg][accelSeg][b];
				maxBin = b;
			}
		}*/
		//System.out.println(maxBin);
		return Utils.normalAbsoluteAngle(
					(double)(maxBin - BINS/2) / (double)(BINS/2) * mEA * eLatDirection + absBearing
				);
	}
	
	public void onPaint(final Graphics2D g2) {
		g2.setColor(Color.MAGENTA);
		for(final Wave w : _waves) {
			w.paint(g2);
		}
	}
	
	private void storeWave(final Wave w) {
		final double offset = w.getOffset(scannedTarget);
		final int headBin = getBin(w.getOffset(damij.Utils.project(scannedTarget,8d,eHeading)));
		final int buttBin = getBin(w.getOffset(damij.Utils.project(scannedTarget,-8d,eHeading)));
		final double botWidth = Math.max(1d,Math.abs(headBin-buttBin));
		
		final double sigma = botWidth / 2d;
		final double div = sigma*Math.sqrt(2d*Math.PI);
		//System.out.println("Caught them at offset: " + offset);
		final int resultBin = (int)(offset * BINS/2 + BINS/2);
		double total = 0d;
		for(int b = 0; b < BINS; b ++) {
			visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b] += 
				//1d * (Math.exp(-.5d*Math.pow((b-resultBin)/sigma,2d)) - 0.5d) / div;
				.1d / (1d + (double)Math.abs(b - resultBin));
			//System.out.println((1d / (1d + (double)Math.abs(b - resultBin))) + " - " + visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b]);
			total += visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b];
		}
		//System.out.println("Total: " + total);
		for(int b = 0; b < BINS; b ++) {
		visitCountStats[w.velSeg()][w.distSeg()][w.accelSeg()][w.nWallSeg()][b] /= total;
		}
	}
	
	private int sign(final double value) {
		return value >= 0d ? 1 : -1;
	}
	
	public static final int getBin(final double offset) {
		return (int)(offset*BINS/2 + BINS/2);
	}
	
	public void cleanup() {
		scanTime = 0;
		lastScanTime = 0;
		_waves.clear();
	}

}
