package rwh;
import rwh.util.Rut;
import robocode.*;
import robocode.util.Utils;
import java.util.List;
import java.util.ArrayList;
import java.util.Iterator;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.geom.Ellipse2D;
import java.util.HashMap;
import java.util.stream.Stream;
import java.util.stream.Collectors;
import java.awt.geom.Rectangle2D;

/**
 * MyClass - a class by Damij
 */
public class Weapon
{

	public static final double B_POWER_BASE = 2.0d,
		DISP_WIDTH = 190d, DISP_HEIGHT = 75d;

//ADAPTABLE NUM_NEIGHBORS INSTEAD
	public static final int BINS = 261, NUM_NEIGHBORS = 13, MAX_MAP_SIZE = 13500;

	private final AdvancedRobot body;
	
	private final Self self;
	
	private final Enemy enemy;
	
	private final List<Wave> _waves;
	
	private final HashMap<float[], Double> offsetMap;
	
	private final float[][][][][][][] vcs;
	
	private final float[][] paintDangers;
	
	private float[] currKey;
	
	private double bPower = 0.1d, mainGunAngle, antiSurfAngle;

	private long fireTime, currTime, mainGunUses, antiSurfUses;
	
	private int accelSeg, latVelSeg, wallSeg, distSeg, vertSeg, revWallSeg, shots, kNNHits, aSHits, hits;
	
	private double shotBearing;

	private int shotLatDir, shotAccel;
	
	private boolean useAntiSurf;

	public Weapon(final AdvancedRobot body, final Self self, final Enemy enemy) {
		this.body = body;
		this.self = self;
		this.enemy = enemy;
		this._waves = new ArrayList<>();
		this.vcs = new float[3][5][4][2][3][3][BINS];
		this.offsetMap = new HashMap<>();
		this.paintDangers = new float[2][BINS];
	}
	
	public Bullet update() {
	
		this.currTime = self.getTime();
		
		final Iterator<Wave> waverator = _waves.iterator();
		while(waverator.hasNext()) {
			final Wave w = waverator.next();
			//if(w.contacts(enemy.exposePos(), currTime)) {
			//	if(w.isActive())
			//		w.storeWave(enemy.exposePos());
				if(w.contains(enemy.exposePos(), currTime)) {
					waverator.remove();
					shots ++;
				}
				else if(w.contacts(enemy.exposePos(), currTime) && w.isActive()) {
					if(w.isReal()/* && w.key()[w.key().length-1] != 1*/) {
						final float[] key = w.key();
						//key[key.length-1] += 0.001f;
						offsetMap.put(key, w.getFactor(enemy.exposePos()));	
					}
					else
						w.storeWave(enemy.exposePos(), 2d*Math.atan(18d/w.distance(enemy.exposePos())), 0.9f, 0.1f);//(float)Math.pow(0.05, Math.max(1d,self.getRoundNum()/15d)));
					w.deactivate();
					final int[] smacks = w.indicateHits(enemy.exposePos());
					kNNHits += smacks[0];
					aSHits += smacks[1];
				}
			//}
		}
		
		final Bullet shot;
		if(fireTime == self.getTime() && self.getGunTurnRemainingRadians() == 0
			 && (bPower < self.getEnergy() || (Redacted.IS_MC && self.getEnergy() > 0)) && self.getGunHeat() == 0) {
			shot = body.setFireBullet(bPower);
			if(shot!=null) {
			_waves.add(new Wave(true, currKey, self.getX(),self.getY(),shotBearing,
				Rules.getBulletSpeed(bPower), self.getTime(), shotLatDir, vcs[shotAccel+1][latVelSeg][distSeg][vertSeg][wallSeg][revWallSeg],
					new double[]{mainGunAngle, antiSurfAngle})
			);
			if(useAntiSurf) {
				antiSurfUses ++;
			} else {
				mainGunUses ++;
			}
			}
		} else if(bPower < self.getEnergy()) {
			shot = null;
			_waves.add(new Wave(false, null, self.getX(),self.getY(),shotBearing,
				Rules.getBulletSpeed(bPower), self.getTime(), shotLatDir, vcs[shotAccel+1][latVelSeg][distSeg][vertSeg][wallSeg][revWallSeg],
					new double[]{mainGunAngle, antiSurfAngle})
			);
		} else {
			shot = null;
		}
		
		return shot;
		
}

	public void doAimPlan(final boolean shielded) {
		
		this.bPower = B_POWER_BASE;
		if(enemy.distance() < 336d) {
			bPower += (3d-bPower) * (1d-(enemy.distance()-36d)/275d);
		}
		/*if(self.getEnergy() < 16) {
			bPower -= .8d;
			if(self.getEnergy() < 12) {
				bPower -= 0.4d;
				if(self.getEnergy() < 7) {
					bPower -= 0.2d;
					if(self.getEnergy() < 3) {
						bPower = 0.1;
					}
				}
			}
		}*/
		if(1.15d*Rules.getBulletDamage(bPower)>=enemy.energy()&&enemy.energy()<=16) {
			bPower = 0.1d*Math.ceil(10d*((enemy.energy() + 2d)/6d));
			if(enemy.energy() <= 4) {
				bPower = 0.1d*Math.ceil(10d*Math.max(0.1d, enemy.energy()/4d));
			}
		}
		
		if(Redacted.IS_MC) 
			bPower = 3d;

		this.setSegmentation();

		this.determineFiringAngles();
		
		useAntiSurf = true;
		double firingAngle = antiSurfAngle;
		if(kNNHits >= aSHits) {
			firingAngle = mainGunAngle;
			useAntiSurf = false;
		}

		final double gunTurn = Utils.normalRelativeAngle(firingAngle - self.getGunHeadingRadians()) + (shielded ? (Math.random()-0.5d)*enemy.angWidth() : 0d);
		body.setTurnGunRightRadians(gunTurn);
		
		this.fireTime = currTime + 1;

	}
	
	public void chalkOneUp(final long currTime) {
		hits ++;
		/*for(final Wave w : _waves) {
			if(!w.isReal()) continue;
			if(Math.abs(w.distTravelled(currTime)-w.distance(enemy.exposePos())) <= w.velocity()) {
				System.out.println(offsetMap.remove(w.key()));
				w.key()[w.key().length-1] = 20;
				offsetMap.put(w.key(), w.getFactor(enemy.exposePos()));
			}
		}*/
	}
	
	private void setSegmentation() {
		accelSeg = enemy.accel() + 1;
		latVelSeg = ((int)Math.abs(enemy.velocity())+1)/2;
		wallSeg = Math.min((int)Math.floor(enemy.wallSpace() * 7), 2);
		revWallSeg = Math.min((int)Math.floor(enemy.revWallSpace() * 7), 2);
		distSeg = (int)Math.floor(enemy.distance() / Redacted.MAX_DIST) * 4;
		vertSeg = enemy.verticality();
		
		shotAccel = enemy.accel();
		shotLatDir = enemy.latDir();
		shotBearing = enemy.absBearingTo();
		
		//LAST ENTRY MUST BE ZERO
		currKey = new float[]{self.getRoundNum(), self.getTime(), accelSeg/2f,
				(float)Math.abs(enemy.velocity())/8f, (float)Math.abs(enemy.latVelocity())/8f, Math.min(0.3333f,(float)enemy.wallSpace())*3f, Math.min(0.33333f,(float)enemy.revWallSpace())*3f,
					distSeg / 3f, vertSeg, (float)bPower/3f,/* (float)Math.abs(Math.sin(enemy.absBearingTo())),*/0f};
	}
	
	private void determineFiringAngles() {
		
		final float[][] neighbors, antiSurfNeighbors;
		if(!offsetMap.isEmpty()) {
			final float[][] datMap = offsetMap.keySet().toArray(new float[0][]);
			neighbors = Rut.findKNNExc(NUM_NEIGHBORS,//ath.max(1,(int)Math.sqrt(offsetMap.size())),//Math.max(NUM_NEIGHBORS, Math.min(100, (int)Math.log(offsetMap.size()))),
				currKey, datMap);
			antiSurfNeighbors = Rut.findKNN(NUM_NEIGHBORS,//Math.max(1,(int)Math.sqrt(offsetMap.size())),//Math.max(NUM_NEIGHBORS, Math.min(100, (int)Math.log(offsetMap.size()))),
				currKey, datMap);
		}
		else {
			neighbors = new float[][]{};
			antiSurfNeighbors = new float[][]{};
		}

		final float[] dangerMap = vcs[accelSeg][latVelSeg][distSeg][vertSeg][wallSeg][revWallSeg];
		
		//final double liveMEA;
		final double stDev = enemy.angWidth();
		final double liveMEA = Math.asin(8d/Rules.getBulletSpeed(bPower)) * enemy.latDir();
		//final int targetBinWidth = (int)Math.ceil((double) BINS / Math.abs(2d*liveMEA) * enemy.angWidth());
		float maxDanger = 0f, maxFDanger = 0f, maxKDanger = 0f, maxASDanger = 0f;
		int maxBin = 3*(BINS-1)/8 + (BINS-1)/2, maxASBin = (BINS-1)/2;
		for(int centBin = 0; centBin < BINS; centBin ++) {
			float danger = 0f, kNNDanger = 0f, antiSurfDanger = 0f;
		//	float danger = 0f, kNNDanger = 0f, antiSurfDanger = 0f;
		//	for(int bin = centBin-targetBinWidth/2; bin < centBin+targetBinWidth/2; bin ++) {
				final int b = Math.max(0, Math.min(BINS-1, centBin));
				danger += dangerMap[b];
				maxFDanger = Math.max(maxFDanger, danger);
				final double binFactor = (2d*b-(BINS-1d))/(BINS-1d);
				for(final float[] neighbor : neighbors) {
					final double x = ((offsetMap.get(neighbor)-binFactor) * liveMEA / stDev );
					//kNNDanger += Math.exp(-0.5d*x*x) / Math.max(1d, Rut.manKeyDist(neighbor, currKey));
					kNNDanger += x*x > 1d ? 0d : 1d;
				}
				maxKDanger = Math.max(maxKDanger, kNNDanger);
				///danger += kNNDanger;
				for(final float[] neighbor : antiSurfNeighbors) {
					final double x = ((offsetMap.get(neighbor)-binFactor) * liveMEA / stDev);
					antiSurfDanger += x*x > 1d ? 0d : 1d;
				}
		//	}
			paintDangers[0][centBin] = danger + kNNDanger;
			paintDangers[1][centBin] = danger + antiSurfDanger;
			if(danger + kNNDanger > maxDanger) {
				maxDanger = danger + kNNDanger;
				maxBin = centBin;
			}
			if(danger + antiSurfDanger > maxASDanger) {
				maxASDanger = danger + antiSurfDanger;
				maxASBin = centBin;
			}
			
		}
		
//		System.out.println("MFD: " + maxFDanger);
//		System.out.println("MKD: " + maxKDanger);
//		System.out.println();
		mainGunAngle = (maxBin-(BINS-1)*0.5d)/((BINS-1d)*0.5d)
				* liveMEA + enemy.absBearingTo();
		antiSurfAngle = (maxASBin-(BINS-1)*0.5d)/((BINS-1d)*0.5d)
				* liveMEA + enemy.absBearingTo();
	}
	
	public void paint(final Graphics2D g) {
		for(final Wave w : _waves) {
			if(!w.isReal()) continue;
			g.setColor(Color.ORANGE);
			w.paintShots(g, self.getTime());
		//	final double distTravelled = w.distTravelled(currTime);
		//	g.draw(new Ellipse2D.Double(
		//		w.getX()-distTravelled,w.getY()-distTravelled,
		//		2d*distTravelled,2d*distTravelled)
	//		);
		}
		g.setColor(Color.BLUE);
		float maxDanger = 0f; float maxAntiSurf = 0f;
		for(final float danger : paintDangers[0]) {
			maxDanger = Math.max(danger, maxDanger);
		}
		for(final float danger : paintDangers[1]) {
			maxAntiSurf = Math.max(danger, maxAntiSurf);
		}
		final double binWidth = DISP_WIDTH / (double) BINS;
		double max = maxAntiSurf;
		for(int d = 1; d >= 0; d --) {
			
			for(int b = 0; b < paintDangers[d].length; b ++) {
				g.fill(
					new Rectangle2D.Double(
						Redacted.PLAY_WIDTH-DISP_WIDTH + b * binWidth,
						0,
						binWidth,
						paintDangers[d][b]/max*DISP_HEIGHT
					)
				);
			}
			max = maxDanger;
			g.setColor(Color.MAGENTA.darker());
		}
	}
	
	public void endRound() {
		_waves.clear();
		if(offsetMap.size() > MAX_MAP_SIZE) {
			final List<float[]> oldKeys = offsetMap.keySet().stream().sorted((a,b)->
				Float.compare(a[0],b[0])==0?Float.compare(a[1],b[1]):Float.compare(a[0],b[0])
			).limit(offsetMap.size()-MAX_MAP_SIZE).collect(Collectors.toList());
			for(final float[] oldKey : oldKeys) {
				offsetMap.remove(oldKey);
			}
			System.out.println("Dropped " + oldKeys.size() + " gun logs");
		}
		System.out.println("MainGun: " + mainGunUses + " uses");
		System.out.println("AntiSurf: " + antiSurfUses + " uses");
	}

}
