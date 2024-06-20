package rwh;
import rwh.util.Rut;
import robocode.*;
import robocode.util.Utils;
import java.awt.geom.Point2D;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.Arrays;

/**
 * MyClass - a class by Damij
 */
public class Wave
{
	private final double[] firingAngles;

	private final float[] bins, key;
	
	private final Point2D source;
	private final double baseAngle, velocity, mEA;
	private final long shotTime;
	private final int latDir;
	private boolean isActive = true, isReal;
	
	public Wave(final boolean isReal, final float[] key, final double x, final double y, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins, final double[] firingAngles) {
		this.source = new Point2D.Double(x,y);
		this.velocity = velocity;
		this.mEA = Math.asin(8d/velocity);
		this.baseAngle = baseAngle;
		this.shotTime = shotTime;
		this.latDir = latDir;
		
		this.bins = bins;
		this.key = key == null ? key : Arrays.copyOf(key, key.length);
		this.isReal = isReal;
		this.firingAngles = firingAngles ==
 null ? firingAngles : Arrays.copyOf(firingAngles, firingAngles.length);
	}
	
	public Wave(final boolean isReal, final float[] key, final double x, final double y, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins) {
		this(isReal, key, x, y, baseAngle, velocity, shotTime, latDir, bins, null);
	}

	public Wave(final float[] key, final double x, final double y, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins) {
		this(true, key, x, y, baseAngle, velocity, shotTime, latDir, bins, null);
	}
	
	public Wave(final float[] key, final Point2D source, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins) {
		this(true, key, source.getX(), source.getY(), baseAngle, velocity, shotTime, latDir, bins, null);
	}	
	
	public Wave(final double x, final double y, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins) {
		this(null, x, y, baseAngle, velocity, shotTime, latDir, bins);
	}

	public Wave(final boolean isReal, final float[] key, final Point2D source, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins) {
		this(isReal, key, source.getX(), source.getY(), baseAngle, velocity, shotTime, latDir, bins, null);
	}
	
	public Wave(final boolean isReal, final double x, final double y, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins) {
		this(isReal, null, x, y, baseAngle, velocity, shotTime, latDir, bins, null);
	}
	
	public Wave(final Point2D source, final double baseAngle,
				final double velocity, final long shotTime, final int latDir, final float[] bins) {
		this(source.getX(),source.getY(),baseAngle,velocity,shotTime,latDir,bins);
	}
	
	public double angleToSource(final Point2D pos) {
		return Math.atan2(source.getX()-pos.getX(),source.getY()-pos.getY());
	}
	
	public double angleFromSource(final Point2D pos) {
		return Math.atan2(pos.getX()-source.getX(),pos.getY()-source.getY());
	}
	
	public double angBotWidth(final Point2D pos) {
		return 2d*Math.atan(18d/pos.distance(source));
	}
	
	public double getFactor(final double bearing) {
		final double angularOffset = Utils.normalRelativeAngle(bearing-baseAngle);
		return angularOffset / mEA * latDir;
	}
	
	public double getFactor(final Point2D target) {
		return getFactor(this.angleFromSource(target));
	}
	
	public double distTravelled(final long currTime) {
		return (currTime-shotTime) * velocity;
	}
	
	public double distance(final Point2D loc) {
		return source.distance(loc);
	}
	
	public int[] indicateHits(final Point2D target) {
		final int[] hits = new int[firingAngles.length];
		final Rectangle2D.Double bot =
			new Rectangle2D.Double(
				target.getX()-18d,target.getY()-18d,
				36d, 36d
			)
		;
		for(int g = 0; g < firingAngles.length; g ++) {
			final Point2D bulletFuture = Rut.project(source, firingAngles[g], Redacted.MAX_DIST);
			final Line2D bulletHist =
				new Line2D.Double(source.getX(), source.getY(), bulletFuture.getX(), bulletFuture.getY());
			hits[g] = bot.intersects(bulletHist.getBounds()) ? 1 : 0;
		}
		return hits;
	}
	
	public int timeTillImpact(final Point2D pos, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		return (int)Math.ceil((pos.distance(source)-distTravelled)/velocity);
	}
	
	public int timeTillImpact(final double x, final double y, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		return (int)Math.ceil((Rut.distance(x-source.getX(),y-source.getY())-distTravelled)/velocity);
	}
	
	public boolean contacts(final Point2D pos, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		return new Ellipse2D.Double(source.getX()-distTravelled,
				source.getY()-distTravelled, 2*distTravelled, 2*distTravelled)
			.intersects(new Rectangle2D.Double(pos.getX()-18,pos.getY()-18,36,36));
	}
	
	public boolean contains(final Point2D pos, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		return new Ellipse2D.Double(source.getX()-distTravelled,
				source.getY()-distTravelled, 2*distTravelled, 2*distTravelled)
			.contains(new Rectangle2D.Double(pos.getX()-18,pos.getY()-18,36,36));
	}
	
	public boolean breaks(final Point2D pos, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		return new Ellipse2D.Double(source.getX()-distTravelled,
				source.getY()-distTravelled, 2*distTravelled, 2*distTravelled)
			.contains(pos);
	}
	
	public boolean contains(final double x, final double y, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		return new Ellipse2D.Double(source.getX()-distTravelled,
				source.getY()-distTravelled, 2*distTravelled, 2*distTravelled)
			.contains(new Rectangle2D.Double(x-18,y-18,36,36));
	}
	
	public boolean matches(final Bullet b, final long currTime, final boolean bhb) {
		final Point2D.Double bPos = new Point2D.Double(b.getX(), b.getY());
		return Math.abs(velocity-b.getVelocity()) < 0.25 &&
			Math.abs((currTime-shotTime-(bhb?1:0))*velocity-bPos.distance(source)) <= 1.7d*velocity;
	}
	
	public void storeWave(final Point2D place) {
	 	this.storeWave(angleFromSource(place), 2d*Math.atan(18d/place.distance(source)));
	}

	public void storeWave(final Point2D place, final float incoming, final float outgoing) {
		//Dont forget the factor here
	 	this.storeWave(angleFromSource(place), 2d*Math.atan(18d/place.distance(source)), incoming, outgoing);
	}
	
	public void storeWave(final Point2D place, final double angWidth, final float incoming, final float outgoing) {
		//Dont forget the factor here
	 	this.storeWave(angleFromSource(place), angWidth, incoming, outgoing);
	}
	
	public void storeWave(final double bearing, final double angularWidth) {
		final double angularOffset = Utils.normalRelativeAngle(bearing-baseAngle);
		final double factor = angularOffset / mEA * latDir;
		final int bin = (int)Math.round(factor*(bins.length-1)/2d + (bins.length-1)/2d);
		
		final double binWidth = angularWidth / mEA * (bins.length-1);
		for(int b = 0; b < bins.length; b ++) {
			final double x = (b-bin)/binWidth;
			bins[b] = 0.309f * bins[b] + 2.236f * (float) Math.exp(-0.5d*x*x);
		}
	}
	
	public void storeWave(final double bearing, final double angularWidth, final float incoming, final float outgoing) {
		final double angularOffset = Utils.normalRelativeAngle(bearing-baseAngle);
		final double factor = angularOffset / mEA * latDir;
		final int bin = (int)Math.round(factor*(bins.length-1)/2d + (bins.length-1)/2d);
		
		final double binWidth = angularWidth / mEA * (bins.length-1);
		for(int b = 0; b < bins.length; b ++) {
			final double x = (b-bin)/binWidth;
			bins[b] = incoming * bins[b] +  outgoing * (float) Math.exp(-0.5d*x*x);
		}
	}
	
	
	public boolean isActive() { return isActive; }
	public void deactivate() { this.isActive = false; }
	public double baseAmp() { return this.baseAngle; }
	public double velocity() { return this.velocity; }
	public double mEA() { return this.mEA; }
	public double getX() { return this.source.getX(); }
	public double getY() { return this.source.getY(); }
	public int latDir() { return latDir; }
	public float[] backend() { return bins; }
	public float[] key() { return key; }
	public boolean isReal() { return isReal; }
	
	@Override
	public int hashCode() {
		return Double.hashCode(baseAngle) ^ Double.hashCode(velocity) 
			^ Double.hashCode(mEA) ^ Long.hashCode(shotTime) ^ latDir;
	}
	
	public void paint(final Graphics2D g, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		g.draw(new Ellipse2D.Double(source.getX()-distTravelled,
				source.getY()-distTravelled, 2*distTravelled, 2*distTravelled));
		final Point2D center = Rut.project(source, baseAngle, distTravelled);
		final Point2D centerTail = Rut.project(source, baseAngle, distTravelled - velocity);
		g.draw(new Line2D.Double(centerTail.getX(),centerTail.getY(),center.getX(),center.getY()));
	}
	
	public void paintShots(final Graphics2D g, final long currTime) {
		final double distTravelled = (currTime-shotTime) * velocity;
		//g.draw(new Ellipse2D.Double(source.getX()-distTravelled,
		//		source.getY()-distTravelled, 2*distTravelled, 2*distTravelled));
		final Point2D mainGun = Rut.project(source, firingAngles[0], distTravelled);
		final Point2D mainGunTail = Rut.project(source, firingAngles[0], distTravelled - velocity);
		g.draw(new Line2D.Double(mainGunTail.getX(),mainGunTail.getY(),mainGun.getX(),mainGun.getY()));
		g.setColor(new Color(05,175,147));////<.brighter().brighter());
		final Point2D aSGun = Rut.project(source, firingAngles[1], distTravelled);
		final Point2D aSGunTail = Rut.project(source, firingAngles[1], distTravelled - velocity);
		g.draw(new Line2D.Double(aSGunTail.getX(),aSGunTail.getY(),aSGun.getX(),aSGun.getY()));
				
	}

}
