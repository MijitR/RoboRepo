package rwh;
import robocode.*;
import robocode.util.Utils;
import java.awt.geom.Point2D;

/**
 * MyClass - a class by Damij
 */
public class Enemy
{

	private Self self;
	
	private Point2D center, lastCenter;

	private double bPower = 0.1d;

	private double velocity, heading, distance, latDirection, latHeading, latVelocity, turnRate,
		bearing, absBearingTo, wallSpace, revWallSpace, energy,
		advVelocity, angWidth, expHealthDelta;
		
	private long scanTime = 0l;

	private int accel = 1, dir, latDir, sideWithUs, sideOfUs;
	
	private boolean shot;
	
	public Enemy(final Self self) {
		this.self = self;
		this.center = new Point2D.Double(0d,0d);
		this.lastCenter = new Point2D.Double(0d,0d);
	}
	
	public double update(final ScannedRobotEvent e) {
		final double scanDelta = (double)(self.getTime()-this.scanTime);
		this.scanTime = self.getTime();
		turnRate = Utils.normalRelativeAngle(e.getHeadingRadians()-heading)/scanDelta;
		bearing = e.getBearingRadians();
		distance = e.getDistance();
		accel = (velocity == e.getVelocity()) ? accel : (int)Math.signum(Math.abs(e.getVelocity()) - Math.abs(velocity));
		angWidth = 2d*Math.atan(18d/distance);
		absBearingTo = Utils.normalRelativeAngle(self.getHeadingRadians() + bearing);
		lastCenter.setLocation(center);
		center.setLocation(self.getX() + distance*Math.sin(absBearingTo), self.getY() + distance*Math.cos(absBearingTo));
		
		final double hDelt = (energy - expHealthDelta - e.getEnergy());
		shot = ((hDelt)) > 0.11d && (hDelt) <= 3d
				&& Math.abs(Math.abs(velocity)-Math.abs(e.getVelocity())) < 2.05d
			&& distance > 51;
			
		if(shot) {
			bPower = hDelt;
		}
			
		expHealthDelta = 0d;

		velocity = e.getVelocity();
		heading = e.getHeadingRadians();
		energy = e.getEnergy();
		
		sideWithUs = (latHeading=Utils.normalRelativeAngle(heading-absBearingTo)) >= 0 ? 1 : -1;
		sideOfUs = bearing < 0 ? -1 : 1;

		dir = Math.abs(velocity) < 0.07d ? dir : (int)Math.signum(velocity);
		latDir = Math.abs(bearing) < 0.001d ? latDir : dir * sideWithUs;
		latVelocity = Math.abs(velocity * Math.sin(heading - absBearingTo));
		advVelocity = velocity * -Math.cos(heading - absBearingTo);
		
		double eGoing = heading;
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
		revWallSpace = Math.max(0, Math.min(1,(float)(Math.min(wallDistLatRev, wallDistVirtRev)/ Math.max(Redacted.PLAY_WIDTH, Redacted.PLAY_HEIGHT))));
		return absBearingTo;
	}
	
	public void expectHealthDelta(final double delta) {
		this.expHealthDelta += delta;
	}
	

	public void cancelShot() {
		this.shot = false;
	}
	
	public double angleTo(final Point2D ref) {
		return Math.atan2(ref.getX()-center.getX(), ref.getY()-center.getY());
	}

	
	public double velocity() { return velocity; }
	public double heading() { return heading; }
	public double distance() { return distance; }
	public double turnRate() { return turnRate; }
	public double bearing() { return bearing; }
	public double absBearingTo() { return absBearingTo; }
	public double wallSpace() { return wallSpace; }
	public double revWallSpace() { return revWallSpace; }
	public double energy() { return energy; }
	public double angWidth() { return angWidth; }
	public double shotPower() { return bPower; }
	public double latVelocity() { return latVelocity; }
	public double advVelocity() { return advVelocity; }
	public double latHeading() { return latHeading; }
	public int sideWithUs() { return sideWithUs; }
	public int sideOfUs() { return sideOfUs; }
	public int accel() { return accel; }
	public int dir() { return dir; }
	public int latDir() { return latDir; }
	public int verticality() { return Math.sin(Math.abs(latHeading)) > 0.5d ? 1  : 0; }
	public boolean shot() { return shot; }
	public void setHealth(final double health) { this.energy = health; }
	
	public double getX() { return center.getX(); }
	public double getY() { return center.getY(); }
	public Point2D exposePos() { return center; }
	public Point2D exposeShotPos() { return lastCenter; }

}
