package damij;
import java.awt.geom.Point2D;
import robocode.util.Utils;
import java.awt.Graphics2D;

/**
 * Wave - a class by Damij
 */
public class Wave
{
	
	private final Point2D.Double source;
	
	private final double centralHeading, speed, power, mEA;
	
	private double distTravelled;
	
	private int velSeg, distSeg, accelSeg, nWallSeg, curl, roundNumber;
	
	public Wave(final Point2D.Double source,
		final double centralHeading, final double power,
			final int curl, final int velSeg, final int distSeg,
				final int accelSeg, final int nWallSeg, final int roundNumber) {
		this.source = new Point2D.Double(source.x, source.y);
		this.centralHeading = Utils.normalAbsoluteAngle(centralHeading);
		this.power = power;
		this.speed = 20 - 3 * power;
		this.mEA = Math.asin(8d/speed)*curl;
		this.velSeg = velSeg;
		this.distSeg = distSeg;
		this.accelSeg = accelSeg;
		this.nWallSeg = nWallSeg;
		this.curl = curl;
		this.roundNumber = roundNumber;
	}
	
	public void progress(final int delta) {
		distTravelled += speed * delta;
	}
	
	public boolean hasBroken(final Point2D.Double target) {
		return distTravelled >= source.distance(target);
	}
	
	public boolean willBreak(final Point2D.Double target, final int offset) {
		return (distTravelled + offset*speed) >= source.distance(target);
	}
	
	public void paint(final Graphics2D g) {
		g.drawOval((int)this.source.x-(int)distTravelled,(int)this.source.y-(int)distTravelled,
			(int)distTravelled*2, (int)distTravelled*2);
	}
	
	public double getOffset(final Point2D.Double pos) {
		final double absAngle = Math.atan2(pos.x-source.x, pos.y-source.y);
		return Utils.normalRelativeAngle(absAngle-centralHeading)
								/ mEA;
	}
	
	public double getOffset(final double heading) {
		System.out.println( Utils.normalRelativeAngle(heading-centralHeading));
		return Utils.normalRelativeAngle(heading-centralHeading) / mEA;
	}
	
	public double heading() {
		return centralHeading;
	}
	
	public double distTravelled() {
		return distTravelled;
	}
	
	public int velSeg() {
		return velSeg;
	}
	
	public int distSeg() {
		return distSeg;
	}
	
	public int accelSeg() {
		return accelSeg;
	}

	public int nWallSeg() {
		return nWallSeg;
	}
	
	public int roundNumber() {
		return roundNumber;
	}
	
	public double speed() {
		return speed;
	}
	
	public double power() {
		return power;
	}
	
	public double timeTillHit(final Point2D.Double p) {
		final double distLimit = p.distance(source) - distTravelled - speed;
		return (distLimit/speed);
	}
	
	public int curl() {
		return curl;
	}

	public double mEA() {
		return mEA;
	}

	public double sourceX() {
		return source.x;
	}
	
	public double sourceY() {
		return source.y;
	}

}
