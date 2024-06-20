package rwh;
import robocode.util.Utils;
import rwh.util.Rut;
import java.awt.geom.Point2D;


/**
 * MyClass - a class by Damij
 */
public class Tank
{

	private final Point2D pos;

	private double velocity, heading;
	
	private int dir;
	
	public Tank(final double x, final double y, final double velocity,
			final double heading , final int dir) {
		this.pos = new Point2D.Double(x,y);
		this.velocity = velocity;
		this.heading = heading;
		this.dir = dir;
	}
	
	void reset(final double x, final double y, final double velocity,
			final double heading, final int dir) {
		this.pos.setLocation(x,y);
		this.velocity = velocity;
		this.heading = heading;
		this.dir = dir;
	}
	
	double step(final Action a) {
		
		double turnLimit = a.limitTurn(velocity);
		double turnRemaining = Utils.normalRelativeAngle(a.turnAmount()-turnLimit);
		
		heading = Utils.normalAbsoluteAngle(heading + turnLimit);
		
		
		velocity =
			Rut.getNextVelocity(
				a.distance(), velocity, a.distance() < 0.73d ? dir : (int)Math.signum(a.distance()));
		
		dir = Math.abs(velocity) < 0.07d ? dir : (int)Math.signum(velocity);
		
		pos.setLocation(Rut.project(pos,heading,velocity));
		
		return turnRemaining;
	}
	
	public double velocity() { return velocity; }
	public double heading() { return heading; }
	public int dir() { return dir; }
	
	public double getX() { return pos.getX(); }
	public double getY() { return pos.getY(); }
	
	public Point2D popPos() {
		return new Point2D.Double(pos.getX(),pos.getY());
	}
	
	public Point2D exposePos() {
		return pos;
	}

}
