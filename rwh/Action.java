package rwh;
import robocode.*;
import robocode.util.Utils;
import rwh.util.Rut;
import java.awt.geom.Point2D;

/**
 * MyClass - a class by Damij
 */
public class Action
{

	public static final Action create(final Point2D source, final Point2D target, final double currHeading) {
		final double dist;
		return new Action(dist = source.distance(target),
			((Math.max(0,dist-0.8d)) == 0 ? 0 : 1) * Utils.normalRelativeAngle(Math.atan2(target.getX()-source.getX(),target.getY()-source.getY()) - currHeading)
		);
	}
	
	public static final Action createAmplified(final Point2D source, final Point2D target, final double currHeading) {
		final double dist;
		return new Action(dist = (100d*source.distance(target)),
			((Math.max(0,dist-0.8d)) == 0 ? 0 : 1) * Utils.normalRelativeAngle(Math.atan2(target.getX()-source.getX(),target.getY()-source.getY()) - currHeading)
		);
	}

	private final int chainID;

	private final double distance;
	
	private double turnAmount;
	
	public Action(final double distance, final double turnAmount) {
		this(0, distance, turnAmount);
	}

	public Action(final int chainID,  double distance, final double turnAmount) {
		this.turnAmount = turnAmount;
		int dir = (int) Math.signum(distance*100d);
		if(Math.abs(this.turnAmount) > Math.PI*0.5d) {
			dir *= -1;
			this.turnAmount = Utils.normalRelativeAngle(turnAmount + Math.PI);
		}
		this.distance = Math.abs(distance) * dir;
		this.chainID = chainID;
	}
	
	public double limitTurn(final double velocity) {
		return Rut.limit(-Rules.getTurnRateRadians(Math.abs(velocity)), turnAmount, Rules.getTurnRateRadians(Math.abs(velocity)));
	}
	
	public int chainID() {
		return chainID();
	}
	
	public double distance() {
		return distance;
	}
	
	public double turnAmount() {
		return turnAmount;
	}

}
