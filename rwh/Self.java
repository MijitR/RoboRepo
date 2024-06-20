package rwh;
import robocode.*;

/**
 * MyClass - a class by Damij
 */
public class Self
{

	private double heading, distRemaining, turnRemaining, gunTurnRemaining, gunHeat;
	private double x,y,velocity, energy, gunHeading;
	private long time;
	private int roundNum;
	
	public void updateStatus(final StatusEvent ev) {
		final RobotStatus e = ev.getStatus();
		setEnergy(e.getEnergy());
		setTime(e.getTime());
		setHeading(e.getHeadingRadians());
		setVelocity(e.getVelocity());
		setX(e.getX());
		setY(e.getY());
		setDistRemaining(e.getDistanceRemaining());
		setTurnRemaining(e.getTurnRemainingRadians());
		setRoundNum(e.getRoundNum());
		setGunTurnRemaining(e.getGunTurnRemainingRadians());
		setGunHeat(e.getGunHeat());
		setGunHeading(e.getGunHeadingRadians());
	}
	
	public void setEnergy(final double energy) {
		this.energy = energy;
	}
	
	public void setTime(final long time) {
		this.time = time;
	}
	
	public void setHeading(final double heading) {
		this.heading = heading;
	}
	
	public void setVelocity(final double velocity) {
		this.velocity = velocity;
	}
	
	public void setX(final double x) {
		this.x = x;
	}
	
	public void setY(final double y) {
		this.y = y;
	}
	
	public void setDistRemaining(final double distRemaining) {
		this.distRemaining = distRemaining;
	}
	
	public void setTurnRemaining(final double turnReamaining) {
		this.turnRemaining = turnRemaining;
	}
	
	public void setRoundNum(final int roundNum) {
		this.roundNum = roundNum;
	}
	
	public void setGunTurnRemaining(final double gunTurnRemaining) {
		this.gunTurnRemaining = gunTurnRemaining;
	}
	
	public void setGunHeat(final double gunHeat) {
		this.gunHeat = gunHeat;
	}
	
	public void setGunHeading(final double gunHeading) {
		this.gunHeading = gunHeading;
	}
	
	public double getBattleFieldWidth() {
		return Redacted.PLAY_WIDTH;
	}
	
	public double getBattleFieldHeight() {
		return Redacted.PLAY_HEIGHT;
	}
	
	public double getHeadingRadians() {
		return heading;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public double getDistanceRemaining() {
		return distRemaining;
	}
	
	public double getTurnRemainingRadians() {
		return turnRemaining;
	}
	
	public double getEnergy() {
		return energy;
	}
	
	public long getTime() {
		return time;
	}
	
	public int getRoundNum() {
		return roundNum;
	}
	
	public double getGunTurnRemainingRadians() {
		return gunTurnRemaining;
	}
	
	public double getGunHeat() {
		return gunHeat;
	}
	
	public double getGunHeadingRadians() {
		return gunHeading;
	}

}
