package damij;
import robocode.*;
import java.awt.Color;
import robocode.util.Utils;
import java.awt.Graphics2D;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * Blister - a robot by (your name here)
 */
public class Blister extends AdvancedRobot
{

	public static final boolean MOVE= true, SHOOT = true;

	private static BraynGun weapon;

	private static Dodger dodger;
	
	private double lastRadarTurn = 0d;
	
	/**
	 * run: Blister's default behavior
	 */
	public void run() {
	
		super.setAdjustRadarForRobotTurn(true);
		super.setAdjustRadarForGunTurn(true);
		super.setAdjustGunForRobotTurn(true);
	
		if(weapon == null) {
			weapon = new BraynGun(this);
		}
		if(dodger == null) {
			dodger = new Dodger(this);
		}
		// Initialization of the robot should be put here

		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:

		 setColors(Color.WHITE,Color.GREEN,Color.WHITE); // body,gun,radar

		//turnRadarRightRadians(Double.POSITIVE_INFINITY);
		// Robot main loop
		while(true) {
			
			if(MOVE)
				dodger.cycle();
			update();
			execute();

		}
	}
	
	private void update() {
		if(getRadarTurnRemainingRadians()>=0)
			setTurnRadarRightRadians(//Math.PI/4d);
				Math.min(2*Math.PI,lastRadarTurn));
		lastRadarTurn = Math.max(2*Math.PI,lastRadarTurn);
	}	

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(final ScannedRobotEvent e) {
		if(getRadarTurnRemainingRadians()>=0) {
			handleRadar(e);
		}
		if(SHOOT)
			weapon.update(e);
		if(MOVE)
			dodger.update(e);
	}
	
	private void handleRadar(final ScannedRobotEvent e) {
		final double absBearing =
			Utils.normalRelativeAngle(getHeadingRadians() + e.getBearingRadians());
		
		final double headingDiff =
			Utils.normalRelativeAngle(getRadarHeadingRadians() - absBearing);
			
		setTurnRadarLeftRadians(
			lastRadarTurn = Math.min(1.75d*headingDiff, Math.PI/4d)
		);
		
		lastRadarTurn = Math.abs(lastRadarTurn);
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(final HitByBulletEvent e) {
		dodger.handleHitByBullet(e);
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(final HitWallEvent e) {
		
	}
	
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
		dodger.handleBulletHitBullet(e);
	}

	public void onPaint(final Graphics2D g2) {
		weapon.onPaint(g2);
		dodger.onPaint(g2);
	}
	
	public void onRoundEnd(final RoundEndedEvent e) {
		weapon.cleanup();
		dodger.cleanup();
	}
	
	public void onDeath(final RobotDeathEvent e) {
		//weapon.cleanup();
		dodger.cleanup();
	}
	
	public void onWin(final WinEvent e) {
		weapon.cleanup();
		//dodger.cleanup();
	}

}
