package DM.mega;
import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.List;
import java.util.ArrayList;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.Polygon;
import java.util.Iterator;
import java.util.HashMap;
import java.util.stream.Stream;
import java.util.stream.Collectors;
import java.util.Arrays;

/**
 * Regicide - a robot by Damij
 */
public class Regicide extends AdvancedRobot
{
	static final int DISPLAY_HEIGHT = 100, DISPLAY_WIDTH = 168,
		DISPLAY_X = 0, DISPLAY_Y = 0;

	private static MagicStick gun; //BanHammer
	
	private static Legs movement;
	
	private static Rectangle2D.Double playField;
	private static RoundRectangle2D.Double surfField;
	
	protected static double MAX_DIST;
	//ISTC = MOVE ONLY
	//ISMC = SHOOT ONLY
	private static final boolean isTC = false, isMC = false;
	
	public static boolean playTains(final Point2D.Double pos) {
		return playField.contains(pos);
	}
	
	public static boolean surfTains(final Point2D.Double pos) {
		return surfField.contains(pos);
	}
	
	
	private List<Bullet> liveShots;
	
	private long timeSinceScan;
	private ScannedRobotEvent lastScanEvent;

	/**
	 * run: Regicide's default behavior
	 */
	public void run() {
		// Initialization of the robot should be put here

		if(gun == null) {
			gun = new MagicStick(this, isMC);
		}
		
		if(movement == null) {
			movement = new Legs(this);
		}
		
		liveShots = new ArrayList<>();
		
		if(playField == null) {
			MAX_DIST =
				Math.sqrt(
					(super.getBattleFieldHeight()*super.getBattleFieldHeight() +
					super.getBattleFieldWidth()*super.getBattleFieldWidth()
				)
			);
			playField = new Rectangle2D.Double(18.0001,18.0001,super.getBattleFieldWidth()-36.0002,super.getBattleFieldHeight()-36.0002);
			
			surfField =  new RoundRectangle2D.Double(35.0001,35.0001,super.getBattleFieldWidth()-70.0002,super.getBattleFieldHeight()-70.0002, 0d, 0d);//getBattleFieldHeight(), getBattleFieldWidth());
		}

		setColors(Color.ORANGE.brighter().brighter(),Color.YELLOW.brighter(),Color.ORANGE.brighter().brighter()); // body,gun,radar

		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);

		// Robot main loop
		while(true) {
			doRadar();
			execute();
		}
	}
	
	private void doRadar() {
		if(timeSinceScan ++ > 1) {
			setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(final ScannedRobotEvent e) {
	
		lastScanEvent = e;
		timeSinceScan = 0;

		final double absBearing = Utils.normalRelativeAngle(
			super.getHeadingRadians() + e.getBearingRadians()
		);

		final double radarOffset = Utils.normalRelativeAngle(
			super.getRadarHeadingRadians() - absBearing
		);
		
		setTurnRadarLeftRadians(1.7d*radarOffset);
		
		if(!isTC) {
			final Bullet possibleShot = gun.update(e, absBearing);
			
			if(possibleShot != null) {
				liveShots.add(possibleShot);
			}
		
			final Iterator<Bullet> bitt = liveShots.iterator();
			while(bitt.hasNext()) {
				final Bullet looker = bitt.next();
				if(!looker.isActive()) {
					bitt.remove();
				}
			}
		}
		
		//movement stuff down here
		if(!isMC) {
			movement.update(e, liveShots, absBearing);
		}
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(final HitByBulletEvent e) {
		movement.storeBullet(e.getBullet(), true);
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(final HitWallEvent e) {
		
	}
	
	@Override
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
		gun.rackOneUp();
		movement.storeBullet(e.getHitBullet(), false);
	}
	
	@Override
	public void onPaint(final Graphics2D g) {
		g.setColor(Color.RED);
		for(final Bullet b : liveShots) {
			g.fill(new Ellipse2D.Double(b.getX()-3, b.getY()-3, 6, 6));
		}
		gun.onPaint(g);
		movement.onPaint(g);
		g.setColor(Color.GREEN);
		g.draw(surfField);
	}
	
	@Override
	public void onDeath(final DeathEvent e) {
		
	}
	
	@Override
	public void onWin(final WinEvent e) {
	
	}
	
	@Override
	public void onRoundEnded(final RoundEndedEvent e) {
		gun.reset();
		movement.reset();
	}
}
