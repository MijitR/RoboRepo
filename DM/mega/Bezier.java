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
import java.awt.geom.Path2D;
import java.awt.Polygon;
import java.util.Iterator;
import java.util.HashMap;
import java.util.stream.Stream;
import java.util.stream.Collectors;
import java.util.Arrays;

/**
 * CownCar - a robot by Damij
 */
public class Bezier extends AdvancedRobot
{

	static final int DISPLAY_HEIGHT = 94, DISPLAY_WIDTH = 172,
		DISPLAY_X = 0, DISPLAY_Y = 0;

	private static// MagicStick gun;
			BanHammer gun;
	
	private static Legs movement;
	
	private static Wheels rimjob;
	
	private static Rectangle2D.Double playField;
	private static RoundRectangle2D.Double surfField;
	
	protected static double MAX_DIST, WALL_DIST = 36d;
	//ISTC = MOVE ONLY
	//ISMC = SHOOT ONLY
	private static final boolean isTC = true, isMC = false;
	
	public static boolean playTains(final Point2D.Double pos) {
		return playField.contains(pos);
	}
	
	public static boolean isInTrack(final Rectangle2D segment) {
		return surfField.contains(segment);
	}
	
	public static boolean meetsTrack(final Rectangle2D segment) {
		return surfField.intersects(segment);
	}
	
	public static boolean isExterior(final Rectangle2D segment) {
		return !playField.contains(segment);
	}
	
	public static boolean surfTains(final Point2D.Double pos) {
		return surfField.contains(pos);
	}
	
	public static Path2D.Double getEnclosure() {
		return new Path2D.Double(surfField);
	}
	
	
	private List<Bullet> liveShots;
	
	private long timeSinceScan;
	private ScannedRobotEvent lastScanEvent;

	/**
	 * run: Regicide's default behavior
	 */
	public void run() {
		// Initialization of the robot should be put here
		liveShots = new ArrayList<>();
		
		if(playField == null) {
			MAX_DIST =
				Math.sqrt(
					(super.getBattleFieldHeight()*super.getBattleFieldHeight() +
					super.getBattleFieldWidth()*super.getBattleFieldWidth()
				)
			);
			playField = new Rectangle2D.Double(18.0001,18.0001,super.getBattleFieldWidth()-36.0002,super.getBattleFieldHeight()-36.0002);
			
			surfField =  new RoundRectangle2D.Double(WALL_DIST,WALL_DIST,super.getBattleFieldWidth()-2*WALL_DIST,super.getBattleFieldHeight()-2*WALL_DIST,//, 0d, 0d);
				Math.max(getBattleFieldHeight(), 0d), Math.max(getBattleFieldWidth()-0d,0d));
		}

		setColors(Color.MAGENTA.darker().darker().darker(),Color.YELLOW.brighter().brighter(),new Color(255, 235, 255)); // body,gun,radar

		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);
		

		if(gun == null) {
			//gun = new MagicStick(this, isMC);
			gun = new BanHammer(this, isMC);
		}
		
		//if(movement == null) {
			//movement = new Legs(this);
		//}
		
		if(rimjob == null) {
			rimjob = new Wheels(this);
			rimjob.setInactivityTime(450);
		}

	}
	
	private void doRadar() {
		if(timeSinceScan ++ > 1) {
			setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
		}
	}
	
	public void onStatus(final StatusEvent e) {
		doRadar();
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
			
			if(possibleShot != null && possibleShot.getPower() >= 1.0d) {
				rimjob.resetInactivityCounter();
			}
			
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
			//movement.update(e, liveShots, absBearing);
			rimjob.update(e, liveShots);
		}
		
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(final HitByBulletEvent e) {
		//movement.storeBullet(e.getBullet(), true);
		//movement.expectDamage(-Rules.getBulletHitBonus(e.getBullet().getPower()));
		final Bullet b = e.getBullet();
		rimjob.expectDamage(-Rules.getBulletHitBonus(b.getPower()));
		rimjob.storeBullet(b, true);
		//rimjob.resetInactivityCounter();
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(final HitWallEvent e) {
		
	}
	
	@Override
	public void onBulletHit(final BulletHitEvent e) {
		gun.rackOneUp();
		//movement.expectDamage(Rules.getBulletDamage(e.getBullet().getPower()));
		rimjob.expectDamage(Rules.getBulletDamage(e.getBullet().getPower()));
		//rimjob.resetInactivityCounter();
	}
	
	@Override
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
		//movement.storeBullet(e.getHitBullet(), false);
		gun.countaShield();
		rimjob.storeBullet(e.getHitBullet(), false);
	}
	
	@Override
	public void onPaint(final Graphics2D g) {
		g.setColor(Color.RED);
		for(final Bullet b : liveShots) {
			g.fill(new Ellipse2D.Double(b.getX()-3, b.getY()-3, 6, 6));
		}
		gun.onPaint(g);
		//movement.onPaint(g);
	//	g.setColor(Color.MAGENTA);
	//	g.draw(surfField);
		rimjob.onPaint(g);
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
		//movement.reset();
		rimjob.endRound();
	}
}
