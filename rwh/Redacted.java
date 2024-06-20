package rwh;
import robocode.*;
import robocode.util.Utils;
import java.util.List;
import java.util.ArrayList;
import java.awt.Graphics2D;
import java.awt.Color;

/**
 * Redacted - a robot by Damij
 */
public class Redacted extends AdvancedRobot
{

	private static MoveSim movement;
	
	private static Weapon weapon;
	
	private static final List<Bullet> _shaders = new ArrayList<>();
	
	private static Enemy enemy;
	
	private static Self self;
	
	public static double PLAY_WIDTH, PLAY_HEIGHT, MAX_DIST;
	
	public static boolean IS_TC = false, IS_MC = false;
	
	private int shots, bhbs;

	/**
	 * run: Redacted's default behavior
	 */
	public void run() {
		// Initialization of the robot should be put here

		PLAY_WIDTH = super.getBattleFieldWidth();
		PLAY_HEIGHT = super.getBattleFieldHeight();
		MAX_DIST = Math.sqrt(PLAY_WIDTH*PLAY_WIDTH+PLAY_HEIGHT*PLAY_HEIGHT);
		
		if(movement == null || self == null) {
			self = new Self();
			enemy = new Enemy(self);
			movement = new MoveSim(self, enemy);
			weapon = new Weapon(this, self, enemy);
		}
		
		super.setAdjustRadarForGunTurn(true);
		super.setAdjustRadarForRobotTurn(true);
		super.setAdjustGunForRobotTurn(true);

		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:

		setColors(
			new Color(214,108,201),
			new Color(231,201,169),
			new Color(235,208,87)
		); // body,gun,radar
		
		super.setEventPriority("PaintEvent", 1);
		super.setEventPriority("StatusEvent", 96);
		super.setEventPriority("ScannedRobotEvent", 95);
		super.setEventPriority("BulletHitEvent", 98);
		super.setEventPriority("BulletHitBulletEvent", 98);
		super.setEventPriority("HitByBulletEvent", 98);
		
		super.addCustomEvent(
			new Condition("MovePlanEvent", 94) {
				@Override
				public boolean test() {
					return true;
				}
			}
		);
		super.addCustomEvent(
			new Condition("EndTurn", 0) {
				@Override
				public boolean test() {
					return true;
				}
			}
		);
		super.addCustomEvent(
			new Condition("ShotEvent", 98) {
				@Override
				public boolean test() {
					return true;
				}
			}
		);
		
		super.addCustomEvent(
			new Condition("AimPlanEvent", 94) {
				@Override
				public boolean test() {
					return true;
				}
			}
		);
	}
	
	public void onCustomEvent(final CustomEvent e) {
		if(e.getCondition().getName().equals("MovePlanEvent"))
			follow(movement.sim(_shaders));
		else if(e.getCondition().getName().equals("AimPlanEvent")) {
			if(!IS_TC) {
				weapon.doAimPlan(((float)bhbs)/(Math.max(1f, shots)) >= 0.3d);
			}
		}
		else if(e.getCondition().getName().equals("ShotEvent")) {
			if(!IS_TC) {
				final Bullet shot;
				if((shot=weapon.update())!=null) {
				
				}
			}
		}
		else {
			enemy.cancelShot();
			execute();
		}
	}
	
	private void follow(final Action a) {
		if(!IS_MC) {
			super.setAhead(a.distance());
			super.setTurnRightRadians(a.turnAmount());
		}
	}
	
	@Override
	public void onStatus(final StatusEvent e) {
		if(self != null) {
			self.updateStatus(e);
			movement.updateStatus(e);
		}
		setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(final ScannedRobotEvent e) {
		setTurnRadarLeftRadians(1.7d*Utils.normalRelativeAngle(getRadarHeadingRadians() - enemy.update(e)));
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(final HitByBulletEvent e) {
		enemy.expectHealthDelta(
			-Rules.getBulletHitBonus(
				e.getBullet().getPower()
			)
		);
		
		movement.handleHit(e.getBullet(), false);
	}
	
	@Override
	public void onBulletHit(final BulletHitEvent e) {
		enemy.expectHealthDelta(
			Rules.getBulletDamage(
				e.getBullet().getPower()
			)
		);
		shots ++;
		weapon.chalkOneUp(super.getTime());
	}
	
	@Override
	public void onBulletMissed(final BulletMissedEvent e) {
		shots ++;
	}
	
	@Override
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
		movement.handleHit(e.getHitBullet(), true);
		this.bhbs ++;
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(final HitWallEvent e) {
		
	}
	
	@Override
	public void onPaint(final Graphics2D g) {
		movement.paint(g);
		weapon.paint(g);
	}
	
	@Override
	public void onRoundEnded(final RoundEndedEvent e) {
		this._shaders.clear();
		movement.endRound();
		weapon.endRound();
	}

}