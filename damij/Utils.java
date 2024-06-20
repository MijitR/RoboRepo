package damij;
import java.awt.geom.Point2D;

/**
 * Utils - MijitR
 */
public class Utils
{

	public static final Point2D.Double project(final Point2D.Double start, final double dist, final double angle) {
		
		return new Point2D.Double(
			Math.sin(angle)*dist + start.x,
			Math.cos(angle)*dist + start.y
		);

	}


}
