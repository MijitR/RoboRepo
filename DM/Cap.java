package DM;
import robocode.*;
import robocode.util.Utils;

public class Cap {
	
static double timeTillHit;
static double bulletSpeed;
static double timeOfHit;

	Cap(){
		System.out.println("Cap Class Initiated");
	}
	
	public static double CtDn(boolean shot, double lifeDif, double timeOfShot, double countdown, double distance, double x, double y){
		
				bulletSpeed = 20 - (3 * (lifeDif * -1));
				timeTillHit = distance/bulletSpeed;
				timeOfHit = Math.round(timeOfShot + timeTillHit);
				//System.out.println("distance " + distance + "   bSpeed " + bulletSpeed + "   time " + timeOfShot);

		return timeOfHit;
	}

}