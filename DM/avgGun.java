package DM;

/**
 * MyClass - a class by (your name here)
 */
public class avgGun
{
	double absoluteBearing = getHeading() + enemy.bearing;
		double absBearing = getHeadingRadians() + e.getBearingRadians();
		double bearingFromGun = normalRelativeAngle(absoluteBearing - getGunHeading());
		if(enemy.RepetitionGun&&enemy.speed!=0){
			
			if(enemy.speed!=newSpeed||timeSinceScan>15){
			oldSpeed = newSpeed;
			newSpeed = enemy.speed;
			 if(avgTimes>=1 && getTime()-timeOfScan>enemy.speed){
				enemy.newPlaceY = enemy.y;
				enemy.newPlaceX = enemy.x;
				enemy.avgX = (enemy.newPlaceX+enemy.oldPlaceX)/2;
				enemy.avgY = (enemy.newPlaceY+enemy.oldPlaceY)/2;
				enemy.oldPlaceY = enemy.newPlaceY;
				enemy.oldPlaceX = enemy.newPlaceX;
				//System.out.println(enemy.avgX + "\n  " + enemy.avgY);
				timeOfScan = getTime();
			 }
			 else{
				avgTimes++;
				enemy.oldPlaceX = enemy.x;
				enemy.oldPlaceY = enemy.y;
			 }
	//double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(enemy.avgY - getY(),enemy.avgX -  getX()));
			}
		double gunOffset = getGunHeadingRadians() - (Math.PI/2 - Math.atan2(enemy.avgY - getY(),enemy.avgX -  getX()));
		setTurnGunLeftRadians(NormaliseBearing(gunOffset)); out.println("Target on the move, firing using avging aiming");
		//if(getGunTurnRemaining()<3&&getGunHeat()==0&&getEnergy()>3)
		fire(bPower);
         
		}
}
