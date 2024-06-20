package DM;

/**
 * MyClass - a class by (your name here)
 */
public class GunSruff
{
	/*else {
		double radius = speed * headingChange;
		Circumference = 2*Math.PI*radius;
		double referenceAngle = Chicken.radToDeg(head) - 90;
		for(int i = 0; i < 10; i++){
		arcAngle = diff*headingChangeRate;
		double pXGain = Math.sin(arcAngle + referenceAngle);
		//Point2D.Double centerofCircleX = new Point2D.Double();
		//Point2D.Double centerofCircleY = new Point2D.Double();
		//Point2D.Double centerofCircle = new Point2D.Double();
		double VerticleToHeadingSlope = -1/getSlope(Chicken.radToDeg(head));
		Point2D.Double centerOfCircleY = new Point2D.Double(getYInterceptionPoints(radius, VerticleToHeadingSlope));
		Point2D.Double centerOfCircleX = new Point2D.Double(ImplyX(centerOfCircleY, radius, VerticleToHeadingSlope));
		Point2D.Double centerOfCircle = new Point2D.Double(Midpoint(centerOfCircleX.y, centerOfCircleX.x, centerOfCircleY.y, centerOfCircleY.x));
		double xLine = pXGain * radius + centerofCircle.x;
		Point2D.Double Interceptions = new Point2D.Double();
		Interceptions = getInterceptionPoints(radius, xLine);
		newX = Interceptions.x;
		newY = Interceptions.y;
		diff = Chicken.getRange(newX, newY, x, y)/(20-3*Chicken.bPower);
		}
	}
		return new Point2D.Double(newX, newY);
	}
	public double getSlope(double angle){
		double slope;
		slope = Math.tan(Chicken.degToRad(angle));
		return slope;
	}
	public Point2D.Double Midpoint(double x1, double x2, double y1, double y2){
		Point2D.Double midPoint = new Point2D.Double();
		//midPoint = (((x1-x2)/2), ((y1-y2)/2));
		return new Point2D.Double ((x1-x2)/2, ((y1-y2)/2));
	}
	public Point2D.Double getYInterceptionPoints(double radius, double slopeOfLine){
		Point2D.Double interceptionPointsY = new Point2D.Double();
		boolean fixed;
		double b;
		double m = slopeOfLine;
		double Point1;
		double Point2;
		double squareRoot;
		b = y;     //-------------------------------------------------------------------------------------------------------------------
		squareRoot = (radius*radius) - (x-radius)*(x-radius);
		if(squareRoot < 0){
			squareRoot*=-1;
			fixed = true;
		}
		Point1 = Math.sqrt(squareRoot) + radius - b - (m*x);
		if(fixed)
		Point2 = -1 * Math.sqrt(squareRoot) + radius - b - (m*x);
		//interceptionPointsY = (Point1, Point2);
		return new Point2D.Double (Point1, Point2);
	}
	public Point2D.Double ImplyX (Point2D.Double p, double radius, double slope) {
		double Point1;
		double Point2;
		double Point3;
		double Point4;
		boolean fixed;
		double b = y;
		double m = slope;
		
		Point2D.Double interceptionsPointsX = new Point2D.Double();
		double squareRoot;
		squareRoot = (radius*radius) - (p.y-radius)*(p.y-radius);
		if(squareRoot < 0){
			squareRoot*=-1;
			fixed = true;
		}
		Point1 = Math.sqrt(squareRoot) + radius - b - (m*x);
		if(fixed)
		Point2 = -1 * Math.sqrt(squareRoot) + radius - b - (m*x);
		
		squareRoot = (radius*radius) - (p.x-radius)*(p.x-radius);
		if(squareRoot < 0){
			squareRoot*=-1;
			fixed = true;
		}
		Point3 = Math.sqrt(squareRoot) + radius - b - (m*x);
		if(fixed)
		Point4 = -1 * Math.sqrt(squareRoot) + radius - b - (m*x);
		//interceptionPointsX = (Point1), Point3);
		return new Point2D.Double (Point1, Point3);
	}
	public Point2D.Double getInterceptionPoints(double radius, double x){
		//Point2D.Double interceptionPoints1 = new Point2D.Double();
		//Point2D.Double interceptionPoints1 = new Point2D.Double();
		Point2D.Double interceptionPoint = new Point2D.Double();
		boolean fixed;
		double YPoint1;
		double YPoint2;
		double XPoint1;
		double XPoint2;
		double squareRoot;
		squareRoot = (radius*radius) - (x-radius)*(x-radius);
		if(squareRoot < 0){
			squareRoot*=-1;
			fixed = true;
		}
		YPoint1 = Math.sqrt(squareRoot) + radius;
		if(fixed)
		YPoint2 = (-1 * Math.sqrt(squareRoot)) + radius;
		XPoint1 = x;
		XPoint2 = x;
		Point2D.Double interceptionPoints1 = new Point2D.Double(XPoint1, YPoint2);
		Point2D.Double interceptionPoints2 = new Point2D.Double(XPoint2, YPoint2);
		//interceptionPoint = checkPoints(interceptionPoints1, interceptionPoints2);
		return new Point2D.Double (checkPoints(interceptionPoints1, interceptionPoints2));
	}
	public Point2D.Double checkPoints(Point2D.Double Point1, Point2D.Double Point2){
		boolean Point;
		double Dist1 = Chicken.getrange(Point1, x, y);
		double Dist2 = Chicken.getrange(Point2, x, y);
		double Angle1 = (Dist1/Circumference) * 360;
		double Angle2 = (Dist2/Circumference) * 360;
		double Diff1 = arcAngle - Angle1;
		double Diff2 = arcAngle - Angle2;
		if(Diff1 < Diff2){
			return Point1;
		}
		else if(Diff2 < Diff1){
			return Point2;
		}
		return Point1;
	}
	*/
}
