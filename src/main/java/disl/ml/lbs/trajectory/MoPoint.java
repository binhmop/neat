package disl.ml.lbs.trajectory;


import java.util.List;
import edu.gatech.lbs.core.vector.*;


public class MoPoint {

  private int segid;
//  private int cellid;
  private long x;
  private long y;


  public MoPoint(int segid, IVector v) {
    this.segid = segid;
    this.x = v.toCartesianVector().getX();
    this.y = v.toCartesianVector().getY();
  }
  
  public MoPoint(int segid, long x, long y) {
    this.segid = segid;
    this.x = x;
    this.y = y;
  }

  public int getSegid() {
    return segid;
  }

  public void setSegid(int segid) {
    this.segid = segid;
  }

  public CartesianVector getV() {
    return new CartesianVector(x,y);
  }

  public long getX() {
    return x;
  }

  public void setX(long x) {
    this.x = x;
  }

  public long getY() {
    return y;
  }

  public void setY(long y) {
    this.y = y;
  }

  public double distToPoint(MoPoint p) {
    double distance;
    long dx = p.getX() - this.getX();
    long dy = p.getY() - this.getY();
    distance = Math.sqrt(dx * dx + dy * dy);
    return distance;
  }
  
  public static double perpendicularDistance (CartesianVector Point1, CartesianVector Point2, CartesianVector Point)
  {
    //Area = |(1/2)(x1y2 + x2y3 + x3y1 - x2y1 - x3y2 - x1y3)|   *Area of triangle
    //Base = v((x1-x2)�+(y1-y2)�)                               *Base of Triangle*
    //Area = .5*Base*H                                          *Solve for height
    //Height = Area/.5/Base
    //Math.Round(value, 2))

    /*double area = Math.Abs(.5 * (Point1.getX() * Point2.getY() + Point2.getX() *
    Point.getY() + Point.getX() * Point1.getY() - Point2.getX() * Point1.getY() - Point.getX() *
    Point2.getY() - Point1.getX() * Point.getY()));*/

    double tmp1 = Point1.getX() * Point2.getY() - Point2.getX() * Point1.getY();
    double tmp2 = Point2.getX() * Point.getY() - Point.getX() * Point2.getY();
    double tmp3 = Point.getX() * Point1.getY() - Point1.getX() * Point.getY();

    double area = Math.abs(.5 * (tmp1 + tmp2 + tmp3));

    double bottom = Math.sqrt(Math.pow(Point1.getX() - Point2.getX(), 2) + Math.pow(Point1.getY() - Point2.getY(), 2));
    if (bottom != 0)
    {
        double height = area / bottom * 2;
        return height;
    }
    else return 0;
    
  }
  public static double lehmerMean(CartesianVector Point1, CartesianVector Point2, List<CartesianVector> points){ 
    double denominatorSum=0;
    double numberatorSum=0;
    for (CartesianVector p: points){
      double d = perpendicularDistance(Point1,Point2,p);
      denominatorSum+=d;
      numberatorSum+=d*d;
    }
    return (numberatorSum/denominatorSum);
  }

}
