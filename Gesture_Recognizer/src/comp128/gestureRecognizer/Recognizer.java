package comp128.gestureRecognizer;

import edu.macalester.graphics.CanvasWindow;
import edu.macalester.graphics.Ellipse;
import edu.macalester.graphics.GraphicsGroup;
import edu.macalester.graphics.Point;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

//template class= deque and a name
/**
 * Recognizer to recognize 2D gestures. Uses the $1 gesture recognition algorithm.
 */
public class Recognizer {


    private final double SIZE=250;
    private double boundingWidth;
    private double boundingHeight;
    public Deque<Template> templates;
    public double score;

    /**
     * Constructs a recognizer object
     */
    public Recognizer(){
    }

    /**
     * Takes in original gesture and reformats it so that it contains n points.
     * @param points Deque of original gesture
     * @param n Desired amount of points for resampled gesture
     * @return resampled Deque
     */
    public Deque<Point> resample(Deque<Point> points, int n){
        double resampleInterval = pathLength(points) / (n-1);
        Deque<Point> resampledPoints= new ArrayDeque<>();
        resampledPoints.add(points.getFirst());
        Iterator<Point> itr = points.iterator();

        Point firstPoint = itr.next();
        Point secondPoint=itr.next();

        double currentDistance=0;
        while (itr.hasNext()){
            double pointDistance = secondPoint.distance(firstPoint);
            
            if (currentDistance+ pointDistance>=resampleInterval){
                Point resampledPoint = Point.interpolate(firstPoint, secondPoint, (resampleInterval-currentDistance)/ firstPoint.distance(secondPoint));
                resampledPoints.add(resampledPoint);
                currentDistance=0;
                firstPoint=resampledPoint;
            }
            else{
                currentDistance+= pointDistance;
                firstPoint= secondPoint;
                secondPoint=itr.next();
            }
        }
        if (resampledPoints.size()!= n){
            resampledPoints.add(points.getLast());
        }
        return resampledPoints;
    }

    /**
     * Determines best matching template for the gesture drawn
     * @param gesture Deque of points from the gesture drawn
     * @return a result that contains the matching template and its match score
     */
    public Result recognize(Deque<Point> gesture){
        double bestDistance=100;
        Template bestTemplate= templates.getFirst();
        for (Template template: templates){
            double distance = distanceAtBestAngle(gesture, template.getPoints());
            if (distance<bestDistance){
                bestDistance=distance;
                bestTemplate=template;
            }
        }
        score = 1-(bestDistance/(.5*Math.sqrt(2 * (SIZE*SIZE))));
        Result match= new Result(bestTemplate, score);
        System.out.println("Score is" + score);
        return match;
    }
    

    public double getScore(){
        return score;

    }


    /**
     * Create a template to use for matching
     * @param name of the template
     * @param points in the template gesture's path
     */
    public void addTemplate(String name, Deque<Point> points){
        templates= new ArrayDeque<>();
        Template newTemplate= new Template(name, points);
        templates.add(newTemplate);
    }


    /**
     * Moves the gesture to a uniform position, according to point k
     * @param points
     * @param k
     * @return new Deque of translated points
     */
    public Deque<Point> translateTo(Deque<Point> points, Point k){
        Deque<Point> translatedPoints= new ArrayDeque<>();
        Point centroid = calcCentroid(points);
        for (Point point: points){
            Point newPoint= point.add(k).subtract(centroid);
            translatedPoints.add(newPoint);
        }
        return translatedPoints;
    }

    /**
     * Calculates the angle between the gesture and the x axis
     * @param points Deque of points from the festure drawn
     * @return the indicative angle, in degrees
     */
    public double indicativeAngle(Deque<Point> points){
       double xCount=0;
       double yCount=0;
        for (Point point : points){
            xCount+= point.getX();
            yCount+= point.getY();
       }
       xCount= xCount/points.size();
       yCount= yCount/points.size();
       double xDiff= xCount-points.getFirst().getX();
       double yDiff= yCount-points.getFirst().getY();
       double indicativeAngle = Math.atan2(yDiff, xDiff);

        return indicativeAngle;
    }

    /**
     * Calculates the middle point of a gesture
     * @param points Deque of points of gesture drawn
     * @return center point of gesture
     */
    public Point calcCentroid(Deque<Point> points){
        double xCount=0;
       double yCount=0;
        for (Point point : points){
            xCount+= point.getX();
            yCount+= point.getY();
       }
       xCount= xCount/points.size();
       yCount= yCount/points.size();
       Point centroid= new Point(xCount, yCount);
       return centroid;
    }

    /**
     * Calulates the length and width of the gesture
     * @param points Deque of points of gesture drawn
     */
    public void boundingBox(Deque<Point> points){
        double maxX=0;
        double maxY=0;
        double minX=points.peekFirst().getX();
        double minY=points.peekFirst().getY();
        for (Point point: points){
            if (point.getX()> maxX){
                maxX=point.getX();
            }
            if (point.getY()> maxY){
                maxY=point.getY();
            }
            if (point.getX()<minX){
                minX=point.getX();
            }
            if(point.getY()<minY){
                minY=point.getY();
            }
        }
        boundingWidth= maxX-minX;
        boundingHeight= maxY-minY;
    }

    public double getBoundingWidth(){
        return boundingWidth;
    }

    public double getBoundingHeight(){
        return boundingHeight;
    }

    /**
     * Scales gesture to a square proportional to size
     * @param points Deque of points of drawn gesture
     * @param size 
     * @return
     */
    public Deque<Point> scaleTo(Deque<Point> points, double size){
        boundingBox(points);
        Deque<Point> scaledPoints= new ArrayDeque<>();
        for (Point point:points){
            Point scaledPoint = point.scale(size/boundingWidth, size/boundingHeight);
            scaledPoints.add(scaledPoint);
        }
        return scaledPoints;
    }

    /**
     * Rotates gesture to uniform angle on the x axis
     * @param points Deque of points of drawn gesture
     * @param angle indicative angle
     * @return new deque of rotated points
     */
    public Deque<Point> rotateBy(Deque<Point> points, double angle){
        Deque<Point> rotatedPoints = new ArrayDeque<>();
        for (Point point : points){
            angle= angle*-1;
            rotatedPoints.add(point.rotate(angle, calcCentroid(points)));
        }
        return rotatedPoints;
    }

    /**
     * Calculates total length of gesture
     * @param points deque of points of drawn gesture
     * @return Total length of gesture
     */
    public double pathLength(Deque<Point> points){
        Iterator<Point> itr= points.iterator();
        Point tempPoint = itr.next();
        Point currentPoint;
        double totalDistance=0;
        while (itr.hasNext()){
            currentPoint = itr.next();
            double distance1= tempPoint.distance(currentPoint);
            totalDistance += distance1;
            tempPoint=currentPoint;
        }
        return totalDistance;
    }
  
    /**
     * Uses a golden section search to calculate rotation that minimizes the distance between the gesture and the template points.
     * @param points
     * @param templatePoints
     * @return best distance
     */
    private double distanceAtBestAngle(Deque<Point> points, Deque<Point> templatePoints){
        double thetaA = -Math.toRadians(45);
        double thetaB = Math.toRadians(45);
        final double deltaTheta = Math.toRadians(2);
        double phi = 0.5*(-1.0 + Math.sqrt(5.0));// golden ratio
        double x1 = phi*thetaA + (1-phi)*thetaB;
        double f1 = distanceAtAngle(points, templatePoints, x1);
        double x2 = (1 - phi)*thetaA + phi*thetaB;
        double f2 = distanceAtAngle(points, templatePoints, x2);
        while(Math.abs(thetaB-thetaA) > deltaTheta){
            if (f1 < f2){
                thetaB = x2;
                x2 = x1;
                f2 = f1;
                x1 = phi*thetaA + (1-phi)*thetaB;
                f1 = distanceAtAngle(points, templatePoints, x1);
            }
            else{
                thetaA = x1;
                x1 = x2;
                f1 = f2;
                x2 = (1-phi)*thetaA + phi*thetaB;
                f2 = distanceAtAngle(points, templatePoints, x2);
            }
        }
        return Math.min(f1, f2);
    }

    /**
     * 
     * @param points
     * @param templatePoints
     * @param theta
     * @return
     */
    private double distanceAtAngle(Deque<Point> points, Deque<Point> templatePoints, double theta){
        Deque<Point> rotatedPoints = null;
        rotatedPoints = rotateBy(points, -theta);
        return pathDistance(rotatedPoints, templatePoints);
    }

    /**
     * Calcualtes average distance between points of template and drawn gesture
     * @param a Deque of points of gesture
     * @param b Deque of points of gesture
     * @return average distance between template and gesture
     */
    public double pathDistance(Deque<Point> a, Deque<Point> b){
        Iterator<Point> itrA= a.iterator();
        Iterator<Point> itrB= b.iterator();
        double totalPointDistance=0;
        while (itrA.hasNext()&& itrB.hasNext()){
            Point pointA= itrA.next();
            Point pointB = itrB.next();
            totalPointDistance+= pointA.distance(pointB);
        }
        totalPointDistance/=a.size();
        return totalPointDistance;
    }

}