package comp128.gestureRecognizer;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

import edu.macalester.graphics.Point;

public class Template {
    
    String mainName;
    Deque<Point> mainPoints;

    /**
     * Stores template information, name and points of template.
     * @param name
     * @param points
     */
    public Template(String name, Deque<Point> points){
        mainName=name;
        mainPoints=points;
    }

    public Deque<Point> getPoints(){
        return mainPoints;
    }

    public String getName(){
        return mainName;
    }
}
