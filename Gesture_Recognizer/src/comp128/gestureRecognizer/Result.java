package comp128.gestureRecognizer;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import edu.macalester.graphics.Point;

public class Result {
    

    public Template templateMatch;
    public double scoreMatch;
    
    /**
     * Stores matching template with its corresponding score.
     * @param bestTemplate
     * @param score
     */
    public Result (Template bestTemplate, double score){
        templateMatch=bestTemplate;
        scoreMatch=score;
    }

    public double getScore(){
        return scoreMatch;
    }

    public Template getTemplate(){
        return templateMatch;
    }
}
