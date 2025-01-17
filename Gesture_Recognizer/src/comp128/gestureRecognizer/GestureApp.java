package comp128.gestureRecognizer;

import edu.macalester.graphics.*;
import edu.macalester.graphics.Point;
import edu.macalester.graphics.events.MouseButtonEventHandler;
import edu.macalester.graphics.ui.Button;
import edu.macalester.graphics.ui.TextField;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.function.Consumer;

/**
 * The window and user interface for drawing gestures and automatically recognizing them
 * Created by bjackson on 10/29/2016.
 */
public class GestureApp {

    private CanvasWindow canvas;
    private Recognizer recognizer;
    private IOManager ioManager;
    private GraphicsGroup uiGroup;
    private Button addTemplateButton;
    private TextField templateNameField;
    private GraphicsText matchLabel;
    private Deque<Point> path;
    private Result result;
  


    public GestureApp(){
        canvas = new CanvasWindow("Gesture Recognizer", 600, 600);
        recognizer = new Recognizer();
        path = new ArrayDeque<>();
        ioManager = new IOManager();
        setupUI();
        result= recognizer.recognize(path);
    }

    public Deque<Point> getPath(){
        return path;
    }
    /**
     * Create the user interface
     */
    private void setupUI(){
        matchLabel = new GraphicsText("Match: " + recognizer.getScore());
        matchLabel.setFont(FontStyle.PLAIN, 24);
        canvas.add(matchLabel, 10, 30);

        uiGroup = new GraphicsGroup();

        templateNameField = new TextField();

        addTemplateButton = new Button("Add Template");
        addTemplateButton.onClick( () -> addTemplate() );


        Point center = canvas.getCenter();
        double fieldWidthWithMargin = templateNameField.getSize().getX() + 5;
        double totalWidth = fieldWidthWithMargin + addTemplateButton.getSize().getX();


        uiGroup.add(templateNameField, center.getX() - totalWidth/2.0, 0);
        uiGroup.add(addTemplateButton, templateNameField.getPosition().getX() + fieldWidthWithMargin, 0);
        canvas.add(uiGroup, 0, canvas.getHeight() - uiGroup.getHeight());

        Consumer<Character> handleKeyCommand = ch -> keyTyped(ch);
        canvas.onCharacterTyped(handleKeyCommand);

        canvas.onMouseDown(event -> {
            removeAllNonUIGraphicsObjects();
            path.clear();
            path.add(event.getPosition());
        });

        
        canvas.onDrag(event -> {
            if (!path.isEmpty()){
                Line newLine = new Line(event.getPosition(), path.getLast());
                canvas.add(newLine);
                path.add(event.getPosition());
            }
        });

        canvas.onMouseUp(event -> {
            Result newResult= recognizer.recognize(path);
            matchLabel.setText("Match:" + newResult.getScore() + " " + newResult.getTemplate().getName());
            System.out.println(result.getTemplate().getName());
        });
    }

    /**
     * Clears the canvas, but preserves all the UI objects
     */
    private void removeAllNonUIGraphicsObjects() {
        canvas.removeAll();
        canvas.add(matchLabel);
        canvas.add(uiGroup);
    }

    /**
     * Handle what happens when the add template button is pressed. This method adds the points stored in path as a template
     * with the name from the templateNameField textbox. If no text has been entered then the template is named with "no name gesture"
     */
    private void addTemplate() {
        String name = templateNameField.getText();
        if (name.isEmpty()){
            name = "no name gesture";
        }
        recognizer.addTemplate(name, path); // Add the points stored in the path as a template
    }

   

    /**
     * Handles keyboard commands used to save and load gestures for debugging and to write tests.
     * Note, once you type in the templateNameField, you need to call canvas.requestFocus() in order to get
     * keyboard events. This is best done in the mouseDown callback on the canvas.
     */
    public void keyTyped(Character ch) {
        if (ch.equals('L')){
            String name = templateNameField.getText();
            if (name.isEmpty()){
                name = "gesture";
            }
            Deque<Point> points = ioManager.loadGesture(name+".xml");
            if (points != null){
                recognizer.addTemplate(name, points);
                System.out.println("Loaded "+name);
            }
        }
        else if (ch.equals('s')){
            String name = templateNameField.getText();
            if (name.isEmpty()){
                name = "gesture";
            }
            ioManager.saveGesture(path, name, name+".xml");
            System.out.println("Saved "+name);
        }
    }

    public static void main(String[] args){
        GestureApp window = new GestureApp();
    }
}
