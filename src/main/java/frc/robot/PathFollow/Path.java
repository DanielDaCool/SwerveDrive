package frc.robot.PathFollow;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.Chassis;
import static frc.robot.PathFollow.PathFollowConstants.*;

public class Path extends CommandBase{
    Chassis chassis;
    RoundedPoint[] corners;

    int segmentIndex;
    int pointsIndex;

    List<Segment> segments = new ArrayList<Segment>();

    boolean isRed;
    pathPoint[] points;
    Segment currentSegment;

    FollowSegment currentFollowSegment;

    public Path(pathPoint[] points){
        this.points = points;
        this.chassis = RobotContainer.robotContainer.chassis;
        segmentIndex = 0;
        pointsIndex = 0;
        addRequirements(chassis);
    }


    private void initPoints(){

        isRed = RobotContainer.robotContainer.isRed();
        // sets first point to chassis pose to prevent bugs with red and blue alliance
        points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(), points[1].getRotation(),
            points[0].getRadius(), chassis.getVelocity().getNorm());
    
        // case for red alliance (blue is the default)
        if (isRed) {
          points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
              Rotation2d.fromDegrees(180).minus(points[1].getRotation()), points[0].getRadius(), chassis.getVelocity().getNorm());
          for (int i = 1; i < points.length; i++) {
            points[i] = new pathPoint(convertAlliance(points[i].getX()), points[i].getY(),
                Rotation2d.fromDegrees(180).minus(points[i].getRotation()),
                points[i].getRadius(), points[i].getVelocity());
                
          }
        }
    }
    private void initCorners(){
        corners = new RoundedPoint[points.length - 2];
        for (int i = 0; i < points.length - 2; i++) {
          corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2]);
        }
    
    }
    
   
    private boolean isLastSegment(){
      return segmentIndex == segments.size() - 1;
    }
    private void initSegments(){
      if (points.length < 3) {

        segments.add(new Leg(points[0].getTranslation(), points[1].getTranslation()));
    
    }


  // case for more then 1 segment
  else {
    // creates the first leg
        segments.add(corners[0].getAtoCurveLeg());
      

      // creates arc than leg
      for (int i = 0; i < corners.length - 1; i++) {
        


        //creates arc
        segments.add(corners[i].getArc());

        //creates leg
        segments.add(new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart()));
        
        
      }
      // creates the last arc and leg
      segments.add(corners[corners.length - 1].getArc());
      segments.add(corners[corners.length - 1].getCtoCurveLeg());
      
    }
    for(int i = 0; i < segments.size(); i++){
      if(segments.get(i).getLength() < PATH_MIN_DISTANCE_SEGMENT) segments.remove(i);
    }
  }


    
    private void init(){
        initPoints();
        initCorners();
        initSegments();
    }

    private void updateCurrentSegment(){
        currentSegment = segments.get(segmentIndex);
    }
   
    @Override
    public void initialize(){
        init();
        currentFollowSegment = new FollowSegment(points[0].getVelocity(), points[1].getVelocity(),
         segments.get(0), points[1].getRotation());
        currentFollowSegment.schedule();
        currentSegment = segments.get(0);

    }
    @Override
    public void execute(){
      
        if(segments.size() == 1){
            return;
        }
        if(isInPoint(chassis.getPose().getTranslation(), points[pointsIndex].getTranslation())){
            pointsIndex++;
        }

        if(currentFollowSegment.isFinished()){
            segmentIndex++;
            updateCurrentSegment();
            currentFollowSegment = new FollowSegment(FIELD_LENGTH, FIELD_HEIGHT, currentSegment,
             points[pointsIndex].getRotation());
            currentFollowSegment.schedule();
        }
        
    }
    @Override
    public void end(boolean interrupted){
      chassis.setVelocity(new ChassisSpeeds());
  
    }
    @Override
    public boolean isFinished(){
      return isInPoint(chassis.getPose().getTranslation(), points[points.length - 1].getTranslation());
    }
    
}
