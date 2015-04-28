package objects;

import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Quad;
import com.jme3.scene.shape.Sphere;

/**
 * The spacecraft geometry used in intersection simulation.
 * @author Joshua Aurich
 */
public class Lander extends Node{
    //Height of the belly of the spacecraft
    public float height = 0.38f;
    //Locations of the feet on the ground
    public Node[] feetGroundLocation = new Node[4];
    //Locations of the leg connections to the spacecraft body. Used to determine ray between foot and body.
    public Node[] feetBodyLocation = new Node[4];
    //Geometry of the feet
    public Geometry[] feetGeom = new Geometry[4];
    //The node about which the spacecraft rotates
    public Node rotateNode = new Node();
    //The body geometry of the spacecraft
    public Sphere body = new Sphere(10,10,3.4f/2);
    //The mesh of the foot - maybe try changing this to a quad to decrease intersection times
    public Sphere footGeom = new Sphere(10,10,0.5f/2);
    //A plane located at the belly of the spacecraft, used for quick and dirty intersections due to minimal triangles
    public Quad bodyPlane = new Quad(3.4f,3.4f);
    //The geometry  of the body
    public Geometry bodyGeom;
    //The geometry of the body plane
    public Geometry bodyPlaneGeom;
    
    public Lander(){
        this.attachChild(rotateNode);
        for(int i = 0; i < 4; i++){
            feetGroundLocation[i] = new Node();
            feetBodyLocation[i] = new Node();
            //feetGeom[i] = new Geometry;
            rotateNode.attachChild(feetGroundLocation[i]);
            rotateNode.attachChild(feetBodyLocation[i]);
        }
        
        feetGroundLocation[0].setLocalTranslation(0,(3.4f/2)-0.25f,0.0001f);
        feetGroundLocation[1].setLocalTranslation(0,-(3.4f/2)+0.25f,0.0001f);
        feetGroundLocation[2].setLocalTranslation(-(3.4f/2)+0.25f,0,0.0001f);
        feetGroundLocation[3].setLocalTranslation((3.4f/2)-0.25f,0,0.0001f);
        feetBodyLocation[0].setLocalTranslation(0,(3.4f/2)-0.25f,height);
        feetBodyLocation[1].setLocalTranslation(0,-(3.4f/2)+0.25f,height);
        feetBodyLocation[2].setLocalTranslation(-(3.4f/2)+0.25f,0,height);
        feetBodyLocation[3].setLocalTranslation((3.4f/2)-0.25f,0,height);
        
        bodyGeom = new Geometry("Body", body);
        bodyGeom.setLocalScale(1, 1, 0.0001f);
        rotateNode.attachChild(bodyGeom);
        bodyGeom.setLocalTranslation(0, 0, height);
        
        bodyPlaneGeom = new Geometry("Body Plane", bodyPlane);
        bodyPlaneGeom.setLocalScale(1, 1, 0.0001f);
        rotateNode.attachChild(bodyPlaneGeom);
        bodyPlaneGeom.setLocalTranslation(-3.4f/2, -3.4f/2, height);
        
        feetGeom[0] = new Geometry("North foot", footGeom);
        feetGeom[0].setLocalScale(1, 1, 0.0001f);
        rotateNode.attachChild(feetGeom[0]);
        feetGeom[0].setLocalTranslation(feetGroundLocation[0].getLocalTranslation());
        
        feetGeom[1] = new Geometry("South foot", footGeom);
        feetGeom[1].setLocalScale(1, 1, 0.0001f);

        rotateNode.attachChild(feetGeom[1]);
        feetGeom[1].setLocalTranslation(feetGroundLocation[1].getLocalTranslation());
        
        feetGeom[2] = new Geometry("West foot", footGeom);
        feetGeom[2].setLocalScale(1, 1, 0.0001f);
        rotateNode.attachChild(feetGeom[2]);
        feetGeom[2].setLocalTranslation(feetGroundLocation[2].getLocalTranslation());
        
        feetGeom[3] = new Geometry("East foot", footGeom);
        feetGeom[3].setLocalScale(1, 1, 0.0001f);
        rotateNode.attachChild(feetGeom[3]);
        feetGeom[3].setLocalTranslation(feetGroundLocation[3].getLocalTranslation());       
    }
}
