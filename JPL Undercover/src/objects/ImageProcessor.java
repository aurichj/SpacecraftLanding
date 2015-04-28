package objects;

import com.jme3.bounding.BoundingVolume;
import com.jme3.collision.CollisionResults;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.shape.Quad;
import java.util.ArrayList;
import java.util.concurrent.Callable;
import utils.Utility;
import utils.interp;

/**
 * This class is the backbone of the algorithm, and implements a multithreaded
 * approach to produce a solution.
 *
 * @author Joshua Aurich
 */
public class ImageProcessor implements Callable<int[][]> {
    //Which subthread we are in

    int child;
    //The dataset of images
    public Dataset vars;
    //The lander geometry to use in the simulation
    public Lander lander;
    //The terrain geometry
    public Geometry terrain;
    //The mask we use to see which areas of the dataset this child is responsible for
    int[][] localMask;
    //Currently unused boolean to specify if we are merging lidar and optical data - hence expanding the dataset
    //from 500x500 to 1000x1000
    boolean augmentLidar = false;
    //the distance between verticies in meters
    float step = (augmentLidar) ? 0.1f : 0.2f;
    //the size of the lidar data array
    int lidarSize = (augmentLidar) ? 1000 : 500;
    //divisor variable
    int div = (augmentLidar) ? 1 : 2;
    //starting location in image
    int startLoc = (augmentLidar) ? 21 : 10;
    //size of local neighborhood around lander
    int windowSize = (augmentLidar) ? 41 : 21;
    //the current coordinate in the algorithm which we are checking
    int xCoordinate = startLoc, yCoordinate = startLoc;

    /**
     * Class constructor
     *
     * @param dataset the dataset number which we are processing
     * @param mask the mask of locations this thread should process
     * @param num the child number of this process
     */
    public ImageProcessor(Dataset dataset, int[][] mask, int num) {
        child = num;
        vars = dataset;
        localMask = mask;
        if (augmentLidar) {
            float[][] augmentedLidar = lidarIntegrateOptical(vars.getImageData(), vars.getLidarData());
            vars.setLidarData(augmentedLidar);
        }
        lander = new Lander();
        terrain = getTerrain();
    }

    /**
     * The function which is executed to produce the solution of this callable
     *
     * @return int[][]
     */
    @Override
    public int[][] call() {
        return Utility.scale(produceSolution(localMask), augmentLidar);
    }

    /**
     * Returns the first iteration of the terrain geometry object.
     *
     * @return
     */
    public Geometry getTerrain() {
        //produce indicies to connect the vertecies of this mesh.
        int width = windowSize;
        float pixDist = step;
        float[] vertexBuffer = new float[width * width * 3];
        int[] indexBuffer = new int[(width - 1) * (width - 1) * 2 * 3];
        int triangleCount = 0;

        for (int y = 0; y < width; y++) {
            for (int x = 0; x < width; x++) {
                int baseCoord = ((width * y) + x) * 3;
                vertexBuffer[baseCoord] = (x - (width / 2)) * pixDist;
                vertexBuffer[baseCoord + 1] = (y - (width / 2)) * pixDist;
                vertexBuffer[baseCoord + 2] = vars.getLidarData()[xCoordinate + x - (width / 2)][yCoordinate + y - (width / 2)];

                if (x < width - 1 && y < width - 1) {
                    int indexBaseCoord = triangleCount * 3;
                    int vCoord = x + (y * width);
                    indexBuffer[indexBaseCoord] = vCoord;
                    indexBuffer[indexBaseCoord + 1] = (vCoord + 1);
                    indexBuffer[indexBaseCoord + 2] = (vCoord + 1 + width);
                    indexBuffer[indexBaseCoord + 3] = (vCoord);
                    indexBuffer[indexBaseCoord + 4] = (vCoord + width + 1);
                    indexBuffer[indexBaseCoord + 5] = (vCoord + width);
                    triangleCount += 2;
                }
            }
        }

        //replace the vertex and index buffers of a generic quad object to create the terrain mesh
        Quad q = new Quad();
        q.setDynamic();
        q.setBuffer(VertexBuffer.Type.Position, 3, vertexBuffer);
        q.setBuffer(VertexBuffer.Type.Normal, 3, new float[]{});
        q.setBuffer(VertexBuffer.Type.Index, 3, indexBuffer);
        q.setBuffer(VertexBuffer.Type.TexCoord, 2, new int[]{});
        q.updateCounts();
        q.updateBound();
        Geometry geom = new Geometry("Terrain", q);
        geom.updateGeometricState();
        geom.updateModelBound();
        return geom;
    }

    /**
     * Updates the terrain geometry once the coordinates we are simulating over
     * have changed by replacing the vertex buffer.
     */
    public void updateTerrain() {
        int width = windowSize;
        float pixDist = step;
        float[] vertexBuffer = new float[width * width * 3];

        for (int y = 0; y < width; y++) {
            for (int x = 0; x < width; x++) {
                int baseCoord = ((width * y) + x) * 3;
                vertexBuffer[baseCoord] = (x - (width / 2)) * pixDist;
                vertexBuffer[baseCoord + 1] = (y - (width / 2)) * pixDist;
                vertexBuffer[baseCoord + 2] = vars.getLidarData()[xCoordinate + x - (width / 2)][yCoordinate + y - (width / 2)];
            }
        }
        terrain.getMesh().setBuffer(VertexBuffer.Type.Position, 3, vertexBuffer);
        terrain.getMesh().createCollisionData();
        terrain.updateGeometricState();
        terrain.updateModelBound();
    }

    /**
     * Returns the max height difference over the current terrain neighborhood.
     *
     * @return
     */
    public float getMaxHeightDifferential(boolean circleCheck) {
        float max = -999999999f;
        float min = 999999999f;
        for (int y = 0; y < windowSize; y++) {
            for (int x = 0; x < windowSize; x++) {
                float temp = vars.getLidarData()[xCoordinate + x - (windowSize / 2)][yCoordinate + y - (windowSize / 2)];
                if (circleCheck) {
                    float xPos = (x - (windowSize / 2)) * step;
                    float yPos = (y - (windowSize / 2)) * step;
                    if (Math.sqrt((xPos * xPos) + (yPos * yPos)) <= 1.7f) {
                        if (max < temp) {
                            max = temp;
                        }
                        if (min > temp) {
                            min = temp;
                        }
                    }
                } else {
                    if (max < temp) {
                        max = temp;
                    }
                    if (min > temp) {
                        min = temp;
                    }
                }
            }
        }
        return max - min;
    }

    /**
     * Returns the [row,col] of the location of the max height. Used to feed
     * interpolation function.
     *
     * @return
     */
    public int[] getMaxHeightLocation() {
        int[] loc = new int[2];
        float max = -999999999f;

        for (int y = 0; y < windowSize; y++) {
            for (int x = 0; x < windowSize; x++) {
                float temp = vars.getLidarData()[xCoordinate + x - (windowSize / 2)][yCoordinate + y - (windowSize / 2)];
                if (max < temp) {
                    float xLoc = (x - (windowSize / 2)) * step;
                    float yLoc = (y - (windowSize / 2)) * step;
                    if (Math.sqrt((xLoc * xLoc) + (yLoc * yLoc)) <= 1.7f) {
                        max = temp;
                        loc[0] = (int) xLoc + xCoordinate;
                        loc[1] = (int) yLoc + yCoordinate;
                    }

                }
            }
        }
        return loc;
    }

    /**
     * Returns [{row},{col}] defined by loc = [x,y] of the highest value
     *
     * @param loc
     * @return
     */
    public double[][] getRowCol(int[] loc) {
        double[] row = new double[windowSize];
        double[] col = new double[windowSize];

        int y = loc[1];
        for (int x = 0; x < windowSize; x++) {
            row[x] = vars.getLidarData()[xCoordinate + x - (windowSize / 2)][y];
        }
        int x = loc[0];
        for (y = 0; y < windowSize; y++) {
            col[y] = vars.getLidarData()[x][yCoordinate + y - (windowSize / 2)];
        }
        return new double[][]{row, col};
    }

    public float getNewMax(double[] values) {
        float maxVal = -9999f;
        for (int i = 0; i < values.length; i++) {
            if (values[i] > maxVal) {
                if (Math.abs(i - (windowSize / 2)) * step <= 1.7f) {
                    maxVal = (float) values[i];
                }
            }
        }
        return maxVal;
    }

    /**
     * Checks to see if there is a height violation in the current terrain
     * neighborhood. This function is only called once we have determined that
     * the spacecraft is on reasonably flat ground. This allows to check the
     * height difference between the feet location and the highest spot on the
     * terrain under the lander to determine an intersection, rather than having
     * to perform a complex intersection.
     *
     * @param height
     * @return
     */
    public boolean getHeightViolation(float height) {
        for (int y = 0; y < windowSize; y++) {
            for (int x = 0; x < windowSize; x++) {
                if (vars.getLidarData()[xCoordinate + x - (windowSize / 2)][yCoordinate + y - (windowSize / 2)] - height >= 0.39f) {
                    float xDis = ((windowSize / 2) - x) * step;
                    float yDis = ((windowSize / 2) - y) * step;
                    if (Math.sqrt((xDis * xDis) + (yDis * yDis)) <= 1.7f) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * This is the main function that iterates over all of the prescribed
     * locations in the mask to generate a solution of safe and unsafe
     * locations.
     *
     * @param mask
     * @return the local result array
     */
    public int[][] produceSolution(int[][] mask) {
        int length = vars.getLidarData().length;
        int[][] solution = new int[length][length];

        //produce the boundary around the global solution
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < length; j++) {
                solution[i][j] = 0;
                solution[j][i] = 0;
                solution[(length) - 1 - i][j] = 0;
                solution[j][(length) - 1 - i] = 0;
            }
        }

        for (int y = startLoc; y < (length) - startLoc - 1; y++) {
            for (int x = startLoc; x < (length) - startLoc - 1; x++) {
                if (mask[x][y] == 255) {
                    xCoordinate = x;
                    yCoordinate = y;
                    if (getMaxHeightDifferential(false) < 0.30f) {
                        solution[x][y] = 255;
                    } else {
                        updateTerrain();
                        if (!intersects(lander, terrain, x, y)) {
                            solution[x][y] = 255;
                        }
                    }
                }
            }
        }
        //Utility.getMemoryUsage();
        return solution;
    }

    /**
     * The intersects function checks for an intersection of the terrain with
     * the belly of the spacecraft. This is accomplished by rotating the
     * spacecraft to different orientations, finding where the feet sit on the
     * terrain, accounting for impossible orientations (where the feet goes
     * under the terrain), further checking for angle violations, and performing
     * a geometric intersetion of the bounding volume of the body of the
     * spacecraft with the mesh of the terrain.
     *
     * @param lander
     * @param terrain
     * @param xx
     * @param yy
     * @return
     */
    public boolean intersects(Lander lander, Geometry terrain, int xx, int yy) {
        //Reset the lander to a default position
        lander.setLocalRotation(new Quaternion(0, 0, 0, 1));
        lander.rotateNode.rotate(new Quaternion(0, 0, 0, 1));
        lander.setLocalTranslation(0, 0, 0);

        //Number of subdivisons when rotating from 0 to 90 degrees
        float radialDivs = 7/**
                 * 8*
                 */
                ;

        //Here we find a list of potential normal vectors for the spacecraft and the associated rotation.
        //A normal vector is a combination of any 3 feet intersection locations with the terrain.
        float angle = (float) ((Math.PI / 2) / radialDivs);
        ArrayList<Vector3f> normals = new ArrayList<Vector3f>();
        ArrayList<Float> rotations = new ArrayList<Float>();
        ArrayList<Integer[]> relatedFootNum = new ArrayList<Integer[]>();
        ArrayList<Integer> nonUsedFoot = new ArrayList<Integer>();
        float currentAngle = 0;
        for (int i = 0; i < radialDivs; i++) {
            boolean[] foundCollision = new boolean[4];
            Vector3f[] terrainLocations = new Vector3f[4];
            int highestLocation = -1;
            for (int j = 0; j < 4; j++) {
                Vector3f footLoc = lander.feetGroundLocation[j].getWorldTranslation().add(0, 0, 200);
                Ray checkRay = new Ray(footLoc, new Vector3f(0, 0, -1));
                CollisionResults bodyResults = new CollisionResults();
                terrain.collideWith(checkRay, bodyResults);
                if (bodyResults.size() > 0) {
                    terrainLocations[j] = bodyResults.getCollision(0).getContactPoint();
                    foundCollision[j] = true;
                    if (highestLocation == -1) {
                        highestLocation = j;
                    } else if (terrainLocations[j].z > terrainLocations[highestLocation].z) {
                        highestLocation = j;
                    }
                }
            }

            Vector3f[] currentNorms = new Vector3f[4];
            //0,1,2
            boolean[] haveNormal = new boolean[4];
            if (foundCollision[0] && foundCollision[1] && foundCollision[2] && (highestLocation == 0 || highestLocation == 1 || highestLocation == 2)) {
                Vector3f v1 = (terrainLocations[0].subtract(terrainLocations[1])).normalize();
                Vector3f v2 = (terrainLocations[2].subtract(terrainLocations[1])).normalize();
                currentNorms[0] = (v1.cross(v2).add(0.000001f, 0.000001f, 0.000001f)).normalize();
                if (currentNorms[0].z < 0) {
                    currentNorms[0].negateLocal();
                }
                haveNormal[0] = true;
            }
            //1,2,3
            if (foundCollision[1] && foundCollision[2] && foundCollision[3] && (highestLocation == 3 || highestLocation == 1 || highestLocation == 2)) {
                Vector3f v1 = (terrainLocations[1].subtract(terrainLocations[2])).normalize();
                Vector3f v2 = (terrainLocations[3].subtract(terrainLocations[2])).normalize();
                currentNorms[1] = (v1.cross(v2).add(0.000001f, 0.000001f, 0.000001f)).normalize();
                if (currentNorms[1].z < 0) {
                    currentNorms[1].negateLocal();
                }
                haveNormal[1] = true;
            }
            //0,2,3
            if (foundCollision[0] && foundCollision[2] && foundCollision[3] && (highestLocation == 0 || highestLocation == 3 || highestLocation == 2)) {
                Vector3f v1 = (terrainLocations[2].subtract(terrainLocations[0])).normalize();
                Vector3f v2 = (terrainLocations[3].subtract(terrainLocations[0])).normalize();
                currentNorms[2] = (v1.cross(v2).add(0.000001f, 0.000001f, 0.000001f)).normalize();
                if (currentNorms[2].z < 0) {
                    currentNorms[2].negateLocal();
                }
                haveNormal[2] = true;
            }

            if (foundCollision[0] && foundCollision[1] && foundCollision[3] && (highestLocation == 0 || highestLocation == 3 || highestLocation == 1)) {
                Vector3f v1 = (terrainLocations[0].subtract(terrainLocations[1])).normalize();
                Vector3f v2 = (terrainLocations[3].subtract(terrainLocations[1])).normalize();
                currentNorms[3] = (v1.cross(v2).add(0.000001f, 0.000001f, 0.000001f)).normalize();
                if (currentNorms[3].z < 0) {
                    currentNorms[3].negateLocal();
                }
                haveNormal[3] = true;
            }

            float angleThreshold = 0;
            for (int j = 0; j < 4; j++) {
                if (normals.isEmpty()) {
                    if (haveNormal[j]) {
                        normals.add(currentNorms[j]);
                        rotations.add(currentAngle);
                        switch (j) {
                            case (0):
                                relatedFootNum.add(new Integer[]{0, 1, 2});
                                nonUsedFoot.add(3);
                                break;
                            case (1):
                                relatedFootNum.add(new Integer[]{1, 2, 3});
                                nonUsedFoot.add(0);
                                break;
                            case (2):
                                relatedFootNum.add(new Integer[]{0, 2, 3});
                                nonUsedFoot.add(1);
                                break;
                            case (3):
                                relatedFootNum.add(new Integer[]{0, 1, 3});
                                nonUsedFoot.add(2);
                                break;
                        }
                    }
                } else if (haveNormal[j]) {
                    boolean addVec = true;
                    for (int k = 0; k < normals.size(); k++) {
                        //check to see if we dont have duplicate normals
                        if (currentNorms[j].angleBetween(normals.get(k)) < angleThreshold) {
                            addVec = false;
                        }

                    }
                    if (addVec) {
                        normals.add(currentNorms[j]);
                        rotations.add(currentAngle);
                        switch (j) {
                            case (0):
                                relatedFootNum.add(new Integer[]{0, 1, 2});
                                nonUsedFoot.add(3);
                                break;
                            case (1):
                                relatedFootNum.add(new Integer[]{1, 2, 3});
                                nonUsedFoot.add(0);
                                break;
                            case (2):
                                relatedFootNum.add(new Integer[]{0, 2, 3});
                                nonUsedFoot.add(1);
                                break;
                            case (3):
                                relatedFootNum.add(new Integer[]{0, 1, 3});
                                nonUsedFoot.add(2);
                                break;
                        }
                    }
                }
            }
            if (radialDivs > 1) {
                lander.rotate(0, 0, angle);
                currentAngle += angle;
            }
        }
        lander.setLocalRotation(new Quaternion(0, 0, 0, 1));
        lander.rotateNode.rotate(new Quaternion(0, 0, 0, 1));

        //Now for every potential normal/rotation combination we found, we iterate through them
        for (int i = 0; i < normals.size(); i++) {
            //rotate the lander to sit in its new plane
            lander.setLocalTranslation(0, 0, 0);
            lander.rotateNode.rotate(0, 0, rotations.get(i));
            lander.lookAt(lander.getLocalTranslation().add(normals.get(i).mult(500)), new Vector3f(0, 0, 1));

            //adjust the spacecraft to sit ontop the terrain in its rotated position
            //assume rotation causes only minor displacement in foot position.
            Integer[] nums = relatedFootNum.get(i);
            CollisionResults dispResults = new CollisionResults();
            int loc = 0;
            for (int k = 0; (k < 3 && dispResults.size() == 0); k++) {
                Vector3f footLoc = lander.feetGroundLocation[nums[k]].getWorldTranslation().add(0, 0, 200);
                Ray checkRay = new Ray(footLoc, new Vector3f(0, 0, -1));
                terrain.collideWith(checkRay, dispResults);
                loc = k;
            }
            float displacement = 0;
            if (dispResults.size() > 0) {
                displacement = dispResults.getCollision(0).getContactPoint().z - lander.feetGroundLocation[nums[loc]].getWorldTranslation().z;
                lander.setLocalTranslation(lander.getLocalTranslation().add(0, 0, displacement - 0.001f));
            }

            //check to see if this is a valid orientation by intersecting the distance between the non-used foot with the terrain
            //if there is intersection, then this is not a valid orientation
            Ray footRay = new Ray(lander.feetGroundLocation[nonUsedFoot.get(i)].getWorldTranslation(), new Vector3f(0, 0, 1));
            CollisionResults footResults = new CollisionResults();
            terrain.collideWith(footRay, footResults);
            boolean valid = true;
            //if the foot intersects with the terrain, this is a bad orientation and we skip it
            if (footResults.size() > 0) {
                valid = false;
            }
            if (valid) {
                if (normals.get(i).angleBetween(new Vector3f(0, 0, 1)) >= 10f * Math.PI / 180.0f /**
                         * && normals.get(i).angleBetween(new Vector3f(0, 0, 1))
                         * <= 10.5f * Math.PI / 180.0f*
                         */
                        ) {
                    return true;
                }

                //Here we check for "flat" ground and use getHeightViolation() to determine intersection.
                if (normals.get(i).angleBetween(new Vector3f(0, 0, 1)) <= 1f * Math.PI / 180.0f) {
                    float height = (lander.feetGeom[relatedFootNum.get(i)[0]].getWorldTranslation().z + lander.feetGeom[relatedFootNum.get(i)[1]].getWorldTranslation().z + lander.feetGeom[relatedFootNum.get(i)[2]].getWorldTranslation().z) / 3.0f;
                    if (getHeightViolation(height)) {
                        return true;
                    } else if (getMaxHeightDifferential(true) > 0.38f) {
                        //lets use vishwas interpolation

                        int[] maxLoc = getMaxHeightLocation();
                        double[][] rowCol = getRowCol(maxLoc);
                        double[] newRow = interp.interp(rowCol[0]);
                        double[] newCol = interp.interp(rowCol[1]);
                        float max = Math.max(getNewMax(newCol), getNewMax(newRow));
                        if (max - height >= 0.39f) {
                            return true;
                        }

                    }
                } else { //otherwise we have to do the full intersection
                    BoundingVolume bv = lander.bodyPlaneGeom.getWorldBound();
                    Geometry bodyPlaneGeom = lander.bodyPlaneGeom;
                    CollisionResults boundingResults = new CollisionResults();
                    terrain.collideWith(bv, boundingResults);

                    if (boundingResults.size() > 0) {
                        for (float x = -1.6f; x <= 1.6f; x += step) {
                            for (float y = -1.6f; y <= 1.6f; y += step) {
                                if (Math.sqrt((x * x) + (y * y)) <= 1.7) {
                                    int xLoc = xCoordinate + (int) (x / step);
                                    int yLoc = yCoordinate + (int) (y / step);
                                    float zValue = vars.getLidarData()[xLoc][yLoc];

                                    Ray checkRay = new Ray(new Vector3f(lander.getWorldTranslation().x + x, lander.getWorldTranslation().y + y, 500), new Vector3f(0, 0, -1));
                                    CollisionResults bodyResults = new CollisionResults();
                                    bodyPlaneGeom.collideWith(checkRay, bodyResults);
                                    if (bodyResults.size() > 0) {
                                        if (bodyResults.getFarthestCollision().getContactPoint().z < zValue) {
                                            //we have found a case where the terrain is higher than the belly of the spacecraft
                                            return true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        //this location is safe!
        return false;
    }

    /**
     * A defunct function used in an attempt to mix optical and lidar data.
     * Note: this did not improve results. The concept was to use the brightness
     * of pixels which did not have a corresponding lidar sample point to
     * interpolate between the two neighbor lidar points. This increased the
     * lidar data size from 500x500 to 1000x1000 and yet reduced accuracy by 3%
     * in the final solution while increasing runtimes x4.
     *
     * @param opticalData
     * @param lidarData
     * @return
     */
    @Deprecated
    public float[][] lidarIntegrateOptical(int[][] opticalData, float[][] lidarData) {
        /**
         * Testing a way to merge optical and lidar data *
         */
        float[][] updatedLidar = new float[1000][1000];
        int[][] opticalToLidar = new int[1000][1000];
        for (float i = 1; i < 1000; i++) {
            for (float j = 1; j < 1000; j++) {
                if (i + 1 <= 999 && j + 1 <= 999 && i - 1 >= 0 && j - 1 >= 0) {
                    if (i % 2 == 0 && j % 2 != 0) {
                        //check y
                        float pixVal1 = vars.getImageData()[(int) i][(int) j - 1];
                        float pixVal2 = vars.getImageData()[(int) i][(int) j + 1];
                        float lidarVal1 = lidarData[(int) i / 2][(int) (j - 1) / 2];
                        float lidarVal2 = lidarData[(int) i / 2][(int) (j + 1) / 2];

                        float lamda = 0;
                        if ((pixVal2 - pixVal1) != 0) {
                            lamda = Math.abs(((float) ((vars.getImageData()[(int) i][(int) j] - pixVal1))) / (pixVal2 - pixVal1));
                        }
                        if (lamda > 1.8f) {
                            lamda = 0.5f;
                        }
                        //lamda = 0.5f;
                        float h = lidarVal1 + (lamda * (lidarVal2 - lidarVal1));
                        updatedLidar[(int) i][(int) j] = h;
                        if ((vars.getImageData()[(int) i][(int) j - 1] < vars.getImageData()[(int) i][(int) j] && vars.getImageData()[(int) i][(int) j + 1] < vars.getImageData()[(int) i][(int) j]) || (vars.getImageData()[(int) i][(int) j - 1] > vars.getImageData()[(int) i][(int) j] && vars.getImageData()[(int) i][(int) j + 1] > vars.getImageData()[(int) i][(int) j])) {
                            opticalToLidar[(int) i][(int) j] = 255;
                        } else {
                        }
                    } else if (i % 2 != 0 && j % 2 == 0) {
                        //check x
                        float pixVal1 = vars.getImageData()[(int) i - 1][(int) j];
                        float pixVal2 = vars.getImageData()[(int) i + 1][(int) j];
                        float lidarVal1 = lidarData[(int) (i - 1) / 2][(int) j / 2];
                        float lidarVal2 = lidarData[(int) (i + 1) / 2][(int) j / 2];
                        if ((vars.getImageData()[(int) i - 1][(int) j] < vars.getImageData()[(int) i][(int) j] && vars.getImageData()[(int) i + 1][(int) j] < vars.getImageData()[(int) i][(int) j]) || (vars.getImageData()[(int) i - 1][(int) j] > vars.getImageData()[(int) i][(int) j] && vars.getImageData()[(int) i + 1][(int) j] > vars.getImageData()[(int) i][(int) j])) {
                            opticalToLidar[(int) i][(int) j] = 255;
                        } else {
                        }

                        float lamda = 0;
                        if ((pixVal2 - pixVal1) != 0) {
                            lamda = Math.abs(((float) ((vars.getImageData()[(int) i][(int) j] - pixVal1))) / (pixVal2 - pixVal1));
                        }
                        if (lamda > 1f) {
                            lamda = 0.5f;
                        }
                        //lamda = 0.5f;
                        float h = lidarVal1 + (lamda * (lidarVal2 - lidarVal1));
                        updatedLidar[(int) i][(int) j] = h;
                        //

                    } else if (i % 2 != 0 && j % 2 != 0) {
                        //check y and x
                        if ((vars.getImageData()[(int) i + 1][(int) j - 1] < vars.getImageData()[(int) i][(int) j] && vars.getImageData()[(int) i - 1][(int) j + 1] < vars.getImageData()[(int) i][(int) j]) || (vars.getImageData()[(int) i + 1][(int) j + 1] < vars.getImageData()[(int) i][(int) j] && vars.getImageData()[(int) i - 1][(int) j - 1] < vars.getImageData()[(int) i][(int) j])) {
                            opticalToLidar[(int) i][(int) j] = 255;
                        }

                        float pixVal1a = vars.getImageData()[(int) i - 1][(int) j + 1];
                        float pixVal2a = vars.getImageData()[(int) i + 1][(int) j - 1];
                        float lidarVal1a = lidarData[(int) (i - 1) / 2][(int) (j + 1) / 2];
                        float lidarVal2a = lidarData[(int) (i + 1) / 2][(int) (j - 1) / 2];
                        float stepa = Math.abs((lidarVal1a - lidarVal2a) / (pixVal1a - pixVal2a));
                        float heighta = lidarVal1a + ((vars.getImageData()[(int) i][(int) j] - pixVal1a) * stepa);

                        float pixVal1b = vars.getImageData()[(int) i + 1][(int) j - 1];
                        float pixVal2b = vars.getImageData()[(int) i - 1][(int) j + 1];
                        float lidarVal1b = lidarData[(int) (i + 1) / 2][(int) (j - 1) / 2];
                        float lidarVal2b = lidarData[(int) (i - 1) / 2][(int) (j + 1) / 2];
                        float stepb = Math.abs((lidarVal1b - lidarVal2b) / (pixVal1b - pixVal2b));
                        float heightb = lidarVal1b + ((vars.getImageData()[(int) i][(int) j] - pixVal1b) * stepb);

                        float lamda1 = 0;
                        if ((pixVal2a - pixVal1a) != 0) {
                            lamda1 = Math.abs(((float) ((vars.getImageData()[(int) i][(int) j] - pixVal1a))) / (pixVal2a - pixVal1a));
                        }
                        if (lamda1 > 1.0f) {
                            lamda1 = 0.5f;
                        }
                        float lamda2 = 0;
                        if ((pixVal2b - pixVal1b) != 0) {
                            lamda2 = Math.abs(((float) ((vars.getImageData()[(int) i][(int) j] - pixVal1b))) / (pixVal2b - pixVal1b));
                        }
                        if (lamda2 > 1.0f) {
                            lamda2 = 0.5f;
                        }
                        //lamda1 = 0.5f;
                        //lamda2 = 0.5f;
                        float h = ((lidarVal1a + (lamda1 * (lidarVal2a - lidarVal1a))) + (lidarVal1b + (lamda2 * (lidarVal2b - lidarVal1b)))) / 2.0f;

                        updatedLidar[(int) i][(int) j] = h;
                    } else {
                        //this point has aleady been sampled
                        updatedLidar[(int) i][(int) j] = lidarData[(int) i / 2][(int) j / 2];
                    }
                }
            }
        }
        Utility.writeOutput(opticalToLidar, "testHeightFix_" + child + ".jpg");

        int[][] testPic = new int[1000][1000];
        for (int i = 0; i < 1000; i++) {
            for (int j = 0; j < 1000; j++) {
                if (updatedLidar[i][j] >= 0.39f) {
                    testPic[i][j] = 255;
                }
            }
        }
        Utility.writeOutput(testPic, "opticalToLidar_" + child + ".jpg");
        /**
         * *
         */
        return updatedLidar;
    }
}
