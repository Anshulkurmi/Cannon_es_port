package shapes;
import java.util.ArrayList;
import java.util.List;

import math.Vec3;
import shapes.ShapeTypes;

/**
 * Cylinder class.
 * @example
 *     double radiusTop = 0.5
 *     double radiusBottom = 0.5
 *     double height = 2
 *     int numSegments = 12
 *     Shape cylinderShape = new CANNON.Cylinder(radiusTop, radiusBottom, height, numSegments)
 *     Body cylinderBody = new CANNON.Body({ mass: 1, shape: cylinderShape })
 *     world.addBody(cylinderBody)
 */
public class Cylinder extends ConvexPolyhedron {

    // Properties
     /** The radius of the top of the Cylinder. */
    private double radiusTop;
    /** The radius of the bottom of the Cylinder. */
    private double radiusBottom;
    /** The height of the Cylinder. */
    private double height;
    /** The number of segments to build the cylinder out of. */
    private int numSegments;

    public Cylinder(){
        this(1,1,1,8);
    }
    /**
     * Cylinder class constructor.
     * @param radiusTop The radius of the top of the Cylinder.
     * @param radiusBottom The radius of the bottom of the Cylinder.
     * @param height The height of the Cylinder.
     * @param numSegments The number of segments to build the cylinder out of.
     */
    public Cylinder(double radiusTop, double radiusBottom, double height, int numSegments) {
    	
    	super();
    	
    	
        if (radiusTop < 0) {
            throw new IllegalArgumentException("The cylinder radiusTop cannot be negative.");
        }

        if (radiusBottom < 0) {
            throw new IllegalArgumentException("The cylinder radiusBottom cannot be negative.");
        }

        int N = numSegments;
        List<Vec3> vertices = new ArrayList<>();
        List<Vec3> axes = new ArrayList<>();
        List<Face> faces = new ArrayList<>();
        // List<Integer> bottomFace = new ArrayList<>();
        // List<Integer> topFace = new ArrayList<>();
        
        int[] bottomFace = new int[N];
        int[] topFace = new int[N];

        // Constants for cos and sin functions
        double pi2 = 2 * Math.PI;
        double halfHeight = height * 0.5;
        
        for (int i = 0; i < N; i++) {
            double theta = (pi2 / N) * (i + 1);
            double thetaN = (pi2 / N) * (i + 0.5);
            
            // Bottom
            vertices.add(new Vec3(-radiusBottom * Math.sin(theta), -halfHeight, radiusBottom * Math.cos(theta)));
            bottomFace[i] = (2 * i);
            
            // Top
            vertices.add(new Vec3(-radiusTop * Math.sin(theta), halfHeight, radiusTop * Math.cos(theta)));
            topFace[i] = (2 * i + 1);

            // Axis: we can cut off half of them if we have an even number of segments
            if (N % 2 == 1 || i < N / 2) {
                axes.add(new Vec3(-Math.sin(thetaN), 0, Math.cos(thetaN)));
            }
            
            // Faces
            if (i < N - 1) {
                // Face
                faces.add(new Face(new int[]{2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2}));
            } else {
                faces.add(new Face(new int[]{2 * i, 2 * i + 1, 1, 0})); // Connect
            }
        }
        
        // // First bottom point
        // vertices.add(new Vec3(-radiusBottom * Math.sin(0), -halfHeight, radiusBottom * Math.cos(0)));
        // bottomFace.add(2 * N);
        
        // // First top point
        // vertices.add(new Vec3(-radiusTop * Math.sin(0), halfHeight, radiusTop * Math.cos(0)));
        // topFace.add(2 * N + 1);

        faces.add(new Face(new int[]{2 * N, 2 * N + 1, 2 * N + 3, 2 * N + 2})); // Top face
        faces.add(new Face(bottomFace)); // Bottom face
        axes.add(new Vec3(0, 1, 0));
        
        this.vertices = vertices;
        this.faces = faces;
        computeNormals();
        updateBoundingSphereRadius();
        if (this.faceNormals.isEmpty()) {
            computeNormals();
        }

        this.type = ShapeTypes.CYLINDER;
        this.radiusTop = radiusTop;
        this.radiusBottom = radiusBottom;
        this.height = height;
        this.numSegments = numSegments;
    }
	public double getRadiusTop() {
		return radiusTop;
	}
	public double getRadiusBottom() {
		return radiusBottom;
	}
	public double getHeight() {
		return height;
	}
	public int getNumSegments() {
		return numSegments;
	}
}
