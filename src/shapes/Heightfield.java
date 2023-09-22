package shapes;

import java.awt.image.BufferedImage;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import collision.AABB;
import math.Quaternion;
import math.Vec3;


/**
 * Heightfield shape class. Height data is given as an array. These data points
 * are spread out evenly with a given distance.
 * 
 * @todo Should be possible to use along all axes, not just y
 * @todo should be possible to scale along all axes
 * @todo Refactor elementSize to elementSizeX and elementSizeY
 *
 * @example
 * actual creation of heightfield is given in the following link
 * https://github.com/pmndrs/cannon-es/blob/master/examples/heightfield.html
 *          
 * //you can laugh at the below code/joke
 * // Generate some height data (y-values).
 *          const data = []
 *          for (let i = 0; i < 1000; i++) {
 *          const y = 0.5 * Math.cos(0.2 * i)
 *          data.push(y)
 *          }
 *
 *          // Create the heightfield shape
 *          const heightfieldShape = new CANNON.Heightfield(data, {
 *          elementSize: 1 // Distance between the data points in X and Y
 *          directions
 *          })
 *          const heightfieldBody = new CANNON.Body({ shape: heightfieldShape })
 *          world.addBody(heightfieldBody)
 */
public class Heightfield extends Shape {
    /**
     * An array of numbers, or height values, that are spread out along the x axis.
     */
    public double[][] data;
    /**
     * Max value of the data points in the data array.
     */
    private double maxValue;
    /**
     * Minimum value of the data points in the data array.
     */
    private double minValue;
    /**
     * World spacing between the data points in X and Y direction.
     * 
     * @todo elementSizeX and Y
     * @default 1
     */
    public double elementSize;
    /**
     * @default true
     */
    private boolean cacheEnabled;
    public ConvexPolyhedron pillarConvex;
    public Vec3 pillarOffset;
    private final Map<String, HeightfieldPillar> _cachedPillars;

    public Heightfield(double[][] data) {
        this(data, null);
    }

    /**
     * @param data An array of numbers, or height values, that are spread out along
     *             the x axis.
     */
    public Heightfield(double[][] data, HeightfieldOptions options) {
        // changed commented out
        // options = Utils.defaults(options, new HeightfieldOptions());

        super(ShapeTypes.HEIGHTFIELD);

        this.data = data;
        if (options != null) {
            this.maxValue = options.maxValue;// options.maxValue != null ? options.maxValue : null;
            this.minValue = options.minValue; // options.minValue != null ? options.minValue : null;
            this.elementSize = options.elementSize; // options.elementSize != null ? options.elementSize : 1.0;

        } else {
            updateMinValue();
            updateMaxValue();
            this.elementSize=1;
        }

        this.cacheEnabled = true;
        this.pillarConvex = new ConvexPolyhedron();
        this.pillarOffset = new Vec3();
        updateBoundingSphereRadius();
        this._cachedPillars = new HashMap<>();
    }

    public void update() {
        this._cachedPillars.clear();
    }

    /**
     * Automatically find minimum height from the data
     */
    public void updateMinValue() {
        double minValue = data[0][0];
        for (int i = 0; i < data.length; i++) {
            for (int j = 0; j < data[i].length; j++) {
                double v = data[i][j];
                if (v < minValue) {
                    minValue = v;
                }
            }
        }
        this.minValue = minValue;
    }
 /**
     * Automatically find maximum height from the data
     */
    public void updateMaxValue() {
        double maxValue = data[0][0];
        for (int i = 0; i < data.length; i++) {
            for (int j = 0; j < data[i].length; j++) {
                double v = data[i][j];
                if (v > maxValue) {
                    maxValue = v;
                }
            }
        }
        this.maxValue = maxValue;
    }

    /**
   * Set the height value at an index. Don't forget to update maxValue and minValue after you're done.
   */
    public void setHeightValueAtIndex(int xi, int yi, double value) {
        data[xi][yi] = value;
        clearCachedConvexTrianglePillar(xi, yi, false);
        if (xi > 0) {
            clearCachedConvexTrianglePillar(xi - 1, yi, true);
            clearCachedConvexTrianglePillar(xi - 1, yi, false);
        }
        if (yi > 0) {
            clearCachedConvexTrianglePillar(xi, yi - 1, true);
            clearCachedConvexTrianglePillar(xi, yi - 1, false);
        }
        if (yi > 0 && xi > 0) {
            clearCachedConvexTrianglePillar(xi - 1, yi - 1, true);
        }
    }

    /**
   * Get max/min in a rectangle in the matrix data
   * @param result An array to store the results in.
   * @return The result array, if it was passed in. Minimum will be at position 0 and max at 1.
   */
    public void getRectMinMax(int iMinX, int iMinY, int iMaxX, int iMaxY, double[] result) {
        double max = minValue;
        for (int i = iMinX; i <= iMaxX; i++) {
            for (int j = iMinY; j <= iMaxY; j++) {
                double height = data[i][j];
                if (height > max) {
                    max = height;
                }
            }
        }
        result[0] = minValue;
        result[1] = max;
    }

     /**
   * Get the index of a local position on the heightfield. The indexes indicate the rectangles, so if your terrain is made of N x N height data points, you will have rectangle indexes ranging from 0 to N-1.
   * @param result Two-element array
   * @param clamp If the position should be clamped to the heightfield edge.
   */
    public boolean getIndexOfPosition(double x, double y, int[] result, boolean clamp) {
        double w = elementSize;
        int xi = (int) Math.floor(x / w);
        int yi = (int) Math.floor(y / w);
        result[0] = xi;
        result[1] = yi;
        if (clamp) {
            if (xi < 0) {
                xi = 0;
            }
            if (yi < 0) {
                yi = 0;
            }
            if (xi >= data.length - 1) {
                xi = data.length - 1;
            }
            if (yi >= data[0].length - 1) {
                yi = data[0].length - 1;
            }
        }
        return xi >= 0 && yi >= 0 && xi < data.length - 1 && yi < data[0].length - 1;
    }

    public boolean getTriangleAt(double x, double y, boolean edgeClamp, Vec3 a, Vec3 b, Vec3 c) {
        int[] idx = getHeightAt_idx;
        getIndexOfPosition(x, y, idx, edgeClamp);
        int xi = idx[0];
        int yi = idx[1];
        if (edgeClamp) {
            xi = Math.min(data.length - 2, Math.max(0, xi));
            yi = Math.min(data[0].length - 2, Math.max(0, yi));
        }
        double elementSize = this.elementSize;
        double lowerDist2 = Math.pow(x / elementSize - xi, 2) + Math.pow(y / elementSize - yi, 2);
        double upperDist2 = Math.pow(x / elementSize - (xi + 1), 2) + Math.pow(y / elementSize - (yi + 1), 2);
        boolean upper = lowerDist2 > upperDist2;
        getTriangle(xi, yi, upper, a, b, c);
        return upper;
    }

    public void getNormalAt(double x, double y, boolean edgeClamp, Vec3 result) {
        Vec3 a = getNormalAt_a;
        Vec3 b = getNormalAt_b;
        Vec3 c = getNormalAt_c;
        Vec3 e0 = getNormalAt_e0;
        Vec3 e1 = getNormalAt_e1;
        getTriangleAt(x, y, edgeClamp, a, b, c);
        b.vsub(a, e0);
        c.vsub(a, e1);
        e0.cross(e1, result);
        result.normalize();
    }

    /**
   * Get an AABB of a square in the heightfield
   * @param xi
   * @param yi
   * @param result
   */
    public void getAabbAtIndex(int xi, int yi, AABB result) {
        double elementSize = this.elementSize;
        result.lowerBound.set(xi * elementSize, yi * elementSize, data[xi][yi]);
        result.upperBound.set((xi + 1) * elementSize, (yi + 1) * elementSize, data[xi + 1][yi + 1]);
    }

    public double getHeightAt(double x, double y, boolean edgeClamp) {
        int[] idx = getHeightAt_idx;
        
        Vec3 a = getHeightAt_a;
        Vec3 b = getHeightAt_b;
        Vec3 c = getHeightAt_c;
        getIndexOfPosition(x, y, idx, edgeClamp);
        int xi = idx[0];
        int yi = idx[1];
        if (edgeClamp) {
            xi = Math.min(data.length - 2, Math.max(0, xi));
            yi = Math.min(data[0].length - 2, Math.max(0, yi));
        }
        boolean upper = getTriangleAt(x, y, edgeClamp, a, b, c);
        barycentricWeights(x, y, a.x, a.y, b.x, b.y, c.x, c.y, getHeightAt_weights);

        Vec3 weights = getHeightAt_weights;
        if (upper) {
            // Top triangle verts
            return data[xi + 1][yi + 1] * /*added weights.x*/weights.x+ data[xi][yi + 1] * weights.y + data[xi + 1][yi] * weights.z;
        } else {
            // Top triangle verts
            return data[xi][yi] * weights.x + data[xi + 1][yi] * weights.y+ data[xi][yi + 1] * weights.z;
        }
    }

    public String getCacheConvexTrianglePillarKey(int xi, int yi, boolean getUpperTriangle) {
        return xi + "_" + yi + "_" + (getUpperTriangle ? 1 : 0);
    }

    public HeightfieldPillar getCachedConvexTrianglePillar(int xi, int yi, boolean getUpperTriangle) {
        return this._cachedPillars.get(getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle));
    }

    public void setCachedConvexTrianglePillar(int xi, int yi, boolean getUpperTriangle, ConvexPolyhedron convex,
            Vec3 offset) {
        this._cachedPillars.put(getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle),
                new HeightfieldPillar(convex, offset));
    }

    public void clearCachedConvexTrianglePillar(int xi, int yi, boolean getUpperTriangle) {
        this._cachedPillars.remove(getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle));
    }

    /**
   * Get a triangle from the heightfield
   */
    public void getTriangle(int xi, int yi, boolean upper, Vec3 a, Vec3 b, Vec3 c) {
        double elementSize = this.elementSize;
        if (upper) {
            a.set((xi + 1) * elementSize, (yi + 1) * elementSize, data[xi + 1][yi + 1]);
            b.set(xi * elementSize, (yi + 1) * elementSize, data[xi][yi + 1]);
            c.set((xi + 1) * elementSize, yi * elementSize, data[xi + 1][yi]);
        } else {
            a.set(xi * elementSize, yi * elementSize, data[xi][yi]);
            b.set((xi + 1) * elementSize, yi * elementSize, data[xi + 1][yi]);
            c.set(xi * elementSize, (yi + 1) * elementSize, data[xi][yi + 1]);
        }
    }

    /**
   * Get a triangle in the terrain in the form of a triangular convex shape.
   */
    public void getConvexTrianglePillar(int xi, int yi, boolean getUpperTriangle) {
        ConvexPolyhedron result = this.pillarConvex;
        Vec3 offsetResult = this.pillarOffset;

        if (this.cacheEnabled) {
            HeightfieldPillar data = getCachedConvexTrianglePillar(xi, yi, getUpperTriangle);
            if (data != null) {
                this.pillarConvex = data.convex;
                this.pillarOffset = data.offset;
                return;
            }

            result = new ConvexPolyhedron();
            offsetResult = new Vec3();

            this.pillarConvex = result;
            this.pillarOffset = offsetResult;
        }

        double elementSize = this.elementSize;
        // changed List<face> to List<int[]>
        List<int[]> faces = result.faces;

        // Reuse verts if possible
        while (result.vertices.size() < 6) {
            result.vertices.add(new Vec3());
        }

        // Reuse faces if possible
        while (faces.size() < 5) {
            //added : length of face array = 4
            faces.add(new int[4]);
        }

        List<Vec3> verts = result.vertices;

        // added More Math.min
        double h = (Math.min(Math.min(data[xi][yi], data[xi + 1][yi]), Math.min(data[xi][yi + 1], data[xi + 1][yi + 1]))
                - this.minValue)
                / 2 + this.minValue;

        if (!getUpperTriangle) {
            offsetResult.set((xi + 0.25) * elementSize, (yi + 0.25) * elementSize, h);

            verts.get(0).set(-0.25 * elementSize, -0.25 * elementSize, data[xi][yi] - h);
            verts.get(1).set(0.75 * elementSize, -0.25 * elementSize, data[xi + 1][yi] - h);
            verts.get(2).set(-0.25 * elementSize, 0.75 * elementSize, data[xi][yi + 1] - h);
            verts.get(3).set(-0.25 * elementSize, -0.25 * elementSize, -Math.abs(h) - 1);
            verts.get(4).set(0.75 * elementSize, -0.25 * elementSize, -Math.abs(h) - 1);
            verts.get(5).set(-0.25 * elementSize, 0.75 * elementSize, -Math.abs(h) - 1);

            //faces.get(0).length = 0 ;//();
            faces.set(0,new int[]{0,1,2});
            // faces.get(0).add(0);
            // faces.get(0).add(1);
            // faces.get(0).add(2);

            // faces.get(1).clear();
            // faces.get(1).add(5);
            // faces.get(1).add(4);
            // faces.get(1).add(3);
            faces.set(1,new int[]{5,4,3});

            // faces.get(2).clear();
            // faces.get(2).add(0);
            // faces.get(2).add(2);
            // faces.get(2).add(5);
            // faces.get(2).add(3);
            faces.set(2,new int[]{0,2,5,3});

            // faces.get(3).clear();
            // faces.get(3).add(1);
            // faces.get(3).add(0);
            // faces.get(3).add(3);
            // faces.get(3).add(4);
            faces.set(3, new int[]{1,0,3,4});

            // faces.get(4).clear();
            // faces.get(4).add(4);
            // faces.get(4).add(5);
            // faces.get(4).add(2);
            // faces.get(4).add(1);
            faces.set(4,new int[]{4,5,2,1});
        } else {
            offsetResult.set((xi + 0.75) * elementSize, (yi + 0.75) * elementSize, h);

            verts.get(0).set(0.25 * elementSize, 0.25 * elementSize, data[xi + 1][yi + 1] - h);
            verts.get(1).set(-0.75 * elementSize, 0.25 * elementSize, data[xi][yi + 1] - h);
            verts.get(2).set(0.25 * elementSize, -0.75 * elementSize, data[xi + 1][yi] - h);
            verts.get(3).set(0.25 * elementSize, 0.25 * elementSize, -Math.abs(h) - 1);
            verts.get(4).set(-0.75 * elementSize, 0.25 * elementSize, -Math.abs(h) - 1);
            verts.get(5).set(0.25 * elementSize, -0.75 * elementSize, -Math.abs(h) - 1);

            // faces.get(0).clear();
            // faces.get(0).add(0);
            // faces.get(0).add(1);
            // faces.get(0).add(2);
            faces.set(0,new int[]{0,1,2});

            // faces.get(1).clear();
            // faces.get(1).add(5);
            // faces.get(1).add(4);
            // faces.get(1).add(3);
            faces.set(1,new int[]{5,4,3});

            // faces.get(2).clear();
            // faces.get(2).add(2);
            // faces.get(2).add(5);
            // faces.get(2).add(3);
            // faces.get(2).add(0);
            faces.set(2,new int[]{2,5,3,0});

            // faces.get(3).clear();
            // faces.get(3).add(3);
            // faces.get(3).add(4);
            // faces.get(3).add(1);
            // faces.get(3).add(0);
            faces.set(3,new int[]{3,4,1,0});

            // faces.get(4).clear();
            // faces.get(4).add(1);
            // faces.get(4).add(4);
            // faces.get(4).add(5);
            // faces.get(4).add(2);
            faces.set(4,new int[]{1,4,5,2});
        }

        result.computeNormals();
        result.computeEdges();
        result.updateBoundingSphereRadius();

        setCachedConvexTrianglePillar(xi, yi, getUpperTriangle, result, offsetResult);
    }

    public Vec3 calculateLocalInertia(double mass, Vec3 target) {
        target.set(0, 0, 0);
        return target;
    }

    public double volume() {
        return Double.MAX_VALUE; // The terrain is infinite
    }

    public void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
        // Do it properly
        min.set(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
        max.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public void updateBoundingSphereRadius() {
        // Use the bounding box of the min/max values
        double s = this.elementSize;
        this.boundingSphereRadius = new Vec3(data.length * s, data[0].length * s,
                Math.max(Math.abs(this.maxValue), Math.abs(this.minValue))).length();
    }

    /**
     * Sets the height values from an image. Currently only supported in browser.
     */
    public void setHeightsFromImage(BufferedImage image, Vec3 scale) {
        double x = scale.x;
        double z = scale.z;
        double y = scale.y;
        int width = image.getWidth();
        int height = image.getHeight();
        double[][] matrix = this.data;
        matrix = new double[height][width];
        this.elementSize = Math.abs(x) / width;
        for (int i = 0; i < height; i++) {
            double[] row = new double[width];
            for (int j = 0; j < width; j++) {
                int rgb = image.getRGB(j, i);
                int red = (rgb >> 16) & 0xFF;
                int green = (rgb >> 8) & 0xFF;
                int blue = rgb & 0xFF;
                double heightValue = ((red + green + blue) / 4 / 255.0) * z;
                if (x < 0) {
                    row[j] = heightValue;
                } else {
                    row[width - 1 - j] = heightValue;
                }
            }
            if (y < 0) {
                matrix[i] = row;
            } else {
                matrix[height - 1 - i] = row;
            }
        }
        this.updateMaxValue();
        this.updateMinValue();
        this.update();
    }

    public void barycentricWeights(double x, double y, double ax, double ay, double bx, double by, double cx, double cy,
            Vec3 result) {
        result.x = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / ((by - cy) * (ax - cx) + (cx - bx) * (ay - cy));
        result.y = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / ((by - cy) * (ax - cx) + (cx - bx) * (ay - cy));
        result.z = 1 - result.x - result.y;
    }

    static Vec3 getNormalAt_a = new Vec3();
    static Vec3 getNormalAt_b = new Vec3();
    static Vec3 getNormalAt_c = new Vec3();
    static Vec3 getNormalAt_e0 = new Vec3();
    static Vec3 getNormalAt_e1 = new Vec3();

    static Vec3 getHeightAt_a = new Vec3();
    static Vec3 getHeightAt_b = new Vec3();
    static Vec3 getHeightAt_c = new Vec3();
    static int[] getHeightAt_idx = new int[2];//added : size of array 2 
    static Vec3 getHeightAt_weights = new Vec3();
}
