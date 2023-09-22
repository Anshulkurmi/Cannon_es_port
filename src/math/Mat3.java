package math;

/**
 * A 3x3 matrix.
 * Authored by {@link http://github.com/schteppe/ schteppe}
 */
public class Mat3 {
    /**
     * A vector of length 9, containing all matrix elements.
     */
    public double[] elements;
    
    /**
     * creates a 3x3 Matrix and storing the data in a 1D array of length 9 
     */
    public Mat3() {
   	 	this(new double[]{0,0,0,0,0,0,0,0,0}) ;
    }
    /**
     * @param elements A vector of length 9, containing all matrix elements.
     */
    public Mat3(double[] elements) {
        this.elements = elements;
    }

    /**
     * Sets the matrix to identity
     */
    public void identity() {
        elements[0] = 1;
        elements[1] = 0;
        elements[2] = 0;
        elements[3] = 0;
        elements[4] = 1;
        elements[5] = 0;
        elements[6] = 0;
        elements[7] = 0;
        elements[8] = 1;
    }

    /**
     * Set all elements to zero
     */
    public void setZero() {
        for (int i = 0; i < elements.length; i++) {
            elements[i] = 0;
        }
    }

    /**
     * Sets the matrix diagonal elements from a Vec3
     */
    public void setTrace(Vec3 vector) {
        elements[0] = vector.x;
        elements[4] = vector.y;
        elements[8] = vector.z;
    }
    
    public Vec3 getTrace() {
    	return getTrace(new Vec3());
    }
    /**
     * Gets the matrix diagonal elements
     * @param target : 3D vector to store the trace(Diagonal elements of the matrix)
     * @return target Vector 
     */
    public Vec3 getTrace(Vec3 target) {
        target.x = elements[0];
        target.y = elements[4];
        target.z = elements[8];
        return target;
    }
    
    public Vec3 vmult(Vec3 v) {
    	return vmult (v, new Vec3());
    }

    /**
   * Matrix-Vector multiplication
   * @param v The vector to multiply with
   * @param target Optional, target to save the result in.
   */
    public Vec3 vmult(Vec3 v, Vec3 target) {
        double x = v.x;
        double y = v.y;
        double z = v.z;
        target.x = elements[0] * x + elements[1] * y + elements[2] * z;
        target.y = elements[3] * x + elements[4] * y + elements[5] * z;
        target.z = elements[6] * x + elements[7] * y + elements[8] * z;

        return target;
    }

    /**
     * Matrix-scalar multiplication
     */
    public void smult(double s) {
        for (int i = 0; i < elements.length; i++) {
            elements[i] *= s;
        }
    }
    
    public Mat3 mmult(Mat3 matrix) {
    	return mmult(matrix , new Mat3());
    }

    /**
   * Matrix multiplication
   * @param matrix Matrix to multiply with from left side.
   */
    public Mat3 mmult(Mat3 matrix, Mat3 target) {
        double[] A = elements;
        double[] B = matrix.elements;
        double[] T = target.elements;

        double a11 = A[0], a12 = A[1], a13 = A[2],
                a21 = A[3], a22 = A[4], a23 = A[5],
                a31 = A[6], a32 = A[7], a33 = A[8];

        double b11 = B[0], b12 = B[1], b13 = B[2],
                b21 = B[3], b22 = B[4], b23 = B[5],
                b31 = B[6], b32 = B[7], b33 = B[8];

        T[0] = a11 * b11 + a12 * b21 + a13 * b31;
        T[1] = a11 * b12 + a12 * b22 + a13 * b32;
        T[2] = a11 * b13 + a12 * b23 + a13 * b33;

        T[3] = a21 * b11 + a22 * b21 + a23 * b31;
        T[4] = a21 * b12 + a22 * b22 + a23 * b32;
        T[5] = a21 * b13 + a22 * b23 + a23 * b33;

        T[6] = a31 * b11 + a32 * b21 + a33 * b31;
        T[7] = a31 * b12 + a32 * b22 + a33 * b32;
        T[8] = a31 * b13 + a32 * b23 + a33 * b33;

        return target;
    }

    /**
     * Scale each column of the matrix by Vector 
     * @param target : Stores the scaled Matrix 
     * @return target 
     */
    public Mat3 scale(Vec3 vector, Mat3 target) {
        double[] e = elements;
        double[] t = target.elements;

        for (int i = 0; i < 3; i++) {
            t[3 * i + 0] = vector.x * e[3 * i + 0];
            t[3 * i + 1] = vector.y * e[3 * i + 1];
            t[3 * i + 2] = vector.z * e[3 * i + 2];
        }

        return target;
    }
    
    /**
     * default case where no target given so , we created one
     * @param b
     * @return
     */
    public Vec3 solve(Vec3 b) {
    	return solve (b , new Vec3());
    }

    /**
     * Solve Ax=b , where A is 3x3 matrix and x,b are vectors using Gauss-elimination method 
     * Read more about Gauss-elimination method here ... {@link https://en.wikipedia.org/wiki/Gaussian_elimination
     * @param b : 3D Vector for which we have to calculate solution of x 
     * @param target : 3D Vector in which we store the solution of the equation Ax = b 
     * @return target 
     */
    public Vec3 solve(Vec3 b, Vec3 target) {
        // Construct equations
        int nr = 3; // num rows
        int nc = 4; // num cols
        double[] eqns = new double[nr * nc];
        int i, j;
        for (i = 0; i < nr * nc; i++) {
            eqns[i] = 0;
        }
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                eqns[i + nc * j] = elements[i + 3 * j];
            }
        }
        eqns[3 + 4 * 0] = b.x;
        eqns[3 + 4 * 1] = b.y;
        eqns[3 + 4 * 2] = b.z;

        // Compute right upper triangular version of the matrix - Gauss elimination
        int n = 3;

        int k = n;
        int np;
        int kp = 4; // num rows
        int p;
        do {
            i = k - n;
            if (eqns[i + nc * i] == 0) {
                // the pivot is null, swap lines
                for (j = i + 1; j < k; j++) {
                    if (eqns[i + nc * j] != 0) {
                        np = kp;
                        do {
                            // do line( i ) = line( i ) + line( k )
                            p = kp - np;
                            eqns[p + nc * i] += eqns[p + nc * j];
                        } while (--np > 0);//added --np > 0
                        break;
                    }
                }
            }
            if (eqns[i + nc * i] != 0) {
                for (j = i + 1; j < k; j++) {
                    double multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                    np = kp;
                    do {
                        // do line( k ) = line( k ) - multiplier * line( i )
                        p = kp - np;
                        eqns[p + nc * j] = p <= i ? 0 : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                    } while (--np >0);
                }
            }
        } while (--n > 0);

        // Get the solution
        target.z = eqns[2 * nc + 3] / eqns[2 * nc + 2];
        target.y = (eqns[1 * nc + 3] - eqns[1 * nc + 2] * target.z) / eqns[1 * nc + 1];
        target.x = (eqns[0 * nc + 3] - eqns[0 * nc + 2] * target.z - eqns[0 * nc + 1] * target.y) / eqns[0 * nc + 0];

        if (Double.isNaN(target.x) || Double.isNaN(target.y) || Double.isNaN(target.z) ||
            target.x == Double.POSITIVE_INFINITY || target.y == Double.POSITIVE_INFINITY || target.z == Double.POSITIVE_INFINITY) {
            throw new ArithmeticException("Could not solve equation!");
        }

        return target;
    }

    /**
     * Get an element in the matrix by index. Index starts at 0, not 1!!!
     * @param row : starts at 0
     * @param column : starts at 0
     * @return element value at [row,column]
     */
    public double e(int row, int column) {
        return elements[column + 3 * row];
    }
    
    /**
     * sets  element at specified index [row,column] 
     * @param row : starts at 0
     * @param column : starts at 0
     * @param value : Element value to set 
     * @return
     */
    public void e(int row , int column , double value) {
    		this.elements[column + 3 * row] = value ;	
    }
    
    /**
     * Copy another matrix into this matrix object.
     * @param matrix : matrix to be copied in this matrix 
     */
    public Mat3 copy(Mat3 matrix) {
        //added
        //System.arraycopy(matrix.elements, 0, elements, 0, matrix.elements.length);
        for (int i = 0; i < matrix.elements.length; i++) {
            this.elements[i] = matrix.elements[i] ;
          }
        return this;
    }

    /**
     * Returns a string representation of the matrix.
     */
    @Override
    public String toString() {
        StringBuilder r = new StringBuilder();
        String sep = ",";
        for (int i = 0; i < 9; i++) {
            r.append(elements[i]).append(sep);
        }
        return r.toString();
    }

    //default implementation
    public Mat3 reverse() {
    	return reverse (new Mat3());
    }
    /**
     * Reverse the matrix and store the reversed matrix in target matrix .
     * @param target : target matrix
     * @return target 
     */
    public Mat3 reverse(Mat3 target) {
        // Construct equations
        int nr = 3; // num rows
        int nc = 6; // num cols
        double[] eqns = reverse_eqns ;
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                eqns[i + nc * j] = elements[i + 3 * j];
            }
        }
        eqns[3 + 6 * 0] = 1;
        eqns[3 + 6 * 1] = 0;
        eqns[3 + 6 * 2] = 0;
        eqns[4 + 6 * 0] = 0;
        eqns[4 + 6 * 1] = 1;
        eqns[4 + 6 * 2] = 0;
        eqns[5 + 6 * 0] = 0;
        eqns[5 + 6 * 1] = 0;
        eqns[5 + 6 * 2] = 1;

        // Compute right upper triangular version of the matrix - Gauss elimination
        int n = 3;

        int k = n;
        int np;
        int kp = nc; // num rows
        int p;
        do {
            i = k - n;
            if (eqns[i + nc * i] == 0) {
                // the pivot is null, swap lines
                for (j = i + 1; j < k; j++) {
                    if (eqns[i + nc * j] != 0) {
                        np = kp;
                        do {
                            // do line( i ) = line( i ) + line( k )
                            p = kp - np;
                            eqns[p + nc * i] += eqns[p + nc * j];
                        } while (--np > 0);
                        break;
                    }
                }
            }
            if (eqns[i + nc * i] != 0) {
                for (j = i + 1; j < k; j++) {
                    double multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                    np = kp;
                    do {
                        // do line( k ) = line( k ) - multiplier * line( i )
                        p = kp - np;
                        eqns[p + nc * j] = p <= i ? 0 : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                    } while (--np > 0);
                }
            }
        } while (--n > 0);

        // Eliminate the upper left triangle of the matrix
        i = 2;
        do {
            j = i - 1;
            do {
                double multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                np = nc;
                do {
                    p = nc - np;
                    eqns[p + nc * j] = eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                } while (--np > 0);
            } while (j-- > 0);
        } while (i-- > 0);

        // Operations on the diagonal
        i = 2;
        do {
            double multiplier = 1 / eqns[i + nc * i];
            np = nc;
            do {
                p = nc - np;
                eqns[p + nc * i] = eqns[p + nc * i] * multiplier;
            } while (--np > 0);
        } while (i-- > 0);

        i = 2;
        do {
            j = 2;
            do {
                target.e(i, j, eqns[nr + j + nc * i]);
            } while (j-- > 0);
        } while (i-- > 0);

        return target;
    }

    /**
     * TODO
     * Set the matrix from a quaternion
     * @param q : Quaternion 
     */
    public Mat3 setRotationFromQuaternion(Quaternion q) {
        double x = q.x;
        double y = q.y;
        double z = q.z;
        double w = q.w;
        double x2 = x + x;
        double y2 = y + y;
        double z2 = z + z;
        double xx = x * x2;
        double xy = x * y2;
        double xz = x * z2;
        double yy = y * y2;
        double yz = y * z2;
        double zz = z * z2;
        double wx = w * x2;
        double wy = w * y2;
        double wz = w * z2;
        double[] e = elements;

        e[3 * 0 + 0] = 1 - (yy + zz);
        e[3 * 0 + 1] = xy - wz;
        e[3 * 0 + 2] = xz + wy;

        e[3 * 1 + 0] = xy + wz;
        e[3 * 1 + 1] = 1 - (xx + zz);
        e[3 * 1 + 2] = yz - wx;

        e[3 * 2 + 0] = xz - wy;
        e[3 * 2 + 1] = yz + wx;
        e[3 * 2 + 2] = 1 - (xx + yy);

        return this;
    }

    /**
     *  Transpose the matrix 
     *  interchanging rows and columns 
     *  @param target :  transpose of this matrix 
     *  @return target 
     */
    public Mat3 transpose(Mat3 target) {
        double[] M = elements;
        double[] T = target.elements;
        double tmp;

        // Set diagonals
        T[0] = M[0];
        T[4] = M[4];
        T[8] = M[8];

        tmp = M[1];
        T[1] = M[3];
        T[3] = tmp;

        tmp = M[2];
        T[2] = M[6];
        T[6] = tmp;

        tmp = M[5];
        T[5] = M[7];
        T[7] = tmp;

        return target;
    }

    static double[] reverse_eqns = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} ; 

}
