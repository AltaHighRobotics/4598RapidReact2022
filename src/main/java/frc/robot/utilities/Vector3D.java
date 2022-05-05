package frc.robot.utilities;

/** An object that contains an x, y, and z value.
 * Each instance also contains various functions for performing standard math operations on itself.
 */
public class Vector3D {

    public double x;
    public double y;
    public double z;
    public double direction;
    public double magnitude;

    /** Makes a vector3D object with the specified values.
     * 
     * @param x A double representing the initial x value.
     * @param y A double representing the initial y value.
     * @param z A double representing the initial z value.
     */
    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /** Copies the value of an existing vector into the parent vector.
     * 
     * @param a The vector to be copied.
     */
    public void copy(Vector3D a) {
        this.x = a.x;
        this.y = a.y;
        this.z = a.z;
    }

    /** Sets an existing vector to the specified values.
     * 
     * @param x A double representing the new x value.
     * @param y A double representing the new y value.
     * @param z A double representing the new z value.
     */
    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /** Creates a clone of the vector
     * 
     * @return An exact copy of the initial vector.
     */
    public Vector3D clone() {
        return new Vector3D(this.x, this.y, this.z);
    }

    /** Gets the result of adding another vector to the parent vector. This function does NOT modify the parent vector.
     * 
     * @param a The vector to add to the parent.
     * @return A new vector3D containing the result of the addition.
     */
    public Vector3D getAddition(Vector3D a) {
        return new Vector3D(this.x + a.x, this.y + a.y, this.z + a.z);
    }

    /** Gets the result of subtracting another vector from the parent vector. This function does NOT modify the parent vector.
     * 
     * @param a The vector to subtract from the parent.
     * @return A new vector3D containing the result of the subtraction.
     */
    public Vector3D getSubtraction(Vector3D a) {
        return new Vector3D(this.x - a.x, this.y - a.y, this.z - a.z);
    }

    /** Gets the result of multiplying the parent vector by a number. This function does NOT modify the parent vector.
     * 
     * @param scalar A double that will be used to multiply each component of the parent vector
     * @return A new vector3D containing the result of the multiplication.
     */
    public Vector3D getMultiplication(double scalar) {
        return new Vector3D(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    /** Gets the result of dividing the parent vector by a number. This function does NOT modify the parent vector.
     * 
     * @param scalar A double that will be used to divide each component of the parent vector.
     * @return A new vector3D containing the result of the division.
     */
    public Vector3D getDivision(double scalar) {
        return new Vector3D(this.x / scalar, this.y / scalar, this.z / scalar);
    }

    /** Gets the result of normalizing the parent vector to a length of 1. This function does NOT modify the parent vector. Calling this function also updates the parent vector's magnitude.
     * 
     * @return A new vector3D with the same direction as the parent, but with a length of 1.
     */
    public Vector3D getNormalization() {
        return this.getDivision(this.magnitude());
    }

    /** Adds another vector to the parent vector. This function DOES modify the parent vector.
     * 
     * @param a The vector to add to the parent.
     */
    public void add(Vector3D a) {
        this.x += a.x;
        this.y += a.y;
        this.z += a.z;
    }

    /** Subtracts another vector to the parent vector. This function DOES modify the parent vector.
     * 
     * @param a The vector to subtract from the parent.
     */
    public void subtract(Vector3D a) {
        this.x -= a.x;
        this.y -= a.y;
        this.z -= a.z;
    }

    /** Multiplies the parent vector by a number. This function DOES modify the parent vector.
     * 
     * @param scalar A double that will be used to multiply each component of the parent vector.
     */
    public void multiply(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
    }

    /** Divides the parent vector by a number. This function DOES modify the parent vector.
     * 
     * @param scalar A double that will be used to divide each component of the parent vector.
     */
    public void divide(double scalar) {
        this.x /= scalar;
        this.y /= scalar;
        this.z /= scalar;
    }

    /** Normalizes the parent vector to a length of 1. This function DOES modify the parent vector. Calling this function also updates the parent vector's magnitude.
     */
    public void normalize() {
        this.divide(this.magnitude());
    }

    /** Calculates magnitude of the parent vector.
     * 
     * @return A double representing the magnitude of the parent vector. This value is also stored in the parent to avoid extra calculations when possible
     */
    public double magnitude() {
        this.magnitude = Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2) + Math.pow(this.z, 2));
        return this.magnitude;
    }
}
