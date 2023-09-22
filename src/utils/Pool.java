package utils ;

import java.util.ArrayList;

/**
 * For pooling objects that can be reused.
 * @param <T> The type of objects to pool.
 */
public class Pool<T> {
	/** pool of free available objects */
    protected ArrayList<T> objects = new ArrayList<T>();

    /**
     * Release one or more objects after use.
     * @param args The array of objects to release.
     * @return This Pool instance, for method chaining.
     */
    // public Pool<T> release(T[] args) {
    //     for (T obj : args) {
    //         objects.add(obj);
    //     }
    //     return this;
    // }
    //changed T[] to T 
    public  Pool<T> release(T args){
        objects.add(args);
        return this;
    }

    /**
     * Get an object from the pool.
     * @return An object from the pool.
     */
    public T get() {
        if (objects.isEmpty()) {
            return constructObject();
        } else {
            return objects.remove(objects.size() - 1);
        }
    }

    /**
     * Construct a new object. This method should be implemented in each subclass.
     * @return A new object.
     * @throws UnsupportedOperationException If this method is not implemented in the subclass.
     */
    protected T constructObject() {
        throw new UnsupportedOperationException("constructObject() not implemented in this Pool subclass yet!");
    }

    /**
     * Resize the pool to a specific size.
     * @param size The desired size of the pool.
     * @return This Pool instance, for method chaining.
     */
    public Pool<T> resize(int size) {
        while (objects.size() > size) {
            objects.remove(objects.size() - 1);
        }

        while (objects.size() < size) {
            objects.add(constructObject());
        }

        return this;
    }
}
