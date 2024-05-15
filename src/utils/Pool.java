package utils ;
import java.util.ArrayList;
import java.util.List;

/**
 * For pooling objects that can be reused.
 */
public class Pool {
    /**
     * The objects list.
     */
    private List<Object> objects = new ArrayList<>();
    /**
     * The type of the objects.
     */
    private Class<?> type = Object.class;

    /**
     * Release an object after use.
     * @param args Objects to be released.
     * @return The Pool instance for chaining.
     */
    public Pool release(Object... args) {
        int Nargs = args.length;
        for (int i = 0; i < Nargs; i++) {
            this.objects.add(args[i]);
        }
        return this;
    }

    /**
     * Get an object.
     * @return The retrieved object.
     */
    public Object get() {
        if (this.objects.isEmpty()) {
            return this.constructObject();
        } else {
            return this.objects.remove(this.objects.size() - 1);
        }
    }

    /**
     * Construct an object. Should be implemented in each subclass.
     * @throws UnsupportedOperationException if not implemented in a subclass.
     */
    protected Object constructObject() {
        throw new UnsupportedOperationException("constructObject() not implemented in this Pool subclass yet!");
    }

    /**
     * Resize the pool to a specific size.
     * @param size The new size of the pool.
     * @return Self, for chaining.
     */
    public Pool resize(int size) {
        List<Object> objects = this.objects;

        while (objects.size() > size) {
            objects.remove(objects.size() - 1);
        }

        while (objects.size() < size) {
            objects.add(this.constructObject());
        }

        return this;
    }
}


