package world;

import objects.Body;

public class AddBodyEvent {
    public String type;
    public Body body;


    public AddBodyEvent(String type, Body body) {
        this.type = type;
        this.body = body;
    }
}
