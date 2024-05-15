package shapes;

import java.util.ArrayList;

public class Face {
public int[] vertices;
protected ArrayList<Integer> connectedFaces;

public Face(int[] vertices) {
	this.vertices = vertices  ;
}

public int indexOf(int i) {
	int index=0;
	for(int j:vertices) {
		index++;
		if(j==i)return index;
	}
	
	return -1;
}
}
