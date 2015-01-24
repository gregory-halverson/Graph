import java.io.IOException;
import java.util.ArrayList;

/**
 * Created by Gregory on 5/10/14.
 */
public class TestGraph
{
    public static void main(String[] args) throws IOException
    {
        Graph graph = Graph.readFromFile("input.txt");

        System.out.println(graph);
        System.out.println(graph.getWeight());

        Graph mst = graph.kruskalMST();

        System.out.println();
        System.out.println(mst);
        System.out.println(mst.getWeight());
    }
}
