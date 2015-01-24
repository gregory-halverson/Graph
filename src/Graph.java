// Gregory Halverson
// Pierce College
// CS 532
// Spring 2014

import java.io.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;

public class Graph<K>
{
    private boolean directed;
    private boolean weighted;
    private int numEdges = 0;
    private Vertex [] vertices;

    public class Vertex
    {
        private boolean directed;
        private int key; // Integer identifying this vertex
        private ArrayList<Edge> edges;

        // Initialize vertex object with integer key as directed or undirected
        Vertex(int key, boolean directed)
        {
            this.key = key;
            this.directed = directed;
            edges = new ArrayList<Edge>(key);
        }

        public boolean isDirected() { return directed; }

        // Check if this vertex is adjacent to another vertex
        public boolean isAdjacent(int other)
        {
            for (Edge e : edges)
            {
                if (isDirected())
                {
                    return e.getVertex2() == other;
                }
                else
                {
                    if (e.getVertex1() == key && e.getVertex2() == other)
                        return true;

                    if (e.getVertex1() == other && e.getVertex2() == key)
                        return true;
                }
            }

            return false;
        }

        public Edge getEdge(Vertex vertex)
        {
            return getEdge(vertex.getKey());
        }

        // Returns the edge object connecting this vertex with another vertex if it exists
        public Edge getEdge(int key)
        {
            for (Edge e : edges)
                if (e.getVertex1() == key || e.getVertex2() == key)
                    return e;

            return null;
        }

        // Returns the list of edges adjacent to this vertex
        public ArrayList<Edge> getEdges()
        {
            return edges;
        }

        public ArrayList<Vertex> getAdjacentVertices()
        {
            ArrayList<Vertex> adjacent = new ArrayList<Vertex>();

            for (Edge edge : edges)
            {
                if (edge.getVertex2() == key)
                    adjacent.add(vertices[edge.getVertex1()]);
                else
                    adjacent.add(vertices[edge.getVertex2()]);
            }

            return adjacent;
        }

        // Returns the integer identifying this vertex
        public int getKey()
        {
            return key;
        }

        // Adds an edge object to the list of adjacent edges for this vertex
        public void addEdge(Edge e)
        {
            edges.add(e);
        }

        // Removes edge from vertex
        public void removeEdge(int key)
        {
            for (Edge edge : edges)
                if (edge.containsVertex(key))
                    edges.remove(edge);
        }

        // Print vertex number and list of edges
        @Override
        public String toString()
        {
            StringWriter outputStream = new StringWriter();
            PrintWriter output = new PrintWriter(outputStream);

            output.print(key + ": ");

            for (int e = 0; e < edges.size(); e++)
            {
                int edge;

                if (key == edges.get(e).getVertex2())
                    edge = edges.get(e).getVertex1();
                else
                    edge = edges.get(e).getVertex2();

                output.print(edge);

                if (isWeighted())
                    output.print(" (" + edges.get(e).getWeight() + ")");

                if (e != edges.size() - 1)
                    output.print(", ");
            }

            return outputStream.toString();
        }
    }

    // Edge object connecting one vertex to another in a graph
    public class Edge
    {
        public static final int UNWEIGHTED = -1;

        boolean directed = false;
        private int vertex1;
        private int vertex2;
        private double weight;

        // Create an unweighted edge using the integers identifying two vertices in a graph
        Edge(int vertex1, int vertex2) {
            this.vertex1 = vertex1;
            this.vertex2 = vertex2;
            this.weight = UNWEIGHTED;
        }

        // Create a new edge between two vertices with a specified weight
        Edge(int vertex1, int vertex2, double weight) {
            this.vertex1 = vertex1;
            this.vertex2 = vertex2;
            this.weight = weight;
        }

        public boolean isDirected() {
            return directed;
        }

        public boolean isWeighted() {
            return weight != UNWEIGHTED;
        }

        public boolean containsVertex(int key) {
            return (vertex1 == key || vertex2 == key);
        }

        public int getVertex1() {
            return vertex1;
        }

        public int getVertex2() {
            return vertex2;
        }

        public double getWeight()
        {
            if (isWeighted())
                return weight;
            else
                return 1;
        }

        public void setDirected(boolean directed)
        {
            this.directed = directed;
        }

        public void setWeight(double weight)
        {
            this.weight = weight;
        }

        // Print edge data
        public String printData()
        {
            final int FIELD_SIZE = 10;

            StringWriter outputWriter = new StringWriter();
            PrintWriter output = new PrintWriter(outputWriter);

            output.println(StringFormat.padText("v1:", FIELD_SIZE, StringFormat.Alignment.left, ' ') + vertex1);
            output.println(StringFormat.padText("v2:", FIELD_SIZE, StringFormat.Alignment.left, ' ') + vertex2);
            output.println(StringFormat.padText("Directed:", FIELD_SIZE, StringFormat.Alignment.left, ' ') + isDirected());
            output.println(StringFormat.padText("Weight:", FIELD_SIZE, StringFormat.Alignment.left, ' ') + (isWeighted() ? getWeight() : "unweighted"));

            return outputWriter.toString();
        }

        @Override
        public String toString()
        {
            if (isWeighted())
                return "(" + vertex1 + ", " + vertex2 + ", " + weight + ")";
            else
                return "(" + vertex1 + ", " + vertex2 + ")";
        }
    }

    // Initialize graph with n vertices as directed or undirected and weighted or unweighted
    Graph(int n, boolean directed, boolean weighted)
    {
        vertices = new Graph.Vertex[n];
        this.directed = directed;
        this.weighted = weighted;
        numEdges = 0;

        for (int v = 0; v < getNumVertices(); v++)
            vertices[v] = new Vertex(v, directed);
    }

    public void addEdge(int vertex1, int vertex2)
    {
        if (isWeighted())
            return;

        addEdge(new Edge(vertex1, vertex2));
    }

    public void addEdge(int vertex1, int vertex2, double weight)
    {
        if (!isWeighted())
            return;

        addEdge(new Edge(vertex1, vertex2, weight));
    }

    // Add an edge object to connect two vertices in the graph
    private void addEdge(Edge e)
    {
        // Abort if either vertex specified is out of bounds
        if (    e.getVertex1() > getNumVertices() ||
                e.getVertex1() < 0 ||
                e.getVertex2() > getNumVertices() ||
                e.getVertex2() < 0)
            return;

        // Abort if equivalent edge already exists
        if (vertices[e.getVertex1()].isAdjacent(e.getVertex2()))
            return;

        // Pass directed flag from graph to edge
        e.setDirected(directed);

        // Initialize edge weight if necessary

        if (isWeighted() && !e.isWeighted())
            e.setWeight(1);

        if (!isWeighted() && e.isWeighted())
            e.setWeight(Edge.UNWEIGHTED);

        // Add edge to adjacency list of appropriate vertex or vertices

        vertices[e.getVertex1()].addEdge(e);

        if (!directed)
            vertices[e.getVertex2()].addEdge(e);

        // Increment count of edges
        numEdges++;
    }

    // Removes an edge from the graph
    public void removeEdge(int vertex1, int vertex2)
    {
        vertices[vertex1].removeEdge(vertex2);
        vertices[vertex2].removeEdge(vertex1);
    }

    // Returns all edges in graph
    public ArrayList<Edge> getEdges()
    {
        ArrayList<Edge> allEdges = new ArrayList<Edge>();

        for (Vertex vertex : vertices)
            for (Edge edge : vertex.getEdges())
                if (!allEdges.contains(edge))
                    allEdges.add(edge);

        return allEdges;
    }

    // Return list of adjacent edges for given vertex identifies by integer
    public ArrayList<Edge> getAdjacentEdges(int v)
    {
        return vertices[v].getEdges();
    }

    // Return list of vertices adjacent to given vertex
    public ArrayList<Integer> getAdjacentVertices(int v)
    {
        ArrayList<Integer> adjacentVertices = new ArrayList<Integer>();

        for (Edge e : getAdjacentEdges(v))
        {
            if (e.isDirected() || e.getVertex2() != v)
                adjacentVertices.add(e.getVertex2());
            else
                adjacentVertices.add(e.getVertex1());
        }

        return adjacentVertices;
    }

    // Return the edge object between two vertices if it exists
    public Edge getEdge(int v1, int v2)
    {
        if (v1 > getNumVertices() || v2 > getNumVertices())
            return null;

        return vertices[v1].getEdge(v2);
    }

    public int getNumVertices()
    {
        return vertices.length;
    }

    public int getNumEdges()
    {
        return numEdges;
    }

    public double getWeight()
    {
        double weight = 0;

        for (Edge edge : getEdges())
            weight += edge.getWeight();

        return weight;
    }

    public boolean isDirected()
    {
        return directed;
    }

    public boolean isWeighted()
    {
        return weighted;
    }

    public boolean isConnected()
    {
        if (isDirected())
            return false;

        for (Vertex vertex : vertices)
            if (vertex.getEdges().size() == 0)
                return false;

        return true;
    }

    public Boolean isCyclic()
    {
        UnionFindTree tree = new UnionFindTree(getNumVertices());

        if (isDirected())
            return null;

        for (Edge edge : getEdges())
        {
            if (tree.sameRoot(edge.getVertex1(), edge.getVertex2()))
                return true;

            tree.wunion(edge.getVertex1(), edge.getVertex2());
        }

        return false;
    }

    public Graph kruskalMST()
    {
        if (isDirected() || !isWeighted() || !isConnected())
            return null;

        ArrayList<Edge> edges = getEdges();

        PriorityQueue<Edge> queue = new PriorityQueue<Edge>();

        for (Edge edge : edges)
            queue.enqueue(edge, edge.getWeight());

        ArrayList<Edge> T = new ArrayList<Edge>();

        UnionFindTree tree = new UnionFindTree(getNumVertices());

        while (queue.size() > 0)
        {
            Edge edge = queue.dequeue();

            if (!tree.sameRoot(edge.getVertex1(), edge.getVertex2()))
            {
                T.add(edge);
                tree.wunion(edge.getVertex1(), edge.getVertex2());
            }
        }

        Graph mst = new Graph(getNumVertices(), isDirected(), isWeighted());

        if (T.size() < getNumVertices() - 1)
            return null;

        for (Edge edge : T)
            mst.addEdge(new Graph.Edge(edge.getVertex1(), edge.getVertex2(), edge.getWeight()));

        return mst;
    }

    public ArrayList<Integer> shortestPath(int start, int end)
    {
        final int INFINITE = -1;
        final int NONE = -1;

        ArrayList<Integer> path = new ArrayList<Integer>();
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
        double [] distance = new double[getNumVertices()];
        int [] predecessor = new int[getNumVertices()];
        boolean [] visited = new boolean[getNumVertices()];

        distance[start] = 0;
        predecessor[start] = NONE;

        for (Vertex vertex : vertices)
        {
            visited[vertex.getKey()] = false;

            if (vertex.getKey() == start)
                continue;

            if (vertex.isAdjacent(start))
            {
                distance[vertex.getKey()] = vertices[start].getEdge(vertex.getKey()).getWeight();
                predecessor[vertex.getKey()] = start;
                queue.enqueue(vertex, distance[vertex.getKey()]);
            }
            else
            {
                distance[vertex.getKey()] = INFINITE;
                predecessor[vertex.getKey()] = NONE;
            }
        }

        queue.enqueue(vertices[start], distance[start]);

        while (!queue.empty())
        {
            Vertex currentVertex = queue.dequeue();

            if (currentVertex.getKey() == end)
                break;

            if (visited[currentVertex.getKey()])
                continue;

            for (Vertex neighbor : currentVertex.getAdjacentVertices())
            {
                if (visited[neighbor.getKey()])
                    continue;

                double newDistance = distance[currentVertex.getKey()] + currentVertex.getEdge(neighbor).getWeight();

                if (newDistance < distance[neighbor.getKey()] || distance[neighbor.getKey()] == INFINITE)
                {
                    distance[neighbor.getKey()] = newDistance;
                    predecessor[neighbor.getKey()] = currentVertex.getKey();
                    queue.enqueue(neighbor, distance[neighbor.getKey()]);
                }
            }

            visited[currentVertex.getKey()] = true;
        }

        int vertex = end;

        while (vertex != NONE)
        {
            //System.out.println(vertex + " -> " + predecessor[vertex]);
            path.add(vertex);
            vertex = predecessor[vertex];
        }

        Collections.reverse(path);

        return path;
    }

    public static Graph readFromFile(String fileName) throws IOException
    {
        Graph graph = null;

        int size = 0;
        boolean directed = false;
        boolean weighted = false;

        BufferedReader input;

        try
        {
            input = new BufferedReader(new FileReader(fileName));
        }
        catch (IOException e)
        {
            throw e;
        }

        String line = "";
        String [] tokens;

        line = input.readLine();
        tokens = line.split("\\s");

        size = Integer.parseInt(tokens[0]);

        if (tokens.length < 2)
            directed = false;
        else if (tokens[1].equals("D"))
            directed = true;
        else if (tokens[1].equals("U"))
            directed = false;
        else
            throw new IOException();

        if (tokens.length < 3)
            weighted = true;
        else if (tokens[2].equals("W"))
            weighted = true;
        else if (tokens[2].equals("N"))
            weighted = false;
        else
            throw new IOException();

        graph = new Graph(size, directed, weighted);

        while ((line = input.readLine()) != null)
        {
            int v1, v2;
            double weight;

            tokens = line.split("\\s");

            if (weighted && tokens.length != 3)
                throw new IOException();

            if (!weighted && tokens.length != 2)
                throw new IOException();

            v1 = Integer.parseInt(tokens[0]);
            v2 = Integer.parseInt(tokens[1]);

            if (weighted)
            {
                weight = Double.parseDouble(tokens[2]);
                graph.addEdge(v1, v2, weight);
            }
            else
            {
                graph.addEdge(v1, v2);
            }
        }

        return graph;
    }

    // Print vertical list of vertices with horizontal lists of adjacent vertices (and weights)
    @Override
    public String toString()
    {
        StringWriter outputWriter = new StringWriter();
        PrintWriter output = new PrintWriter(outputWriter);

        for (Vertex v: vertices)
        {
            output.println(v);
        }

        return outputWriter.toString();
    }
}