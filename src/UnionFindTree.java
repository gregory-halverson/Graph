// Gregory Halverson
// Pierce College
// CS 532
// Spring 2014

import java.io.PrintWriter;
import java.io.StringWriter;

// Represents a Union/Find Tree of given size using unsigned integers as keys
public class UnionFindTree
{
    // Member data
    private int parent[];
    private int size;

    // Constructor
    UnionFindTree(int size)
    {
    	// Initialize member data
        parent = new int[size];
        this.size = size;
        
        // Initialize parent array to root indicator/single node tree
        for (int i = 0; i < size; i++)
        {
            parent[i] = -1;
        }
    }

    // Check for root indicator
    boolean root(int key)
    {
        return (parent[key] < 0);
    }

    // Find parent of tree
    int pcfind(int key)
    {
        // Recursively traverse tree until root indicator is found
        if (root(key))
            return key;
        else
            return (parent[key] = pcfind(parent[key])); // Compress path by pointing node to the parent of its tree
    }

    // Compare parents of trees
    boolean sameRoot(int left, int right)
    {
        return (pcfind(left) == pcfind(right));
    }

    // Extract tree size from root indicator
    int treeSize(int key)
    {
        return Math.abs(parent[pcfind(key)]);
    }

    // Join two disjoint sets into the same tree
    void wunion(int left, int right)
    {
        // Check if the nodes are in two different trees
        if (!sameRoot(left, right))
        {
            // Compare and accumulate tree sizes
            if (treeSize(left) < treeSize(right))
            {
                // If the left tree is smaller than the right,
                // attach the left tree to the root of the right
                parent[pcfind(right)] -= treeSize(left);
                parent[pcfind(left)] = pcfind(right);
            }
            else
            {
                // If the left tree is larger than the right,
                // or they are the same size,
                // Attach the right tree to the root of the left
                parent[pcfind(left)] -= treeSize(right);
                parent[pcfind(right)] = pcfind(left);
            }
        }
    }

    // Output parent array to string horizontally
    String printArray()
    {
        StringBuilder output = new StringBuilder();

        output.append("[");

        // Iterate through array
        for (int i = 0; i < size; i++)
        {
            output.append(parent[i]);

            if (i != size - 1)
                output.append(", ");
        }

        output.append("]");

        return output.toString();
    }

    // Output parent array to string vertically
    String printVertical()
    {
        StringWriter outputString = new StringWriter();
        PrintWriter output = new PrintWriter(outputString);

        // Iterate through array
        for (int i = 0; i < size; i++)
        {
            output.println(i + ": " + parent[i]);
        }

        return outputString.toString();
    }
}