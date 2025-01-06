using System;
using System.Collections;
using System.ComponentModel;
using System.Linq;
using System.Xml.Linq;

class Program
{
    static void Main(string[] args)
    { 
        // Intialise nodes
        Node A = new Node("Alicia");
        Node B = new Node("Britney");
        Node C = new Node("Claire");
        Node D = new Node("Diana");
        Node E = new Node("Edward");
        Node F = new Node("Fred");
        Node G = new Node("Gloria");
        Node H = new Node("Harry");

        Node[] nodes = new Node[] { A, B, C, D, E, F, G, H};

        //Initialise edges
        Edge[] edges = new Edge[]
        {
            new Edge(A, B),
            new Edge(B, C),
            new Edge(C, D),
            new Edge(D, E),
            new Edge(D, H),
            new Edge(E, H),
            new Edge(E, G),
            new Edge(E, F),
            new Edge(G, F),
            new Edge(H, G),
        };
        // Build adjacency list
        Dictionary<Node, List<Node>> adjacencyList = BuildAdjacencyList(nodes, edges);

        // For each node, find all shortest paths to all other nodes
        foreach (Node source in nodes)
        {
            Dictionary<Node, int> distances = BFS(source, adjacencyList);
            double total = 0;
            foreach(Node node in nodes) 
            {
                total += distances[node];
            
            }
            // Influence score = numOfNodes -1 / totalCostOfAllShortestPaths 
            Console.WriteLine(source.name + " Influence Score: " + ((nodes.Count() - 1) / total));
        } 
    }

    // Take edges and nodes and build an adjacency list
    private static Dictionary<Node, List<Node>> BuildAdjacencyList(Node[] nodes, Edge[] edges)
    {
        Dictionary<Node, List<Node>> adjacencyList = new Dictionary<Node, List<Node>>();
        foreach (Node node in nodes)
        {
            adjacencyList[node] = new List<Node>();
        }

        foreach (Edge edge in edges)
        {
            adjacencyList[edge.startNode].Add(edge.endNode);
            adjacencyList[edge.endNode].Add(edge.startNode);
        }
        return adjacencyList;
    }

    private static Dictionary<Node,int> BFS(Node source, Dictionary<Node, List<Node>> adjacencyList)
    {
        Queue<Node> queue = new Queue<Node>();
        List<Node> visited = new List<Node>();

        //Dictonary to store number of jumps to each node
        Dictionary<Node, int> distances = new Dictionary<Node, int> { { source, 0 } };

        queue.Enqueue(source);
        visited.Add(source);

        while (queue.Count > 0)
        {
            Node current = queue.Dequeue();

            // check all neighbours of each node if not already visited
            foreach (Node neighbor in adjacencyList[current])
            {
                if (!visited.Contains(neighbor))
                {
                    visited.Add(neighbor);
                    distances[neighbor] = distances[current] + 1;
                    queue.Enqueue(neighbor);
                }
            }
        }
        return distances; 
    }
}