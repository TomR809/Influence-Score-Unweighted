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
        Node A = new Node('A');
        Node B = new Node('B');
        Node C = new Node('C');
        Node D = new Node('D');
        Node E = new Node('E');
        Node F = new Node('F');
        Node G = new Node('G');
        Node H = new Node('H');
        Node I = new Node('I');
        Node J = new Node('J');

        Node[] nodes = new Node[]{ A,B,C,D,E,F,G,H,I,J};

        int n = nodes.Length;

        // Initialize adjacency matrix
        int[,] adjacencyMatrix = new int[n, n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                adjacencyMatrix[i, j] = int.MaxValue; // Use int.MaxValue to represent no edge
            }
        }

        // Populate adjacency matrix with edge weights
        AddEdge(adjacencyMatrix, nodes, 'A', 'B', 1);
        AddEdge(adjacencyMatrix, nodes, 'A', 'C', 1);
        AddEdge(adjacencyMatrix, nodes, 'A', 'E', 5);
        AddEdge(adjacencyMatrix, nodes, 'B', 'C', 4);
        AddEdge(adjacencyMatrix, nodes, 'B', 'E', 1);
        AddEdge(adjacencyMatrix, nodes, 'B', 'G', 1);
        AddEdge(adjacencyMatrix, nodes, 'B', 'H', 1);
        AddEdge(adjacencyMatrix, nodes, 'C', 'D', 3);
        AddEdge(adjacencyMatrix, nodes, 'C', 'E', 1);
        AddEdge(adjacencyMatrix, nodes, 'D', 'E', 2);
        AddEdge(adjacencyMatrix, nodes, 'D', 'F', 1);
        AddEdge(adjacencyMatrix, nodes, 'D', 'G', 5);
        AddEdge(adjacencyMatrix, nodes, 'E', 'G', 2);
        AddEdge(adjacencyMatrix, nodes, 'F', 'G', 1);
        AddEdge(adjacencyMatrix, nodes, 'G', 'H', 2);
        AddEdge(adjacencyMatrix, nodes, 'H', 'I', 3);
        AddEdge(adjacencyMatrix, nodes, 'I', 'J', 3);

        // For each node, find all shortest paths to all other nodes
        foreach (Node source in nodes)
        {
            double total = 0;
            foreach(Node destination in nodes)
            {
                if (source != destination)
                {
                    // Reset all nodes between searches
                    foreach (Node node in nodes)
                    { 
                        node.Reset();
                    }
                    total += AStar(source, destination, adjacencyMatrix, nodes);
                }
            }
            // Influence score = numOfNodes -1 / totalCostOfAllShortestPaths 
            Console.WriteLine(source.name + " Influence Score: " + ((nodes.Count()-1)/ total));         
        }
    }

    private static void AddEdge(int[,] matrix, Node[] nodes, char start, char end, int weight)
    {
        int startIndex = Array.FindIndex(nodes, node => node.name == start);
        int endIndex = Array.FindIndex(nodes, node => node.name == end);

        matrix[startIndex, endIndex] = weight;
        matrix[endIndex, startIndex] = weight;
    }

    private static int AStar(Node source, Node destination, int[,] adjacencyMatrix, Node[] nodes)
    {
        // intialise open and closed list
        List<Node> queue = new List<Node>(new[] {source});
        List<Node> visited = new List<Node>();

        source.g = 0;
        source.f = source.g + calculateHeuristic(source, destination, adjacencyMatrix, nodes); // Calculate source f value

        while (queue.Count > 0)
        {
            Node currentNode = queue.OrderBy(node => node.f).First(); // Get node with lowest f value

            if (currentNode == destination)
            {
                // Outputs the full paths between nodes as well as its total cost
                return getPathDistance(currentNode, adjacencyMatrix, nodes);
            }
            
            // List node as visited
            queue.Remove(currentNode);
            visited.Add(currentNode);

            // Get all neighbour nodes
            foreach (Node neighbour in getNeighbours(currentNode, adjacencyMatrix, nodes)) 
            {
                // if neighbour has already been visited, continue to next neightbour
                if (visited.Contains(neighbour))
                {
                    continue;
                }
                // Calculate the neighbours g value from the path generated so far
                int tentativeG = currentNode.g + getDistance(currentNode, neighbour, adjacencyMatrix, nodes);
                    
                // If neighbour is not in the queue or tentative g value is less than orignial, update neighbour and path
                if(!queue.Contains(neighbour) || tentativeG < neighbour.g)
                {
                    neighbour.parent = currentNode;
                    neighbour.g = tentativeG;
                    neighbour.f = neighbour.g + calculateHeuristic(neighbour, destination, adjacencyMatrix, nodes);

                    if (!queue.Contains(neighbour))
                    {    
                        queue.Add(neighbour); // Add neighbour to queue
                    }
                }                     
            }
        }
        return int.MaxValue; // if no path is found, return infinity
    }

    // Use a breadth-first-search, smallest number of edges between nodes = heurstic value
    private static int calculateHeuristic(Node source, Node destination, int[,] adjacencyMatrix, Node[] nodes)
    {
        // if already at destination, return 0
        if (source == destination)
            return 0;

        Queue<Node> queue = new Queue<Node>();
        List<Node> visited = new List<Node>();
        int total = 0;

        queue.Enqueue(source);
        visited.Add(source);

        while (queue.Count > 0)
        {
            Node current = queue.Dequeue();
            total++;

            foreach (Node neighbor in getNeighbours(current, adjacencyMatrix, nodes))
            {
                if (!visited.Contains(neighbor))
                {
                    visited.Add(neighbor);

                    if (neighbor == destination)
                    {
                        return total;
                    }

                    queue.Enqueue(neighbor);
                }
            }
        }

        return int.MaxValue; // if no path is found to destination, return infinity
    }


    private static List<Node> getNeighbours(Node currentNode, int[,] matrix, Node[] nodes)
    {
        int index = Array.FindIndex(nodes, node => node == currentNode);
        List<Node> neighbours = new List<Node>();

        for (int i = 0; i < nodes.Length; i++)
        {
            if (matrix[index, i] != int.MaxValue)
            {
                neighbours.Add(nodes[i]);
            }
        }
        return neighbours;
    }

    // finds weight between two nodes
    private static int getDistance(Node a, Node b, int[,] matrix, Node[] nodes)
    {
        int index1 = Array.FindIndex(nodes, node => node == a);
        int index2 = Array.FindIndex(nodes, node => node == b);

        return matrix[index1, index2];
    }

    // Outputs the full paths between nodes as well as its total cost
    private static int getPathDistance(Node currentNode, int[,] matrix, Node[] nodes)
    {
        int totalCost = 0;
        while (currentNode.parent != null)
        {
            totalCost += getDistance(currentNode, currentNode.parent, matrix, nodes);
            currentNode = currentNode.parent;
        }
        return totalCost;
    } 
}

    

