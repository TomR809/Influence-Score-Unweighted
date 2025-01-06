using System;
using System.Collections;
using System.ComponentModel;
using System.Linq;

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

        //Initialise edges
        Edge[] edges = new Edge[]
        {
            new Edge(A, B, 1),
            new Edge(A, C, 1),
            new Edge(A, E, 5),
            new Edge(B, C, 4),
            new Edge(B, E, 1),
            new Edge(B, G, 1),
            new Edge(B, H, 1),
            new Edge(C, D, 3),
            new Edge(C, E, 1),
            new Edge(D, E, 2),
            new Edge(D, F, 1),
            new Edge(D, G, 5),
            new Edge(E, G, 2),
            new Edge(F, G, 1),
            new Edge(G, H, 2),
            new Edge(H, I, 3),
            new Edge(I, J, 3),    
        };

        // For each node, find all shortest paths to all other nodes
        foreach(Node source in nodes)
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
                    total += AStar(source, destination, edges);
                }
            }
            // Influence score = numOfNodes -1 / totalCostOfAllShortestPaths 
            Console.WriteLine(source.name + " Influence Score: " + ((nodes.Count()-1)/ total));         
        }
    }

    private static int AStar(Node source, Node destination, Edge[] edges)
    {
        // intialise open and closed list
        List<Node> queue = new List<Node>(new[] {source});
        List<Node> visited = new List<Node>();

        source.g = 0;
        source.f = source.g + calculateHeuristic(source, destination, edges); // Calculate source f value

        while (queue.Count > 0)
        {
            Node currentNode = queue.OrderBy(node => node.f).First(); // Get node with lowest f value

            if (currentNode == destination)
            {
                // Outputs the full paths between nodes as well as its total cost
                return getPathDistance(currentNode, edges);
            }
            
            // List node as visited
            queue.Remove(currentNode);
            visited.Add(currentNode);

            // Get all neighbour nodes
            foreach (Node neighbour in getNeighbours(currentNode, edges)) 
            {
                // if neighbour has already been visited, continue to next neightbour
                if (visited.Contains(neighbour))
                {
                    continue;
                }
                // Calculate the neighbours g value from the path generated so far
                int tentativeG = currentNode.g + getDistance(currentNode, neighbour, edges);
                    
                // If neighbour is not in the queue or tentative g value is less than orignial, update neighbour and path
                if(!queue.Contains(neighbour) || tentativeG < neighbour.g)
                {
                    neighbour.parent = currentNode;
                    neighbour.g = tentativeG;
                    neighbour.f = neighbour.g + calculateHeuristic(neighbour, destination, edges);

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
    private static int calculateHeuristic(Node source, Node destination, Edge[] edges)
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

            foreach (Node neighbor in getNeighbours(current, edges))
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


    private static List<Node> getNeighbours(Node currentNode, Edge[] edges)
    {
        List<Node> neighbours = new List<Node>();

        // Find all edges which contain the current node and add the other node in the edge to list
        foreach (Edge edge in edges) 
        { 
            if(edge.startNode == currentNode || edge.endNode == currentNode)
            {
                if (currentNode == edge.startNode) neighbours.Add(edge.endNode); 
                else neighbours.Add(edge.startNode);
            }
        }
        return neighbours;
    }

    // finds weight between two nodes
    private static int getDistance(Node a, Node b, Edge[] edges)
    {
        var edge = edges.FirstOrDefault(e => (e.startNode == a && e.endNode == b) || (e.startNode == b && e.endNode == a));
        return edge?.weight ?? int.MaxValue; // if no edge is found, return infinty
    }

        // Outputs the full paths between nodes as well as its total cost
    private static int getPathDistance(Node currentNode, Edge[] edges)
    {
        int totalCost = 0;

        // Iterates through each parent node until no longer has a parent and thus path has ended
        while (currentNode.parent != null)
        {
            totalCost += getDistance(currentNode, currentNode.parent, edges);
            currentNode = currentNode.parent;
        }

        return totalCost;
    }
}

    

