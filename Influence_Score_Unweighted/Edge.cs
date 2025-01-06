using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
    internal class Edge
    {
        public Node startNode { get; set; }
        public Node endNode { get; set; }

        public Edge(Node node1, Node node2)
        {
            startNode = node1;
            endNode = node2;
        }
    }
