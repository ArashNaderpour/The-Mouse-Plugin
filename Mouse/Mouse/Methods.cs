using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Priority_Queue;

namespace Mouse
{
    class Methods
    {
        public static float calcHeuristic(Spot s1, Spot s2)
        {
            return (float) s1.pos.DistanceTo(s2.pos);
        }

        /// <summary>
        /// A recursive method for building the shoetest path from the goal node to the start node
        /// by following the parent children relationship between nodes.
        /// </summary>
        /// <param name="path"> A list of Point3ds that shortest path passes through them.</param>
        /// <param name="node"> A node object representing the current location.</param>
        /// <param name="start"> A node object representing the start location.</param>
        public static void buildPath(List<Point3d> path, Spot node, Spot start)
        {
            path.Add(node.pos);
            if (node.parent != start)
            {
                path.Add(node.parent.pos);
                buildPath(path, node.parent, start);
            }
            else
            {
                path.Add(start.pos);
            }
        }

        /// <summary>
        /// A method for generating Polyline representaiton of the shortest path.
        /// </summary>
        /// <param name="path"> A list of Point3ds that shortest path passes through them.</param>
        /// <returns> A Polyline representing the shortest path.</returns>
        public static Polyline drawPath(List<Point3d> path)
        {
            Polyline shortestPath = new Polyline(path);

            return shortestPath;
        }

        public static void updateBounds(Spot[,] grid, Spot s, Spot start)
        {
            s.lb = -float.MaxValue;
            s.ub = float.MaxValue;

            float angle = new float();

            Cell adjCell;
            Spot neighbor;

            if (s != start)
            {
                // Condition - 1
                for(int i = 0; i < s.adjCells.Count; i++)
                {
                    adjCell = s.adjCells[i];
                    if (adjCell.wall == 1)
                    {
                        for (int j =0; j < adjCell.cornerSpots.Length; j++)
                        {
                            adjCell.cornerSpots[j].los = false;
                            angle = theta(s.pos, s.parent.pos, adjCell.cornerSpots[j].pos);

                            if (s.parent == adjCell.cornerSpots[j] || angle < 0 || (angle == 0 && s.parent.pos.DistanceTo(adjCell.cornerSpots[j].pos) <= s.parent.pos.DistanceTo(s.pos)))
                            {
                                s.lb = 0;
                            }
                            if (s.parent == adjCell.cornerSpots[j] || angle > 0 || (angle == 0 && s.parent.pos.DistanceTo(adjCell.cornerSpots[j].pos) <= s.parent.pos.DistanceTo(s.pos)))
                            {
                                s.ub = 0;
                            }
                        }
                    }
                }
                for (int i = 0; i < s.neighbors.Count; i++)
                {
                    neighbor = grid[s.neighbors[i][0], s.neighbors[i][1]];
                    angle = theta(s.pos, s.parent.pos, neighbor.pos);
                    // Condition - 2
                    if (neighbor.closedSet == true && s.parent == neighbor.parent && neighbor != start)
                    {
                        if (neighbor.lb + angle <= 0)
                        {
                            s.lb = Math.Max(s.lb, neighbor.lb + angle);
                        }
                        if(neighbor.ub + angle >= 0)
                        {
                            s.ub = Math.Min(s.ub, neighbor.ub + angle);
                        }
                    }
                    // Condition - 3
                    if ((neighbor.closedSet == false || s.parent != neighbor.parent) && (s.parent.pos.DistanceTo(neighbor.pos) < s.parent.pos.DistanceTo(s.pos)) && s.parent != neighbor)
                    {
                        if (angle < 0)
                        {
                            s.lb = Math.Max(s.lb, angle);
                        }
                        if (angle > 0)
                        {
                            s.ub = Math.Min(s.ub, angle);
                        }
                    }
                }
            }
        }

        public static void updateVertex(SimplePriorityQueue<Spot> openSet, Spot s, Spot succ, Spot start, Spot goal)
        {
            double angle = theta(s.pos, s.parent.pos, succ.pos);
            if(s != start && s.lb <= angle && s.ub >= angle)
            {
                // Path - 2
                if (s.parent.g + s.parent.pos.DistanceTo(succ.pos) < succ.g && s.los)
                {
                    succ.g = s.parent.g + (float) s.parent.pos.DistanceTo(succ.pos);
                    succ.parent = s.parent;
                    if (openSet.Contains(succ))
                    {
                        openSet.Remove(succ);
                    }
                    succ.f = succ.g + calcHeuristic(succ, goal);
                    openSet.Enqueue(succ, succ.f);
                }
            }
            else
            {
                // Path - 1
                if (s.g + s.pos.DistanceTo(succ.pos) < succ.g && s.los)
                {
                    succ.g = s.g + (float)s.pos.DistanceTo(succ.pos);
                    succ.parent = s;
                    if (openSet.Contains(succ))
                    {
                        openSet.Remove(succ);
                    }
                    succ.f = succ.g + calcHeuristic(succ, goal);
                    openSet.Enqueue(succ, succ.f);
                }
            }
        }

        /// <summary>
        /// A method that calculates angle between three nodes.
        /// </summary>
        /// <param name="s">A node object that represents the current node.</param>
        /// <param name="parent"> A node object that represents parent of the current node.</param>
        /// <param name="succ"> A node object that represents successor of the current node.</param>
        /// <returns> The angle between the three input nodes, limited in the range of -180 to 180 degree.</returns>
        public static float theta(Point3d s, Point3d parent, Point3d succ)
        {
            double dotProduct = Vector3d.Multiply(new Vector3d(s.X - parent.X, s.Y - parent.Y, 0), new Vector3d(succ.X - parent.X, succ.Y - parent.Y, 0));
            double sign = Vector3d.CrossProduct(new Vector3d(new Vector3d(s.X - parent.X, s.Y - parent.Y, 0)), new Vector3d(succ.X - parent.X, succ.Y - parent.Y, 0)).Z;
            double Mag_Parent_S = Math.Sqrt(Math.Pow(s.X - parent.X, 2) + Math.Pow(s.Y - parent.Y, 2));
            double Mag_Parent_Succ = Math.Sqrt(Math.Pow(succ.X - parent.X, 2) + Math.Pow(succ.Y - parent.Y, 2));
            if (sign > 0)
            {
                return (float) (Math.Acos(dotProduct / (Mag_Parent_S * Mag_Parent_Succ)) * (180 / Math.PI));
            }
            else
            {
                return (float) (Math.Acos(dotProduct / (Mag_Parent_S * Mag_Parent_Succ)) * (-180 / Math.PI));
            }
        }
    }
}




