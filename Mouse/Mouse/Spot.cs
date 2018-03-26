using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Priority_Queue;


namespace Mouse
{
    /// <summary>
    /// A custom object that contains information about each spot of the navigation model.
    /// This object has IComparable implemented in it to be sortable.
    /// </summary>
     class Spot: IComparable<Spot>
    {
        public int[] id = new int[2];

        public Point3d pos;

        public double cellSize;

        public float lb, ub;

        public bool los = true;

        public bool closedSet = false;

        public float f, g, he;

        public Spot parent;

        public List<int[]> neighbors;

        public List<Cell> adjCells;

        public Spot(double[] location ,int[] id,  double cellSize)
        {
            neighbors = new List<int[]>();
            adjCells = new List<Cell>();

            this.id = id;

            this.cellSize = cellSize;

            this.pos = new Point3d(this.id[0] * this.cellSize + location[0], this.id[1] * this.cellSize + location[1], 0);
        }

        public void getNeighbors(int gridSize)
        {
            if (this.id[0] < gridSize)
            {
                neighbors.Add(new int[] { this.id[0] + 1, this.id[1] });
            }
            if (this.id[0] > 0)
            {
                neighbors.Add(new int[] { this.id[0] - 1, this.id[1] });
            }
            if (this.id[1] < gridSize)
            {
                neighbors.Add(new int[] { this.id[0], this.id[1] + 1 });
            }
            if (this.id[1] > 0)
            {
                neighbors.Add(new int[] { this.id[0], this.id[1] - 1 });
            }
            if (this.id[0] > 0 && this.id[1] > 0)
            {
                neighbors.Add(new int[] { this.id[0] - 1, this.id[1] - 1 });
            }
            if (this.id[0] < gridSize && this.id[1] < gridSize)
            {
                neighbors.Add(new int[] { this.id[0] + 1, this.id[1] + 1 });
            }
            if (this.id[0] > 0 && this.id[1] < gridSize)
            {
                neighbors.Add(new int[] { this.id[0] - 1, this.id[1] + 1 });
            }
            if (this.id[0] < gridSize && this.id[1] > 0)
            {
                neighbors.Add(new int[] { this.id[0] + 1, this.id[1] - 1 });
            }
        }

        public int CompareTo(Spot other)
        {
            if (this.f == other.f)
            {
                return 0;
            }
            if(this.f > other.f)
            {
                return 1;
            }
            else
            {
                return -1;
            }
        }
    }
}
