using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Mouse
{
    /// <summary>
    /// A custom object that contains information about each cell of the navigation model.
    /// </summary>
    class Cell
    {
        public int wall = 0;

        public Spot[] cornerSpots = new Spot[4];

        public Cell(double cellSize, int wall)
        {
            this.wall = wall;
        }

        public Object display()
        {
            Rectangle3d visualization = new Rectangle3d(Plane.WorldXY, cornerSpots[0].pos, new Point3d(cornerSpots[2].pos));
            if (this.wall == 0)
            {
                return visualization;
            }
            else
            {
                return Brep.CreateFromBox(visualization.BoundingBox);
            }
        }

    }
}
