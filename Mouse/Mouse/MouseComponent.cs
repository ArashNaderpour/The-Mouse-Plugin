using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using Priority_Queue;


namespace Mouse
{
    public class MouseComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MouseComponent()
          : base("Mouse", "Mouse",
              "Shortest path algorithm based on line of sight.",
              "Mouse", "Mouse")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // Use the pManager object to register your input parameters.
            // You can often supply default values when creating parameters.
            // All parameters must have the correct access type. If you want 
            // to import lists or trees of values, modify the ParamAccess flag.
            pManager.AddBooleanParameter("Run", "R", "Start the algorithm.", GH_ParamAccess.item, false);
            pManager.AddBrepParameter("Context", "C", "A surface representing the context of the project.", GH_ParamAccess.item);
            pManager.AddPointParameter("StartPoint", "Sta", "Points representing start points.", GH_ParamAccess.item);
            pManager.AddPointParameter("DestinationPoint", "Des", "Points representing goal points.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("WallData", "D", "Obstacle information.", GH_ParamAccess.tree);
            pManager.AddNumberParameter("CellSize", "S", "Size of model's cells.", GH_ParamAccess.item);

            // If you want to change properties of certain parameters, 
            // you can use the pManager instance to access them by index:
            //pManager[0].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // Use the pManager object to register your output parameters.
            // Output parameters do not have default values, but they too must have the correct access type.
            pManager.AddGeometryParameter("Routes", "R", "Shortest routes", GH_ParamAccess.item);

            // Sometimes you want to hide a specific parameter from the Rhino preview.
            // You can use the HideParameter() method as a quick way:
            //pManager.HideParameter(0);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // First, we need to retrieve all data from the input parameters.
            // We'll start by declaring variables and assigning them starting values.
            bool run = new bool();
            Brep context = new Brep();
            Point3d startPoint = new Point3d();
            Point3d goalPoint = new Point3d();
            GH_Structure<GH_Integer> wallData = new GH_Structure<GH_Integer>();
            double cellSize = new double();

            // Then we need to access the input parameters individually. 
            // When data cannot be extracted from a parameter, we should abort this method.
            if (!DA.GetData(0, ref run)) return;
            if (!DA.GetData(1, ref context)) return;
            if (!DA.GetData(2, ref startPoint)) return;
            if (!DA.GetData(3, ref goalPoint)) return;
            if (!DA.GetDataTree<GH_Integer>(4, out wallData)) return;
            if (!DA.GetData(5, ref cellSize)) return;

            // We should now validate the data and warn the user if invalid data is supplied.
            if (cellSize <= 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Cell size must be larger than 0.");
                return;
            }

            // We're set to create the spiral now. To keep the size of the SolveInstance() method small, 
            // The actual functionality will be in a different method:
            Polyline routes = new Polyline();

            if (run)
            {
                routes = pathFinding(run, context, startPoint, goalPoint, wallData, cellSize);
                DA.SetData(0, routes);
            }
            // Finally assign the spiral to the output parameter.

        }
        /// <summary>
        /// Core Process of Theta* pathfinding algorithm.
        /// </summary>
        /// <param name="run"> A Boolean value that declare starting of the algorithm.</param>
        /// <param name="context"> A Brep representing the context of the navigation model.</param>
        /// <param name="startPoint"> A Point3d representing the start location.</param>
        /// <param name="goalPoint"> A Point3d representing the destination location.</param>
        /// <param name="wallData"> A DataTree representing the locations of the obstacle cells in the navigation model.</param>
        /// <param name="cellSize"> A double that indicates size of the navigation model's cells.</param>
        /// <returns> A Polyline that represents the result of the Theta* pathfinding algorithm.</returns>
        Polyline pathFinding(bool run, Brep context, Point3d startPoint, Point3d goalPoint, GH_Structure<GH_Integer> wallData, double cellSize)
        {
            int[] startID = new int[2];
            int[] goalID = new int[2];

            SimplePriorityQueue<Spot> openSet = new SimplePriorityQueue<Spot>();

            List<Point3d> path = new List<Point3d>();

            Polyline finalPaths = new Polyline();

            int gridSize = wallData.Branches.Count;

            double[] modelLocation = { context.Vertices[0].Location.X, context.Vertices[0].Location.Y };

            Spot neighbor;
            Spot current;

            Spot[,] grid = new Spot[gridSize + 1, gridSize + 1];
            Cell[,] cells = new Cell[gridSize, gridSize];

            for (int i = 0; i < gridSize + 1; i++)
            {
                for (int j = 0; j < gridSize + 1; j++)
                {
                    if (i < gridSize && j < gridSize)
                    {
                        cells[i, j] = new Cell(cellSize, wallData.Branches[i][j].Value);
                    }

                    grid[i, j] = new Spot(modelLocation, new int[] { i, j }, cellSize);
                    grid[i, j].g = float.MaxValue;
                    grid[i, j].f = float.MaxValue;
                    grid[i, j].getNeighbors(gridSize);
                    if (i == 0 && j == 0)
                    {
                        grid[i, j].adjCells.Add(cells[i, j]);
                    }
                    if (i == 0 && j == gridSize)
                    {
                        grid[i, j].adjCells.Add(cells[0, j - 1]);
                    }
                    if (i == gridSize && j == 0)
                    {
                        grid[i, j].adjCells.Add(cells[i - 1, j]);
                    }
                    if (i == gridSize && j == gridSize)
                    {
                        grid[i, j].adjCells.Add(cells[i - 1, j - 1]);
                    }
                    if (i == 0 && j > 0 && j < gridSize)
                    {
                        grid[i, j].adjCells.Add(cells[i, j]);
                        grid[i, j].adjCells.Add(cells[i, j - 1]);
                    }
                    if (j == 0 && i > 0 && i < gridSize)
                    {
                        grid[i, j].adjCells.Add(cells[i, j]);
                        grid[i, j].adjCells.Add(cells[i - 1, j]);
                    }
                    if (i == gridSize && j > 0 && j < gridSize)
                    {
                        grid[i, j].adjCells.Add(cells[i - 1, j - 1]);
                        grid[i, j].adjCells.Add(cells[i - 1, j]);
                    }
                    if (j == gridSize && i > 0 && i < gridSize)
                    {
                        grid[i, j].adjCells.Add(cells[i - 1, j - 1]);
                        grid[i, j].adjCells.Add(cells[i, j - 1]);
                    }
                    if (i > 0 && j > 0 && i < gridSize && j < gridSize)
                    {
                        grid[i, j].adjCells.Add(cells[i, j]);
                        grid[i, j].adjCells.Add(cells[i - 1, j]);
                        grid[i, j].adjCells.Add(cells[i - 1, j - 1]);
                        grid[i, j].adjCells.Add(cells[i, j - 1]);
                    }
                }
            }

            for (int i = 0; i < gridSize; i++)
            {
                for (int j = 0; j < gridSize; j++)
                {
                    cells[i, j].cornerSpots[0] = grid[i, j];
                    cells[i, j].cornerSpots[1] = grid[i + 1, j];
                    cells[i, j].cornerSpots[2] = grid[i + 1, j + 1];
                    cells[i, j].cornerSpots[3] = grid[i, j + 1];
                }
            }

            double tempValue = double.MaxValue;

            double tempDistance = new double();

            for (int i = 0; i < grid.GetLength(0); i++)
            {
                for (int j = 0; j < grid.GetLength(1); j++)
                {
                    tempDistance = startPoint.DistanceTo(grid[i, j].pos);
                    if (tempDistance < tempValue)
                    {
                        tempValue = tempDistance;
                        startID[0] = i;
                        startID[1] = j;
                    }
                }
            }

            tempValue = double.MaxValue;

            for (int i = 0; i < grid.GetLength(0); i++)
            {
                for (int j = 0; j < grid.GetLength(1); j++)
                {
                    tempDistance = goalPoint.DistanceTo(grid[i, j].pos);
                    if (tempDistance < tempValue)
                    {
                        tempValue = tempDistance;
                        goalID[0] = i;
                        goalID[1] = j;
                    }
                }
            }

            Spot start = grid[startID[0], startID[1]];
            Spot goal = grid[goalID[0], goalID[1]];

            start.g = 0;
            start.parent = start;
            start.he = Methods.calcHeuristic(start, goal);

            openSet.Enqueue(start, start.f);

            while (openSet.Count > 0)
            {
                current = openSet.Dequeue();
                if (current == goal)
                {
                    Methods.buildPath(path, current, start);
                    finalPaths = Methods.drawPath(path);
                    break;
                }
                current.closedSet = true;
                Methods.updateBounds(grid, current, start);
                for (int k = 0; k < current.neighbors.Count; k++)
                {
                    neighbor = grid[current.neighbors[k][0], current.neighbors[k][1]];
                    if (neighbor.closedSet == false)
                    {
                        if (!openSet.Contains(neighbor))
                        {
                            neighbor.g = float.MaxValue;
                        }
                        Methods.updateVertex(openSet, current, neighbor, start, goal);
                    }
                }
            }

            return finalPaths;
        }

        /// <summary>
        /// The Exposure property controls where in the panel a component icon 
        /// will appear. There are seven possible locations (primary to septenary), 
        /// each of which can be combined with the GH_Exposure.obscure flag, which 
        /// ensures the component will only be visible on panel dropdowns.
        /// </summary>
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return Mouse.Properties.Resources.mouse;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("9a06f900-1baf-4275-9f9b-8ecdc266bff5"); }
        }
    }
}
