using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using Newtonsoft.Json;

namespace Mouse.MazeComponent
{
    public class MazeComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MazeComponent()
          : base("Maze", "Maze",
              "Construct an Archimedean, or arithmetic, spiral given its radii and number of turns.",
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
            pManager.AddBrepParameter("SearchArea", "B", "Breps representing the search area.", GH_ParamAccess.tree);
            pManager.AddIntegerParameter("Subdivision", "N", "Level of subdivion for identifying walkable cells.", GH_ParamAccess.item, 3);
            pManager.AddNumberParameter("ScaleFactor", "F", "Search context scale factor.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("CellSize", "S", "Size of the navigation model's cells.", GH_ParamAccess.item);
            pManager.AddTextParameter("FilePath", "P", "Directory path for saving navigation model's JSON.", GH_ParamAccess.item, "");
            pManager.AddBooleanParameter("Run", "R", "Save the JSON file to the defined directory path.", GH_ParamAccess.item, false);


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
            //pManager.AddCurveParameter("Visualization", "V", "Navigation Model", GH_ParamAccess.item);
            pManager.AddBrepParameter("Context", "C", "Search Context", GH_ParamAccess.item);
            pManager.AddCurveParameter("Visualization", "V", "Navigation Model", GH_ParamAccess.tree);
            pManager.AddIntegerParameter("WallData", "D", "Wall Data", GH_ParamAccess.tree);
            pManager.AddTextParameter("Export", "E", "Serialization Result", GH_ParamAccess.item);


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
            GH_Structure<GH_Brep> searchAarea = new GH_Structure<GH_Brep>();
            int subdivision = new int();
            double scaleFactor = new float();
            double cellSize = new float();
            string filePath = "";
            bool run = new bool();


            // Then we need to access the input parameters individually. 
            // When data cannot be extracted from a parameter, we should abort this method.
            if (!DA.GetDataTree<GH_Brep>(0, out searchAarea)) return;
            if (!DA.GetData(1, ref subdivision)) return;
            if (!DA.GetData(2, ref scaleFactor)) return;
            if (!DA.GetData(3, ref cellSize)) return;
            if (!DA.GetData(4, ref filePath)) return;
            if (!DA.GetData(5, ref run)) return;


            // We should now validate the data and warn the user if invalid data is supplied.
            if (subdivision == 0.0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Subdivision level must be larger than zero.");
                return;
            }

            if (scaleFactor < 1.0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "ScaleFactor level must be larger than zero.");
                return;
            }

            if (cellSize == 0.0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "CellSize level must be larger than zero.");
                return;
            }


            // We're set to create the spiral now. To keep the size of the SolveInstance() method small, 
            // The actual functionality will be in a different method:
            string serializationResult = "";

            List<object> result = GenerateContext(searchAarea);

            Brep context = (Brep)result[0];
            Mesh searchAreaMesh = (Mesh)result[1];

            result = NavigationModel(searchAreaMesh, context, scaleFactor, cellSize, subdivision);

            DataTree<Polyline> navigationModel = (DataTree<Polyline>)result[0];
            DataTree<int> wallData = (DataTree<int>)result[1];
            WallInformation wallInfo = (WallInformation)result[2];

            if (run == true)
            {
                if (filePath != null && filePath != "")
                    serializationResult = serialization(wallInfo, filePath);
                else
                {
                    serializationResult = "Please provide a directory path for saving the Json text.";
                }
            }


            // Finally assign the spiral to the output parameter.
            DA.SetData(0, context);
            DA.SetDataTree(1, navigationModel);
            DA.SetDataTree(2, wallData);
            DA.SetData(3, serializationResult);
        }


        /// <summary>
        /// The method for generating Context Brep and a single mesh from the input Breps.
        /// </summary>
        /// <param name="breps"> Brep inputs that represent the search area for the pathfinding algorithm.</param>
        /// <returns>Rturns a list of generic types. First Item is a Brep representing the context of the navigation model.
        /// Second item of the list is a Mesh generated from the input Breps.
        /// </returns>

        private List<object> GenerateContext(GH_Structure<GH_Brep> breps)
        {
            List<object> result = new List<object>();
            List<GH_Brep> searchArea = breps.FlattenData();

            Mesh searchAreaMesh = new Mesh();

            Brep context = new Brep();

            // Generating a joined mesh from Breps
            if (searchArea.Count > 0)
            {
                for (int i = 0; i < searchArea.Count; i++)
                {
                    if (i == 0)
                    {
                        searchAreaMesh = Mesh.CreateFromBrep(searchArea[i].Value)[0];
                    }
                    else
                    {
                        searchAreaMesh.Append(Mesh.CreateFromBrep(searchArea[i].Value)[0]);
                    }
                }
            }
            else
            {
                searchAreaMesh = Mesh.CreateFromBrep(searchArea[0].Value)[0];
            }

            // Generating Context 
            BoundingBox bBox = searchAreaMesh.GetBoundingBox(true);
            Point3d origin = bBox.Corner(true, false, true);
            Point3d oposite = bBox.Corner(false, true, true);

            double deltaX = System.Math.Abs(origin.X - oposite.X);
            double deltaY = System.Math.Abs(origin.Y - oposite.Y);

            double dimension = System.Math.Max(deltaX, deltaY);

            Rectangle3d contextBounds = new Rectangle3d(new Plane(origin, Vector3d.ZAxis), dimension, -dimension);

            context = Brep.CreateFromCornerPoints(contextBounds.Corner(0), contextBounds.Corner(1), contextBounds.Corner(2), contextBounds.Corner(3), 0.001);

            result.Add(context);
            result.Add(searchAreaMesh);

            return result;
        }

        /// <summary>
        /// The Method that generates Navigation Model for the PathFinding algorithm.
        /// </summary>
        /// <param name="searchAreaMesh"> A Mesh generated from the input Breps representing the search area.</param>
        /// <param name="context"> A Brep that represents context of the navigation model.</param>
        /// <param name="scaleFactor"> A double larger than 1.</param>
        /// <param name="cellSize"> A double for defining the size of the navigation model cells.</param>
        /// <param name="subdivision"> Number of subdivision of navigation model's cells.</param>
        /// <returns> A list of generic types. The first item of the list is a DataTree of Curves for visualizing the navigation model.
        /// The second item of the list is a DataTree of zeros and ones related to the location of the obstacles and walls in the 
        /// navigation model, which is called wallData.
        /// The final item of the list is a custom object (WallInformation) that contains the wallData in it. This object will be used
        /// to generate and save a Json file from the wallData.
        /// </returns>
        private List<object> NavigationModel(Mesh searchAreaMesh, Brep context, double scaleFactor, double cellSize, int subdivision)
        {
            List<object> result = new List<object>();

            int[,] wallDataArray;
            DataTree<int> wallData = new DataTree<int>();
            DataTree<Polyline> navigationCells = new DataTree<Polyline>();

            Rectangle3d cell = new Rectangle3d();

            Brep scaledContext = new Brep();
            scaledContext.Append(context);
            scaledContext.Transform(Transform.Scale(scaledContext.GetBoundingBox(true).Center, scaleFactor));

            int cellCount = Convert.ToInt32(System.Math.Round((context.Edges[0].Domain[1] * scaleFactor) / cellSize));

            // Array for serializing to Json
            wallDataArray = new int[cellCount, cellCount];

            Plane modelOrigin = new Plane(scaledContext.GetBoundingBox(true).Corner(true, true, true), Vector3d.ZAxis);

            Point3d sample = new Point3d();

            for (int i = 0; i < cellCount; i++)
            {
                for (int j = 0; j < cellCount; j++)
                {
                    cell = new Rectangle3d(new Plane(new Point3d(modelOrigin.OriginX + (i * cellSize), modelOrigin.OriginY + (j * cellSize), modelOrigin.OriginZ), modelOrigin.Normal), cellSize, cellSize);
                    navigationCells.Add(cell.ToPolyline(), new GH_Path(i));

                    int intersect = 0;

                    for (double k = 0; k < (subdivision * 4); k++)
                    {
                        sample = cell.PointAt(k * 4 / (subdivision * 4));

                        if (Rhino.Geometry.Intersect.Intersection.MeshRay(searchAreaMesh, new Ray3d(sample, Vector3f.ZAxis)) >= 0)
                        {
                            intersect++;
                        }
                    }
                    if (intersect >= subdivision * 4)
                    {
                        wallData.Add(0, new GH_Path(i));
                        wallDataArray[i, j] = 0;
                    }
                    else
                    {
                        wallData.Add(1, new GH_Path(i));
                        wallDataArray[i, j] = 1;
                    }

                }
            }

            WallInformation wallInfo = new WallInformation();
            wallInfo.WallData = wallDataArray;

            result.Add(navigationCells);
            result.Add(wallData);
            result.Add(wallInfo);

            return result;
        }

        /// <summary>
        ///  A method for creating and saving a Json file from the wallData of the navigation model.
        /// </summary>
        /// <param name="wallInfo"> A custom object (WallInformation) that contains wallData in it.</param>
        /// <param name="filePath"> A directory path for savig the Json file.</param>
        /// <returns> A string that illustrates result of the serialization and saving process.</returns>
        private string serialization(WallInformation wallInfo, string filePath)
        {
            string result = "Json file was successfully saved.";
            string json = "";

            try
            {
                json = JsonConvert.SerializeObject(wallInfo);
            }
            catch (System.IO.IOException e)
            {
                result = e.ToString();
            }

            if (json != "")
            {
                try
                {
                    System.IO.File.WriteAllText(System.IO.Path.Combine(filePath, "WallInformation.json"), json);
                }
                catch (System.IO.IOException e)
                {
                    result = e.ToString();
                }
            }


            return result;
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
                return Mouse.Properties.Resources.maze;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("f7805db4-3892-4e0b-97e2-d0d18aa00b32"); }
        }
    }
}