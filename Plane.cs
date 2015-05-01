using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace Microsoft.Samples.Kinect.BodyBasics
{
    public class Plane
    {
        Vector3 vectorAB;
        Vector3 vectorAC;
       public Vector3 normal;
       public Vector4 definedPlane;
        public Plane(CameraSpacePoint[] spacePoints)
        {
            definedPlane = new Vector4();
            vectorAB = new Vector3();
            vectorAC = new Vector3();
            normal = new Vector3();
            vectorAB.X = spacePoints[1].X - spacePoints[0].X;
            vectorAB.Y = spacePoints[1].Y - spacePoints[0].Y;
            vectorAB.Z = spacePoints[1].Z - spacePoints[0].Z;

            vectorAC.X = spacePoints[1].X - spacePoints[0].X;
            vectorAC.Y = spacePoints[1].Y - spacePoints[0].Y;
            vectorAC.Z = spacePoints[1].Z - spacePoints[0].Z;

            normal = Vector3.crossProduct(vectorAB, vectorAC);
            var D = -(normal.X * spacePoints[0].X) - (normal.Y * spacePoints[0].Y) - (normal.Z * spacePoints[0].Z);
            definedPlane.X = normal.X;
            definedPlane.Y = normal.Y;
            definedPlane.Z = normal.Z;
            definedPlane.W = D;
            
        }
        public Plane(Vector4 definedPlane)
        {
            this.definedPlane = definedPlane;
            this.normal = new Vector3(definedPlane.X, definedPlane.Y, definedPlane.Z);
        }

 
        public Boolean checkPoint(CameraSpacePoint point){
            Boolean onPlane = false;
        if (((definedPlane.X * point.X) + (definedPlane.Y * point.Y) + (definedPlane.Z * point.Z) + definedPlane.W) == 0)
        {
            onPlane = true;
        }
        return onPlane;
        }


    }
}
