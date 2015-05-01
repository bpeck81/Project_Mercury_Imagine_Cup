using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.BodyBasics
{
   public class Vector3
    {
      public  float X;
      public  float Y;
      public  float Z;

        public Vector3(){

        }
        public Vector3(float X, float Y, float Z)
        {
            this.X = X;
            this.Y = Y;
            this.Z = Z;
        }
        public Vector3(CameraSpacePoint A,CameraSpacePoint B)
        {
            this.X = B.X - A.X;
            this.Y = B.Y - A.Y;
            this.Z = B.Z - A.Z;
            
        }
        public static Vector3 crossProduct(Vector3 AB, Vector3 BC)
        {
            Vector3 orthoVector = new Vector3();

            orthoVector.X = (AB.Y * BC.Z) - (AB.Z * BC.Y);
            orthoVector.Y = (AB.X * BC.Z) - (AB.Z * BC.X);
            orthoVector.Z = (AB.X * BC.Y) - (AB.Y * BC.X);


            return orthoVector;

        }
        public static float dotProduct(Vector3 A, Vector3 B)
        {
            var scalarProduct = (A.X * B.X) + (A.Y * B.Y) + (A.Z * B.Z);

            return scalarProduct;
        }

        public static float magnitude(Vector3 vector)
        {
            float mag = (float)Math.Sqrt((Math.Pow((double)vector.X, 2) + Math.Pow((double)vector.Y, 2) + Math.Pow((double)vector.Z, 2)));
            return mag;
        }

        public static float compProjection(Vector3 A, Vector3 normal)
        {
            var scalar = Vector3.dotProduct(A, normal) / Vector3.magnitude(normal);
            return scalar;
        }
    }
}
