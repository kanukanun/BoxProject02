using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Rhino;
using Rhino.Input;
using Rhino.Geometry;
using Rhino.Display;

namespace _5.Classes
{
    class BoxProject
    {
        private Box box;

        private int size;

        private List<Point3d> corner = new List<Point3d>();
        private List<Point3d> clientpos = new List<Point3d>();//ワールド座標からクライアント座標に変換した3Dポイント

        private Curve crv;
        private Curve enclose_view;

        private List<Point3d> camtarget = new List<Point3d>();

        private RhinoView view;
        private RhinoViewport camera;

        public BoxProject(
            int _size,
            RhinoView _view
        )
        {
            size = _size;
            view = _view;

            camera = new RhinoViewport(view.ActiveViewport);
        }

        public void MakeBox()
        {
            box = new Box(Plane.WorldXY, new Interval(0, size), new Interval(0, size), new Interval(0, size));

            corner.AddRange(box.GetCorners());
        }

        private Vector3d NormalVector(Vector3d v1, Vector3d v2)
        {
            double ax, ay, az, bx, by, bz;
            Vector3d normvec;

            ax = v1.X;
            ay = v1.Y;
            az = v1.Z;
            bx = v2.X;
            by = v2.Y;
            bz = v2.Z;

            normvec = new Vector3d(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
            return normvec / normvec.Length;
        }

        public double VectorAngle(Vector3d v1, Vector3d v2)
        {
            double cx, cy, cz, dx, dy, dz, angle;

            cx = v1.X;
            cy = v1.Y;
            cz = v1.Z;
            dx = v2.X;
            dy = v2.Y;
            dz = v2.Z;

            angle = Math.Acos((cx * dx + cy * dy + cz * dz) / (v1.Length * v2.Length));
            return angle;
        }

        public double[,] PointToMatrix(Point3d point)
        {
            double[,] original_pos = new double[1, 4];
            original_pos[0, 0] = point.X;
            original_pos[0, 1] = point.Y;
            original_pos[0, 2] = point.Z;
            original_pos[0, 3] = 1;

            return original_pos;
        }

        public Point3d CoordinateTransformation(Point3d world_position)
        {
            double halfDiagonalAngle, halfVerticalAngle, halfHorizontalAngle , nearDistance;
            camera.GetCameraAngle(out halfDiagonalAngle, out halfVerticalAngle, out halfHorizontalAngle);

            ///////////////////////
            //view transformation//
            ///////////////////////

            Vector3d vec_norm = NormalVector(camera.CameraZ, Vector3d.ZAxis);
            double angle = VectorAngle(-Vector3d.ZAxis, camera.CameraZ);

            double[,] m1 = new double[4, 4];
            m1[0, 0] = vec_norm.X * vec_norm.X * (1 - Math.Cos(angle)) + Math.Cos(angle);
            m1[1, 0] = vec_norm.X * vec_norm.Y * (1 - Math.Cos(angle)) + vec_norm.Z * Math.Sin(angle);
            m1[2, 0] = vec_norm.Z * vec_norm.X * (1 - Math.Cos(angle)) - vec_norm.Y * Math.Sin(angle);
            m1[0, 1] = vec_norm.X * vec_norm.Y * (1 - Math.Cos(angle)) - vec_norm.Z * Math.Sin(angle);
            m1[1, 1] = vec_norm.Y * vec_norm.Y * (1 - Math.Cos(angle)) + Math.Cos(angle);
            m1[2, 1] = vec_norm.Y * vec_norm.Z * (1 - Math.Cos(angle)) + vec_norm.X * Math.Sin(angle);
            m1[0, 2] = vec_norm.Z * vec_norm.X * (1 - Math.Cos(angle)) + vec_norm.Y * Math.Sin(angle);
            m1[1, 2] = vec_norm.Y * vec_norm.Z * (1 - Math.Cos(angle)) - vec_norm.X * Math.Sin(angle);
            m1[2, 2] = vec_norm.Z * vec_norm.Z * (1 - Math.Cos(angle)) + Math.Cos(angle);
            m1[3, 3] = 1;

            double[,] pos_cameralocation = MultiplyMatrices(PointToMatrix(camera.CameraLocation), m1);
            Vector3d vec_location = new Vector3d(pos_cameralocation[0, 0], pos_cameralocation[0, 1], pos_cameralocation[0, 2]);

            double[,] m2 = new double[4, 4];
            m2[0, 0] = 1;
            m2[1, 1] = 1;
            m2[2, 2] = 1;
            m2[3, 0] = -vec_location.X;
            m2[3, 1] = -vec_location.Y;
            m2[3, 2] = -vec_location.Z;
            m2[3, 3] = 1;
            
            //////////////////////////////
            //purojection transformation//
            //////////////////////////////

            Plane planenear, planefar;
            camera.GetFrustumNearPlane(out planenear);
            camera.GetFrustumFarPlane(out planefar);

            double dis_near = camera.CameraLocation.DistanceTo(planenear.Origin);
            double dis_far = camera.CameraLocation.DistanceTo(planefar.Origin);

            //RhinoApp.WriteLine(String.Format("{0}", RhinoMath.ToDegrees(halfVerticalAngle)));

            double range = Math.Tan(halfVerticalAngle / 2) * dis_near;
            double sy = dis_near / range;
            double sx = (2 * dis_near) / (range * camera.FrustumAspect + range * camera.FrustumAspect);
            double sz = -(dis_far + dis_near) / (dis_far - dis_near);
            double pz = -(2 * dis_far * dis_near) / (dis_far - dis_near);
            double[,] m3 = new double[4, 4];

            m3[0, 0] = sx;
            m3[1, 1] = -sy;
            m3[2, 2] = sz;
            m3[3, 2] = pz;
            m3[2, 3] = -1;

            //screen transformation
            //double w = camera.Size.Width / 2;
            //double h = camera.Size.Height / 2;
            //double[,] m3 = new double[4, 4];
            //m3[0, 0] = w;
            //m3[3, 0] = w;
            //m3[1, 1] = -h;
            //m3[3, 1] = h;
            //m3[2, 2] = 1;
            //m3[3, 3] = 1;
            

            //point coordination
            

            double[,] screen1 = MultiplyMatrices(PointToMatrix(world_position), m1);
            double[,] screen2 = MultiplyMatrices(screen1, m2);
            
            double[,] screen3 = MultiplyMatrices(screen2, m3);

            RhinoApp.WriteLine(String.Format("{0}", m2[0, 1]));
            RhinoApp.WriteLine(String.Format("{0}", m3[0, 1]));

            Point3d pos_screen = new Point3d(screen3[0, 0], screen3[0, 1], screen3[0, 2]);
            return pos_screen;
            }

        public static double[,] MultiplyMatrices(double[,] x, double[,] y)
        {
            double[,] z = new double[1, 4];
            for (int i = 0; i < 4; i++)
            {
                z[0, i] = 0;
                for (int j = 0; j < 4; j++)
                {
                        z[0, i] += x[0, j] * y[j, i];
                }
            }
            return z;
        }

        public void Area()
        {
            List<Point3d> vertex = new List<Point3d>(3);
            List<Point3d> vertex_rec = new List<Point3d>();
            List<Curve> triangle_list = new List<Curve>();

            camtarget.Add(new Point3d(camera.CameraTarget));
            camtarget.Add(new Point3d(camera.CameraLocation));
            Vector3d vec_height = Vector3d.Multiply(camera.Size.Height , camera.CameraY);
            camtarget.Add(Point3d.Add(camera.CameraTarget , vec_height));
            
            //i revise the following sentences
            for (int i = 0; i < corner.Count; i++)
            {
                clientpos.Add(CoordinateTransformation(corner[i]));
            }

            //for (int i = 0; i < clientpos3d.Count - 2; i++)//プロットした各点から３つ選択してできる三角形をすべて求める。
            //{
            //    vertex.Clear();
            //    vertex.Add(clientpos3d[i]);
            //    vertex.Add(clientpos3d[i + 1]);

            //    for (int j = i + 2; j < clientpos3d.Count; j++)
            //    {
            //        vertex.Add(clientpos3d[j]);
            //        vertex.Add(clientpos3d[i]);
            //        triangle_list.Add(Curve.CreateInterpolatedCurve(vertex, 1));
            //        vertex.RemoveAt(3);
            //        vertex.RemoveAt(2);
            //    }
            //}

            //client veiwを2D平面上へプロットする。
            vertex_rec.Add(new Point3d(0 , 0 , 0));
            vertex_rec.Add(new Point3d(camera.Size.Width , 0 , 0));
            vertex_rec.Add(new Point3d(camera.Size.Width, camera.Size.Height, 0));
            vertex_rec.Add(new Point3d(0 , camera.Size.Height, 0));
            vertex_rec.Add(new Point3d(0, 0, 0));

            enclose_view = Curve.CreateInterpolatedCurve(vertex_rec, 1);//画面の範囲を線で囲う
            //Curve enclose_box = Curve.CreateBooleanUnion(triangle_list)[0];//ボックスの面積を囲う
            //crv = Curve.CreateBooleanIntersection(enclose_box , enclose_view)[0];//画面上のボックスの範囲を求める。
            
            //double area_view = AreaMassProperties.Compute(enclose_view).Area; 
            //double area_intersec = AreaMassProperties.Compute(crv).Area;
            //RhinoApp.WriteLine(String.Format("{0}", area_view));
            //RhinoApp.WriteLine(String.Format("{0}", area_intersec));
            //double account = area_intersec / area_view * 100;
            //RhinoApp.WriteLine(String.Format("The box accounts for {0}% of viewport", account));
        }
        
        public void Display(RhinoDoc _doc)
        {
            Brep objs = Brep.CreateFromBox(box);
            _doc.Objects.AddBrep(objs);
            for (int i = 0; i < clientpos.Count; i++)
            {
                _doc.Objects.AddPoint(clientpos[i]);
            }
            //for (int i = 0; i < 4; i++)
            //{
            //    _doc.Objects.AddPoint(camera.GetFarRect()[i]);
            //}
            //for (int i = 0; i < 4; i++)
            //{
            //    _doc.Objects.AddPoint(camera.GetNearRect()[i]);
            //}

            _doc.Objects.AddCurve(Curve.CreateInterpolatedCurve(camera.GetFarRect(), 1));
            _doc.Objects.AddCurve(Curve.CreateInterpolatedCurve(camera.GetNearRect(), 1));

            _doc.Objects.AddPoint(camera.CameraLocation);
            _doc.Objects.AddPoint(camera.CameraTarget);
            _doc.Objects.AddCurve(crv);
            _doc.Objects.AddCurve(enclose_view);
        }
    }
}
