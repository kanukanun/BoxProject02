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

        public Point3d CoordinateTransformation(Point3d world_position)
        {
            double halfDiagonalAngle, halfVerticalAngle, halfHorizontalAngle , nearDistance;
            camera.GetCameraAngle(out halfDiagonalAngle, out halfVerticalAngle, out halfHorizontalAngle);

            //view transformation
            Vector3d vec_location = new Vector3d(camera.CameraLocation.X, camera.CameraLocation.Y, camera.CameraLocation.Z);
            double angle_1 = Vector3d.VectorAngle(Vector3d.YAxis, camera.CameraZ, Plane.WorldXY);
            double angle_2 = Vector3d.VectorAngle(Vector3d.ZAxis, camera.CameraZ, Plane.WorldYZ);
            
            double[,] m1 = new double[4, 4];
            m1[0, 0] = Math.Cos(angle_1);
            m1[1, 0] = -Math.Sin(angle_1);
            m1[0, 1] = Math.Sin(angle_1);
            m1[1, 1] = Math.Cos(angle_1);
            m1[2, 2] = 1;
            m1[3, 3] = 1;

            double[,] m2= new double[4, 4];
            m2[0, 0] = 1;
            m2[1, 1] = Math.Cos(angle_2);
            m2[2, 1] = -Math.Sin(angle_2);
            m2[1, 2] = Math.Sin(angle_2);
            m2[2, 2] = Math.Cos(angle_2);
            m2[3, 3] = 1;
            
            double[,] m3 = new double[4, 4];
            m3[0, 0] = 1;
            m3[3, 0] = -vec_location.X;
            m3[1, 1] = 1;
            m3[3, 1] = -vec_location.Y;
            m3[2, 2] = 1;
            m3[3, 2] = -vec_location.Z;
            m3[3, 3] = 1;

            

            //purojection transformation
            List<Point3d> pos_farrec = new List<Point3d>();
            pos_farrec.Add(camera.GetFarRect()[1]);
            pos_farrec.Add(camera.GetFarRect()[3]);

            List<Point3d> pos_nearrec = new List<Point3d>();
            pos_nearrec.Add(camera.GetNearRect()[1]);
            pos_nearrec.Add(camera.GetNearRect()[3]);

            double dis_far = camera.CameraTarget.DistanceTo(Curve.CreateInterpolatedCurve(pos_farrec , 1).PointAt(0.5));
            double dis_near = camera.CameraTarget.DistanceTo(Curve.CreateInterpolatedCurve(pos_nearrec, 1).PointAt(0.5));
            
            double sy = 1 / Math.Tan(halfVerticalAngle / 2);
            double sx = sy / camera.FrustumAspect;
            double sz = dis_far / (dis_far - dis_near);
            //double[,] m2 = new double[4, 4];

            //m2[0, 0] = sx;
            //m2[1, 1] = sy;
            //m2[2, 2] = sz;
            //m2[3, 2] = -sz * dis_near;
            //m2[2, 3] = 1;

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
            double[,] original_pos = new double[1, 4];
            original_pos[0, 0] = world_position.X;
            original_pos[0, 1] = world_position.Y;
            original_pos[0, 2] = world_position.Z;
            original_pos[0, 3] = 1;

            double[,] screen1 = multiplyMatrices(original_pos, m1);
            double[,] screen2 = multiplyMatrices(screen1, m2);
            double[,] screen3 = multiplyMatrices(screen2, m3);

            Point3d pos_screen = new Point3d(screen3[0, 0], screen3[0, 1], screen3[0, 2]);
            return pos_screen;
            }

        public static double[,] multiplyMatrices(double[,] x, double[,] y)
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
            _doc.Objects.AddPoint(camera.CameraLocation);
            _doc.Objects.AddPoint(camera.CameraTarget);
            _doc.Objects.AddCurve(crv);
            _doc.Objects.AddCurve(enclose_view);
        }
    }
}
