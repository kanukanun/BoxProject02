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
        ////////////// 
        //properties// 
        ////////////// 

        private Box box;

        private int size;

        private List<Point3d> corner = new List<Point3d>();
        private List<Point3d> clientpos = new List<Point3d>();//ワールド座標からクライアント座標に変換したポイントのリスト

        private Curve enclose_intersect;
        private Curve enclose_view;

        private RhinoView view;//Rhinoのビューポートの設定を格納
        private RhinoViewport camera;//ビューポートのカメラの設定

        //////////////// 
        //constructors// 
　      //////////////// 


        public BoxProject(
            int _size,
            RhinoView _view
        )
        {
            size = _size;
            view = _view;

            camera = new RhinoViewport(view.ActiveViewport);//ビューポートのカメラの設定を格納
        }

        /////////// 
        //methods// 
        /////////// 

        //ボックスを作るメソッド
        public void MakeBox()
        {
            box = new Box(Plane.WorldXY, new Interval(0, size), new Interval(0, size), new Interval(0, size));

            corner.AddRange(box.GetCorners());
        }

        //与えられた２つのベクトルに垂直な単位ベクトルを返す
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

        //与えられた２つのベクトルの角度を返す
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

        //与えられたポイントを４次行列式で計算できる行列で返す
        public double[,] PointToMatrix(Point3d point)
        {
            double[,] original_pos = new double[1, 4];
            original_pos[0, 0] = point.X;
            original_pos[0, 1] = point.Y;
            original_pos[0, 2] = point.Z;
            original_pos[0, 3] = 1;

            return original_pos;
        }

        //与えられた２つの行列の積を返す
        public static double[,] MultiplyMatrices(double[,] x, double[,] y)
        {
            double[,] z = new double[1, 4];
            for (int i = 0; i < 4; i++)
            {
                z[0, i] = 0;//ポイントの行列が１行とするため、列の値は変化しない
                for (int j = 0; j < 4; j++)
                {
                    z[0, i] += x[0, j] * y[j, i];
                }
            }
            return z;
        }

        //モデル座標から図面の座標へ変換
        public Point3d CoordinateTransformation(Point3d world_position)
        {
            double halfDiagonalAngle, halfVerticalAngle, halfHorizontalAngle , nearDistance;
            camera.GetCameraAngle(out halfDiagonalAngle, out halfVerticalAngle, out halfHorizontalAngle);

            ///////////////////////
            //view transformation//
            ///////////////////////

            //カメラのZ軸と図面のZ軸を揃える
            Vector3d vec_norm = NormalVector(camera.CameraZ, Vector3d.ZAxis);//回転の軸
            double angle = VectorAngle(-Vector3d.ZAxis, camera.CameraZ);//角度

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

            //カメラの上方向と図面のY軸を揃える
            double angle2 = VectorAngle(camera.CameraX, Vector3d.XAxis);

            double[,] m2 = new double[4, 4];
            m2[0, 0] = Math.Cos(angle2);
            m2[1, 0] = Math.Sin(angle2);
            m2[0, 1] = -Math.Sin(angle2);
            m2[1, 1] = Math.Cos(angle2);
            m2[2, 2] = 1;
            m2[3, 3] = 1;

            //カメラの位置を原点に揃える
            double[,] pos_cameralocation = MultiplyMatrices(MultiplyMatrices(PointToMatrix(camera.CameraLocation), m1), m2);//m1,m2による変換後のカメラの位置を求める
            Vector3d vec_location = new Vector3d(pos_cameralocation[0, 0], pos_cameralocation[0, 1], pos_cameralocation[0, 2]);

            double[,] m3 = new double[4, 4];
            m3[0, 0] = 1;
            m3[1, 1] = 1;
            m3[2, 2] = 1;
            m3[3, 0] = -vec_location.X;
            m3[3, 1] = -vec_location.Y;
            m3[3, 2] = -vec_location.Z;
            m3[3, 3] = 1;

            double[,] screen1 = MultiplyMatrices(PointToMatrix(world_position), m1);
            double[,] screen2 = MultiplyMatrices(screen1, m2);
            double[,] screen3 = MultiplyMatrices(screen2, m3);
            
            //////////////////////////////
            //purojection transformation//
            //////////////////////////////

            Plane planenear, planefar;
            camera.GetFrustumNearPlane(out planenear);
            camera.GetFrustumFarPlane(out planefar);

            double dis_near = camera.CameraLocation.DistanceTo(planenear.Origin);//カメラとNearPlaneの距離を求める
            double dis_far = camera.CameraLocation.DistanceTo(planefar.Origin);//カメラとFarPlaneの距離を求める

            //x座標,y座標の値によって正負を返す
            double direction_y = 1;
            double direction_x = 1;
            if (screen3[0, 1] < 0)
            {
                direction_y = -1; 
            }

            if (screen3[0, 0] < 0)
            {
                direction_x = -1;
            }
            
            double angle_y = VectorAngle(Vector3d.ZAxis, new Vector3d(0, screen3[0, 1], screen3[0, 2]));
            double angle_x = VectorAngle(Vector3d.ZAxis, new Vector3d(screen3[0, 0], 0, screen3[0, 2]));
            double sy = 1 - direction_y * ((screen3[0, 2] - dis_near) * Math.Tan(angle_y)) / screen3[0, 1];
            double sx = 1 - direction_x * ((screen3[0, 2] - dis_near) * Math.Tan(angle_x)) / screen3[0, 0];

            double[,] m4 = new double[4, 4];

            m4[0, 0] = sx;
            m4[1, 1] = sy;
            m4[2, 2] = 0;
            m4[3, 3] = 1;

            double[,] screen4 = MultiplyMatrices(screen3, m4);

            /////////////////////////
            //screen transformation//
            /////////////////////////

            //変換した座標をビューポートのサイズに合わせる
            double[,] m5 = new double[4, 4];

            List<Point3d> nearrec = new List<Point3d>();
            nearrec.AddRange(camera.GetNearRect());

            //NearPlaneとビューポートの大きさの比率を求める
            double sx5 = camera.Size.Width / nearrec[0].DistanceTo(nearrec[1]);
            double sy5 = camera.Size.Height / nearrec[0].DistanceTo(nearrec[2]);

            m5[0, 0] = sx5;
            m5[1, 1] = sy5;
            m5[3, 3] = 1;

            double[,] screen5 = MultiplyMatrices(screen4, m5);

            Point3d pos_screen = new Point3d(screen5[0, 0], screen5[0, 1], screen5[0, 2]);
            return pos_screen;
            }

        //オブジェクトのビューポート内での占有率を返す
        public void Area()
        {
            List<Point3d> vertex = new List<Point3d>(3);//box cornear
            List<Point3d> vertex_rec = new List<Point3d>();//viewrec
            List<Curve> triangle_list = new List<Curve>();

            List<Point3d> pos_viewcorner = new List<Point3d>();
            pos_viewcorner.AddRange(camera.GetNearRect());
            pos_viewcorner.Reverse(2, 2);
            pos_viewcorner.Add(pos_viewcorner[0]);

            //それぞれの座標を図面の座標へと変換する
            for (int i = 0; i < corner.Count; i++)
            {
                clientpos.Add(CoordinateTransformation(corner[i]));
            }

            for (int i = 0; i < pos_viewcorner.Count; i++)
            {
                vertex_rec.Add(CoordinateTransformation(pos_viewcorner[i]));
            }

            //面積を求めるために各点から３つ選択してできる三角形をすべて求める。
            for (int i = 0; i < clientpos.Count - 2; i++)
            {
                vertex.Clear();
                vertex.Add(clientpos[i]);
                vertex.Add(clientpos[i + 1]);

                for (int j = i + 2; j < clientpos.Count; j++)
                {
                    vertex.Add(clientpos[j]);
                    vertex.Add(clientpos[i]);
                    triangle_list.Add(Curve.CreateInterpolatedCurve(vertex, 1));
                    vertex.RemoveAt(3);
                    vertex.RemoveAt(2);
                }
            }

            enclose_view = Curve.CreateInterpolatedCurve(vertex_rec, 1);//画面の範囲を囲う
            Curve enclose_box = Curve.CreateBooleanUnion(triangle_list)[0];//ボックスの見えがかりを囲う
            enclose_intersect = Curve.CreateBooleanIntersection(enclose_box , enclose_view)[0];//画面上でボックスの見えている範囲

            double area_view = AreaMassProperties.Compute(enclose_view).Area;
            double area_intersect = AreaMassProperties.Compute(enclose_intersect).Area;
            double account = area_intersect / area_view * 100;
            RhinoApp.WriteLine(String.Format("The box accounts for {0}% of viewport", account));
        }
        
        public void Display(RhinoDoc _doc)
        {
            Brep objs = Brep.CreateFromBox(box);
            _doc.Objects.AddBrep(objs);
            for (int i = 0; i < clientpos.Count; i++)
            {
                _doc.Objects.AddPoint(clientpos[i]);
            }

            _doc.Objects.AddCurve(enclose_intersect);
            _doc.Objects.AddCurve(enclose_view);
        }
    }
}
