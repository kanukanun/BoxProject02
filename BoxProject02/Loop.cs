using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Rhino;
using Rhino.Input;
using Rhino.Geometry;
using Rhino.Display;

namespace _5.Classes
{
    class Loop : Rhino_Processing
    {
        BoxProject objs;
        public override void Setup()
        {
            RhinoView view;
            RhinoGet.GetView("select viewport", out view);
            objs = new BoxProject(100 , view);

            objs.MakeBox();
            objs.Area();
        }
        
        public override void Draw()
        {
            objs.Display(doc);
        }
    }
}