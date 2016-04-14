//using System;
//using System.Collections.Generic;

//namespace ClipperRedux
//{
//    public class ClipperOffset
//    {
//        private List<List<IntPoint>> _destPolys;
//        private List<IntPoint> _srcPoly;
//        private List<IntPoint> _destPoly;
//        private readonly List<DoublePoint> _normals = new List<DoublePoint>();
//        private double _delta, _sinA, _sin, _cos;
//        private double _miterLim, _stepsPerRad;

//        private IntPoint _lowest;
//        private PolyNode _polyNodes = new PolyNode();

//        public double ArcTolerance { get; set; }
//        public double MiterLimit { get; set; }

//        private const double TWO_PI = Math.PI * 2;
//        private const double DEF_ARC_TOLERANCE = 0.25;

//        public ClipperOffset(double miterLimit = 2.0, double arcTolerance = DEF_ARC_TOLERANCE)
//        {
//            MiterLimit = miterLimit;
//            ArcTolerance = arcTolerance;
//            _lowest.X = -1;
//        }

//        public void Clear()
//        {
//            _polyNodes.Childs.Clear();
//            _lowest.X = -1;
//        }

//        internal static long Round(double value)
//        {
//            return value < 0 ? (long)(value - 0.5) : (long)(value + 0.5);
//        }
//        //------------------------------------------------------------------------------

//        public void AddPath(List<IntPoint> path, JoinType joinType, EndType endType)
//        {
//            var highI = path.Count - 1;
//            if (highI < 0) return;
//            var newNode = new PolyNode();
//            newNode.m_jointype = joinType;
//            newNode.m_endtype = endType;

//            //strip duplicate points from path and also get index to the lowest point ...
//            if (endType == EndType.ClosedLine || endType == EndType.ClosedPolygon)
//                while (highI > 0 && path[0] == path[highI]) highI--;
//            newNode.m_polygon.Capacity = highI + 1;
//            newNode.m_polygon.Add(path[0]);
//            int j = 0, k = 0;
//            for (var i = 1; i <= highI; i++)
//                if (newNode.m_polygon[j] != path[i])
//                {
//                    j++;
//                    newNode.m_polygon.Add(path[i]);
//                    if (path[i].Y > newNode.m_polygon[k].Y ||
//                      (path[i].Y == newNode.m_polygon[k].Y &&
//                      path[i].X < newNode.m_polygon[k].X)) k = j;
//                }
//            if (endType == EndType.ClosedPolygon && j < 2) return;

//            _polyNodes.AddChild(newNode);

//            //if this path's lowest pt is lower than all the others then update m_lowest
//            if (endType != EndType.ClosedPolygon) return;
//            if (_lowest.X < 0)
//                _lowest = new IntPoint(_polyNodes.ChildCount - 1, k);
//            else
//            {
//                IntPoint ip = _polyNodes.Childs[(int)_lowest.X].m_polygon[(int)_lowest.Y];
//                if (newNode.m_polygon[k].Y > ip.Y ||
//                  (newNode.m_polygon[k].Y == ip.Y &&
//                  newNode.m_polygon[k].X < ip.X))
//                    _lowest = new IntPoint(_polyNodes.ChildCount - 1, k);
//            }
//        }

//        public void AddPaths(List<List<IntPoint>> paths, JoinType joinType, EndType endType)
//        {
//            foreach (var p in paths)
//                AddPath(p, joinType, endType);
//        }

//        private void FixOrientations()
//        {
//            //fixup orientations of all closed paths if the orientation of the
//            //closed path with the lowermost vertex is wrong ...
//            if (_lowest.X >= 0 &&
//              !Clipper.Orientation(_polyNodes.Childs[(int)_lowest.X].m_polygon))
//            {
//                for (var i = 0; i < _polyNodes.ChildCount; i++)
//                {
//                    PolyNode node = _polyNodes.Childs[i];
//                    if (node.m_endtype == EndType.ClosedPolygon ||
//                      (node.m_endtype == EndType.ClosedLine &&
//                      Clipper.Orientation(node.m_polygon)))
//                        node.m_polygon.Reverse();
//                }
//            }
//            else
//            {
//                for (var i = 0; i < _polyNodes.ChildCount; i++)
//                {
//                    PolyNode node = _polyNodes.Childs[i];
//                    if (node.m_endtype == EndType.ClosedLine &&
//                      !Clipper.Orientation(node.m_polygon))
//                        node.m_polygon.Reverse();
//                }
//            }
//        }
//        //------------------------------------------------------------------------------

//        internal static DoublePoint GetUnitNormal(IntPoint pt1, IntPoint pt2)
//        {
//            double dx = (pt2.X - pt1.X);
//            double dy = (pt2.Y - pt1.Y);
//            if ((dx == 0) && (dy == 0)) return new DoublePoint();

//            var f = 1 * 1.0 / Math.Sqrt(dx * dx + dy * dy);
//            dx *= f;
//            dy *= f;

//            return new DoublePoint(dy, -dx);
//        }
//        //------------------------------------------------------------------------------

//        private void DoOffset(double delta)
//        {
//            _destPolys = new List<List<IntPoint>>();
//            _delta = delta;

//            //if Zero offset, just copy any CLOSED polygons to m_p and return ...
//            if (ClipperBase.near_zero(delta))
//            {
//                _destPolys.Capacity = _polyNodes.ChildCount;
//                for (var i = 0; i < _polyNodes.ChildCount; i++)
//                {
//                    PolyNode node = _polyNodes.Childs[i];
//                    if (node.m_endtype == EndType.ClosedPolygon)
//                        _destPolys.Add(node.m_polygon);
//                }
//                return;
//            }

//            //see offset_triginometry3.svg in the documentation folder ...
//            if (MiterLimit > 2) _miterLim = 2 / (MiterLimit * MiterLimit);
//            else _miterLim = 0.5;

//            double y;
//            if (ArcTolerance <= 0.0)
//                y = DEF_ARC_TOLERANCE;
//            else if (ArcTolerance > Math.Abs(delta) * DEF_ARC_TOLERANCE)
//                y = Math.Abs(delta) * DEF_ARC_TOLERANCE;
//            else
//                y = ArcTolerance;
//            //see offset_triginometry2.svg in the documentation folder ...
//            var steps = Math.PI / Math.Acos(1 - y / Math.Abs(delta));
//            _sin = Math.Sin(TWO_PI / steps);
//            _cos = Math.Cos(TWO_PI / steps);
//            _stepsPerRad = steps / TWO_PI;
//            if (delta < 0.0) _sin = -_sin;

//            _destPolys.Capacity = _polyNodes.ChildCount * 2;
//            for (var i = 0; i < _polyNodes.ChildCount; i++)
//            {
//                PolyNode node = _polyNodes.Childs[i];
//                _srcPoly = node.m_polygon;

//                var len = _srcPoly.Count;

//                if (len == 0 || (delta <= 0 && (len < 3 ||
//                  node.m_endtype != EndType.ClosedPolygon)))
//                    continue;

//                _destPoly = new List<IntPoint>();

//                if (len == 1)
//                {
//                    if (node.m_jointype == JoinType.Round)
//                    {
//                        double x = 1.0, Y = 0.0;
//                        for (var j = 1; j <= steps; j++)
//                        {
//                            _destPoly.Add(new IntPoint(
//                              Round(_srcPoly[0].X + x * delta),
//                              Round(_srcPoly[0].Y + Y * delta)));
//                            var x2 = x;
//                            x = x * _cos - _sin * Y;
//                            Y = x2 * _sin + Y * _cos;
//                        }
//                    }
//                    else
//                    {
//                        double x = -1.0, Y = -1.0;
//                        for (var j = 0; j < 4; ++j)
//                        {
//                            _destPoly.Add(new IntPoint(
//                              Round(_srcPoly[0].X + x * delta),
//                              Round(_srcPoly[0].Y + Y * delta)));
//                            if (x < 0) x = 1;
//                            else if (Y < 0) Y = 1;
//                            else x = -1;
//                        }
//                    }
//                    _destPolys.Add(_destPoly);
//                    continue;
//                }

//                //build m_normals ...
//                _normals.Clear();
//                _normals.Capacity = len;
//                for (var j = 0; j < len - 1; j++)
//                    _normals.Add(GetUnitNormal(_srcPoly[j], _srcPoly[j + 1]));
//                if (node.m_endtype == EndType.ClosedLine ||
//                  node.m_endtype == EndType.ClosedPolygon)
//                    _normals.Add(GetUnitNormal(_srcPoly[len - 1], _srcPoly[0]));
//                else
//                    _normals.Add(new DoublePoint(_normals[len - 2]));

//                if (node.m_endtype == EndType.ClosedPolygon)
//                {
//                    var k = len - 1;
//                    for (var j = 0; j < len; j++)
//                        OffsetPoint(j, ref k, node.m_jointype);
//                    _destPolys.Add(_destPoly);
//                }
//                else if (node.m_endtype == EndType.ClosedLine)
//                {
//                    var k = len - 1;
//                    for (var j = 0; j < len; j++)
//                        OffsetPoint(j, ref k, node.m_jointype);
//                    _destPolys.Add(_destPoly);
//                    _destPoly = new List<IntPoint>();
//                    //re-build m_normals ...
//                    var n = _normals[len - 1];
//                    for (var j = len - 1; j > 0; j--)
//                        _normals[j] = new DoublePoint(-_normals[j - 1].X, -_normals[j - 1].Y);
//                    _normals[0] = new DoublePoint(-n.X, -n.Y);
//                    k = 0;
//                    for (var j = len - 1; j >= 0; j--)
//                        OffsetPoint(j, ref k, node.m_jointype);
//                    _destPolys.Add(_destPoly);
//                }
//                else
//                {
//                    var k = 0;
//                    for (var j = 1; j < len - 1; ++j)
//                        OffsetPoint(j, ref k, node.m_jointype);

//                    IntPoint pt1;
//                    if (node.m_endtype == EndType.OpenButt)
//                    {
//                        var j = len - 1;
//                        pt1 = new IntPoint((cInt)Round(_srcPoly[j].X + _normals[j].X *
//                          delta), (cInt)Round(_srcPoly[j].Y + _normals[j].Y * delta));
//                        _destPoly.Add(pt1);
//                        pt1 = new IntPoint((cInt)Round(_srcPoly[j].X - _normals[j].X *
//                          delta), (cInt)Round(_srcPoly[j].Y - _normals[j].Y * delta));
//                        _destPoly.Add(pt1);
//                    }
//                    else
//                    {
//                        var j = len - 1;
//                        k = len - 2;
//                        _sinA = 0;
//                        _normals[j] = new DoublePoint(-_normals[j].X, -_normals[j].Y);
//                        if (node.m_endtype == EndType.OpenSquare)
//                            DoSquare(j, k);
//                        else
//                            DoRound(j, k);
//                    }

//                    //re-build m_normals ...
//                    for (var j = len - 1; j > 0; j--)
//                        _normals[j] = new DoublePoint(-_normals[j - 1].X, -_normals[j - 1].Y);

//                    _normals[0] = new DoublePoint(-_normals[1].X, -_normals[1].Y);

//                    k = len - 1;
//                    for (var j = k - 1; j > 0; --j)
//                        OffsetPoint(j, ref k, node.m_jointype);

//                    if (node.m_endtype == EndType.OpenButt)
//                    {
//                        pt1 = new IntPoint((cInt)Round(_srcPoly[0].X - _normals[0].X * delta),
//                          (cInt)Round(_srcPoly[0].Y - _normals[0].Y * delta));
//                        _destPoly.Add(pt1);
//                        pt1 = new IntPoint((cInt)Round(_srcPoly[0].X + _normals[0].X * delta),
//                          (cInt)Round(_srcPoly[0].Y + _normals[0].Y * delta));
//                        _destPoly.Add(pt1);
//                    }
//                    else
//                    {
//                        k = 1;
//                        _sinA = 0;
//                        if (node.m_endtype == EndType.OpenSquare)
//                            DoSquare(0, 1);
//                        else
//                            DoRound(0, 1);
//                    }
//                    _destPolys.Add(_destPoly);
//                }
//            }
//        }
//        //------------------------------------------------------------------------------

//        public void Execute(ref List<List<IntPoint>> solution, double delta)
//        {
//            solution.Clear();
//            FixOrientations();
//            DoOffset(delta);
//            //now clean up 'corners' ...
//            var clpr = new Clipper();
//            clpr.AddPaths(_destPolys, PolyType.Subject, true);
//            if (delta > 0)
//            {
//                clpr.Execute(ClipType.Union, solution, PolyFillType.Positive, PolyFillType.Positive);
//            }
//            else
//            {
//                var r = Clipper.GetBounds(_destPolys);
//                var outer = new List<IntPoint>(4);

//                outer.Add(new IntPoint(r.Left - 10, r.Bottom + 10));
//                outer.Add(new IntPoint(r.Right + 10, r.Bottom + 10));
//                outer.Add(new IntPoint(r.Right + 10, r.Top - 10));
//                outer.Add(new IntPoint(r.Left - 10, r.Top - 10));

//                clpr.AddPath(outer, PolyType.Subject, true);
//                clpr.ReverseSolution = true;
//                clpr.Execute(ClipType.Union, solution, PolyFillType.Negative, PolyFillType.Negative);
//                if (solution.Count > 0) solution.RemoveAt(0);
//            }
//        }

//        public void Execute(ref PolyTree solution, double delta)
//        {
//            solution.Clear();
//            FixOrientations();
//            DoOffset(delta);

//            //now clean up 'corners' ...
//            var clpr = new Clipper();
//            clpr.AddPaths(_destPolys, PolyType.ptSubject, true);
//            if (delta > 0)
//            {
//                clpr.Execute(ClipType.ctUnion, solution,
//                  PolyFillType.pftPositive, PolyFillType.pftPositive);
//            }
//            else
//            {
//                var r = Clipper.GetBounds(_destPolys);
//                var outer = new List<IntPoint>(4);

//                outer.Add(new IntPoint(r.Left - 10, r.Bottom + 10));
//                outer.Add(new IntPoint(r.Right + 10, r.Bottom + 10));
//                outer.Add(new IntPoint(r.Right + 10, r.Top - 10));
//                outer.Add(new IntPoint(r.Left - 10, r.Top - 10));

//                clpr.AddPath(outer, PolyType.Subject, true);
//                clpr.ReverseSolution = true;
//                clpr.Execute(ClipType.Union, solution, PolyFillType.Negative, PolyFillType.Negative);
//                //remove the outer PolyNode rectangle ...
//                if (solution.ChildCount == 1 && solution.Childs[0].ChildCount > 0)
//                {
//                    PolyNode outerNode = solution.Childs[0];
//                    solution.Childs.Capacity = outerNode.ChildCount;
//                    solution.Childs[0] = outerNode.Childs[0];
//                    solution.Childs[0].m_Parent = solution;
//                    for (var i = 1; i < outerNode.ChildCount; i++)
//                        solution.AddChild(outerNode.Childs[i]);
//                }
//                else
//                    solution.Clear();
//            }
//        }

//        private void OffsetPoint(int j, ref int k, JoinType jointype)
//        {
//            //cross product ...
//            _sinA = (_normals[k].X * _normals[j].Y - _normals[j].X * _normals[k].Y);

//            if (Math.Abs(_sinA * _delta) < 1.0)
//            {
//                //dot product ...
//                var cosA = (_normals[k].X * _normals[j].X + _normals[j].Y * _normals[k].Y);
//                if (cosA > 0) // angle ==> 0 degrees
//                {
//                    _destPoly.Add(new IntPoint(Round(_srcPoly[j].X + _normals[k].X * _delta),
//                      Round(_srcPoly[j].Y + _normals[k].Y * _delta)));
//                    return;
//                }
//                //else angle ==> 180 degrees   
//            }
//            else if (_sinA > 1.0) _sinA = 1.0;
//            else if (_sinA < -1.0) _sinA = -1.0;

//            if (_sinA * _delta < 0)
//            {
//                _destPoly.Add(new IntPoint(Round(_srcPoly[j].X + _normals[k].X * _delta),
//                  Round(_srcPoly[j].Y + _normals[k].Y * _delta)));
//                _destPoly.Add(_srcPoly[j]);
//                _destPoly.Add(new IntPoint(Round(_srcPoly[j].X + _normals[j].X * _delta),
//                  Round(_srcPoly[j].Y + _normals[j].Y * _delta)));
//            }
//            else
//                switch (jointype)
//                {
//                    case JoinType.Miter:
//                        {
//                            var r = 1 + (_normals[j].X * _normals[k].X +
//                              _normals[j].Y * _normals[k].Y);
//                            if (r >= _miterLim) DoMiter(j, k, r); else DoSquare(j, k);
//                            break;
//                        }
//                    case JoinType.Square: DoSquare(j, k); break;
//                    case JoinType.Round: DoRound(j, k); break;
//                }
//            k = j;
//        }

//        private void DoSquare(int j, int k)
//        {
//            var dx = Math.Tan(Math.Atan2(_sinA,
//                _normals[k].X * _normals[j].X + _normals[k].Y * _normals[j].Y) / 4);
//            _destPoly.Add(new IntPoint(
//                Round(_srcPoly[j].X + _delta * (_normals[k].X - _normals[k].Y * dx)),
//                Round(_srcPoly[j].Y + _delta * (_normals[k].Y + _normals[k].X * dx))));
//            _destPoly.Add(new IntPoint(
//                Round(_srcPoly[j].X + _delta * (_normals[j].X + _normals[j].Y * dx)),
//                Round(_srcPoly[j].Y + _delta * (_normals[j].Y - _normals[j].X * dx))));
//        }

//        private void DoMiter(int j, int k, double r)
//        {
//            var q = _delta / r;
//            _destPoly.Add(new IntPoint(Round(_srcPoly[j].X + (_normals[k].X + _normals[j].X) * q),
//                Round(_srcPoly[j].Y + (_normals[k].Y + _normals[j].Y) * q)));
//        }

//        private void DoRound(int j, int k)
//        {
//            var a = Math.Atan2(_sinA,
//            _normals[k].X * _normals[j].X + _normals[k].Y * _normals[j].Y);
//            var steps = Math.Max((int)Round(_stepsPerRad * Math.Abs(a)), 1);

//            double x = _normals[k].X, y = _normals[k].Y, x2;
//            for (var i = 0; i < steps; ++i)
//            {
//                _destPoly.Add(new IntPoint(
//                    Round(_srcPoly[j].X + x * _delta),
//                    Round(_srcPoly[j].Y + y * _delta)));
//                x2 = x;
//                x = x * _cos - _sin * y;
//                y = x2 * _sin + y * _cos;
//            }
//            _destPoly.Add(new IntPoint(
//            Round(_srcPoly[j].X + _normals[j].X * _delta),
//            Round(_srcPoly[j].Y + _normals[j].Y * _delta)));
//        }
//    }
//}
