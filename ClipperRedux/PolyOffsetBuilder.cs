using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;

namespace ClipperRedux
{
    public class PolyOffsetBuilder
    {
        private readonly IReadOnlyList<IReadOnlyList<IntPoint>> _p;
        private readonly List<IntPoint> _currentPoly;
        private readonly List<DoublePoint> _normals = new List<DoublePoint>();
        private readonly double _delta;
        private double _r;
        private readonly double _rmin;
        private readonly int _i;
        private readonly int _j;
        private int _k;

        private void OffsetPoint(JoinType jointype, double limit)
        {
            switch (jointype)
            {
                case JoinType.Miter:
                    {
                        _r = 1 + (_normals[_j].X * _normals[_k].X +
                                  _normals[_j].Y * _normals[_k].Y);
                        if (_r >= _rmin)
                            DoMiter();
                        else
                            DoSquare();
                        break;
                    }
                case JoinType.Square:
                    DoSquare();
                    break;
                case JoinType.Round:
                    DoRound(limit);
                    break;
                default:
                    throw new ArgumentOutOfRangeException("jointype", string.Format("Unknown join type '{0}'", jointype));
            }
            _k = _j;
        }

        public static void Offset(IReadOnlyList<IReadOnlyList<IntPoint>> points, List<List<IntPoint>> solution, bool isPolygon, double delta, JoinType jointype, EndType endtype, double limit = 0)
        {
            Contract.Requires(points != null);
            Contract.Requires(solution != null);

            //This is ugly as hell! The constructor does all the work, mutates one of it's arguments and returns a useless object (the builder has already been used by this point).

            // ReSharper disable once ObjectCreationAsStatement
            new PolyOffsetBuilder(points, solution, isPolygon, delta, jointype, endtype, limit);
        }

        private PolyOffsetBuilder(IReadOnlyList<IReadOnlyList<IntPoint>> points, List<List<IntPoint>> solution, bool isPolygon, double delta, JoinType jointype, EndType endtype, double limit = 0)
        {
            Contract.Requires(points != null);
            Contract.Requires(solution != null);

            if (ReferenceEquals(solution, points))
                throw new ArgumentException("Input and Output parameters cannot be the same", "solution");
            if (delta == 0)
                return;

            _p = points;
            _delta = delta;
            _rmin = 0.5;

            if (jointype == JoinType.Miter)
            {
                if (limit > 2)
                    _rmin = 2.0 / (limit * limit);
                limit = 0.25; //just in case endtype == etRound
            }
            else
            {
                if (limit <= 0)
                    limit = 0.25;
                else if (limit > Math.Abs(delta))
                    limit = Math.Abs(delta);
            }

            solution.Clear();
            for (_i = 0; _i < points.Count; _i++)
            {
                var len = points[_i].Count;
                if (len == 0 || (len < 3 && delta <= 0))
                    continue;
                else if (len == 1)
                {
                    _currentPoly = new List<IntPoint>();
                    _currentPoly = InternalHelpers.BuildArc(points[_i][0], 0, 2 * Math.PI, delta, limit);
                    solution.Add(_currentPoly);
                    continue;
                }

                var forceClose = points[_i][0].Equals(points[_i][len - 1]);
                if (forceClose)
                    len--;

                //build normals ...
                _normals.Clear();
                for (var j = 0; j < len - 1; ++j)
                    _normals.Add(points[_i][j].UnitNormal(points[_i][j + 1]));
                if (isPolygon || forceClose)
                    _normals.Add(points[_i][len - 1].UnitNormal(points[_i][0]));
                else
                    _normals.Add(_normals[len - 2]);

                _currentPoly = new List<IntPoint>();
                if (isPolygon || forceClose)
                {
                    _k = len - 1;
                    for (_j = 0; _j < len; ++_j)
                        OffsetPoint(jointype, limit);
                    solution.Add(_currentPoly);
                    if (!isPolygon)
                    {
                        _currentPoly = new List<IntPoint>();
                        _delta = -_delta;
                        _k = len - 1;
                        for (_j = 0; _j < len; ++_j)
                            OffsetPoint(jointype, limit);
                        _delta = -_delta;
                        _currentPoly.Reverse();
                        solution.Add(_currentPoly);
                    }
                }
                else
                {
                    _k = 0;
                    for (_j = 1; _j < len - 1; ++_j)
                        OffsetPoint(jointype, limit);

                    IntPoint pt1;
                    if (endtype == EndType.Butt)
                    {
                        _j = len - 1;
                        pt1 = new IntPoint(InternalHelpers.Round(points[_i][_j].X + _normals[_j].X * delta), InternalHelpers.Round(points[_i][_j].Y + _normals[_j].Y * delta));
                        AddPoint(pt1);
                        pt1 = new IntPoint(InternalHelpers.Round(points[_i][_j].X - _normals[_j].X * delta), InternalHelpers.Round(points[_i][_j].Y - _normals[_j].Y * delta));
                        AddPoint(pt1);
                    }
                    else
                    {
                        _j = len - 1;
                        _k = len - 2;
                        _normals[_j] = new DoublePoint(-_normals[_j].X, -_normals[_j].Y);

                        if (endtype == EndType.Square)
                            DoSquare();
                        else
                            DoRound(limit);
                    }

                    //re-build Normals ...
                    for (var j = len - 1; j > 0; j--)
                    {
                        _normals[_j] = new DoublePoint(-_normals[j - 1].X, -_normals[j - 1].Y);
                    }
                    _normals[0] = new DoublePoint(-_normals[1].X, -_normals[1].Y);

                    _k = len - 1;
                    for (_j = _k - 1; _j > 0; --_j)
                        OffsetPoint(jointype, limit);

                    if (endtype == EndType.Butt)
                    {
                        pt1 = new IntPoint(InternalHelpers.Round(points[_i][0].X - _normals[0].X * delta), InternalHelpers.Round(points[_i][0].Y - _normals[0].Y * delta));
                        AddPoint(pt1);
                        pt1 = new IntPoint(InternalHelpers.Round(points[_i][0].X + _normals[0].X * delta), InternalHelpers.Round(points[_i][0].Y + _normals[0].Y * delta));
                        AddPoint(pt1);
                    }
                    else
                    {
                        _k = 1;
                        if (endtype == EndType.Square)
                            DoSquare();
                        else
                            DoRound(limit);
                    }
                    solution.Add(_currentPoly);
                }
            }

            //finally, clean up untidy corners ...
            var clpr = new Clipper();
            clpr.AddPolygons(solution, PolyType.Subject);
            if (delta > 0)
            {
                clpr.Execute(ClipType.Union, solution, PolyFillType.Positive, PolyFillType.Positive);
            }
            else
            {
                var r = clpr.GetBounds();
                var outer = new List<IntPoint>(4) {
                    new IntPoint(r.Left - 10, r.Bottom + 10),
                    new IntPoint(r.Right + 10, r.Bottom + 10),
                    new IntPoint(r.Right + 10, r.Top - 10),
                    new IntPoint(r.Left - 10, r.Top - 10)
                };

                clpr.AddPolygon(outer, PolyType.Subject);
                clpr.ReverseSolution = true;
                clpr.Execute(ClipType.Union, solution, PolyFillType.Negative, PolyFillType.Negative);
                if (solution.Count > 0)
                    solution.RemoveAt(0);
            }
        }

        [ContractInvariantMethod]
        private void ContractInvariant()
        {
            Contract.Invariant(_p != null);
            Contract.Invariant(_normals != null);
        }

        private void AddPoint(IntPoint pt)
        {
            Contract.Requires(_currentPoly != null);

            _currentPoly.Add(pt);
        }

        private void DoSquare()
        {
            Contract.Requires(_currentPoly != null);
            Contract.Requires(_k >= 0 && _k < _normals.Count);
            Contract.Requires(_j >= 0 && _j < _normals.Count);

            var pt1 = new IntPoint(InternalHelpers.Round(_p[_i][_j].X + _normals[_k].X * _delta), InternalHelpers.Round(_p[_i][_j].Y + _normals[_k].Y * _delta));
            var pt2 = new IntPoint(InternalHelpers.Round(_p[_i][_j].X + _normals[_j].X * _delta), InternalHelpers.Round(_p[_i][_j].Y + _normals[_j].Y * _delta));
            if ((_normals[_k].X * _normals[_j].Y - _normals[_j].X * _normals[_k].Y) * _delta >= 0)
            {
                var a1 = Math.Atan2(_normals[_k].Y, _normals[_k].X);
                var a2 = Math.Atan2(-_normals[_j].Y, -_normals[_j].X);
                a1 = Math.Abs(a2 - a1);
                if (a1 > Math.PI)
                    a1 = Math.PI * 2 - a1;
                var dx = Math.Tan((Math.PI - a1) / 4) * Math.Abs(_delta);
                pt1 = new IntPoint((long)(pt1.X - _normals[_k].Y * dx),
                    (long)(pt1.Y + _normals[_k].X * dx));
                AddPoint(pt1);
                pt2 = new IntPoint((long)(pt2.X + _normals[_j].Y * dx),
                    (long)(pt2.Y - _normals[_j].X * dx));
                AddPoint(pt2);
            }
            else
            {
                AddPoint(pt1);
                AddPoint(_p[_i][_j]);
                AddPoint(pt2);
            }
        }

        private void DoMiter()
        {
            Contract.Requires(_currentPoly != null);
            Contract.Requires(_k >= 0 && _k < _normals.Count);
            Contract.Requires(_j >= 0 && _j < _normals.Count);

            if ((_normals[_k].X * _normals[_j].Y - _normals[_j].X * _normals[_k].Y) * _delta >= 0)
            {
                var q = _delta / _r;
                AddPoint(new IntPoint(InternalHelpers.Round(_p[_i][_j].X + (_normals[_k].X + _normals[_j].X) * q), InternalHelpers.Round(_p[_i][_j].Y + (_normals[_k].Y + _normals[_j].Y) * q)));
            }
            else
            {
                var pt1 = new IntPoint(InternalHelpers.Round(_p[_i][_j].X + _normals[_k].X * _delta), InternalHelpers.Round(_p[_i][_j].Y + _normals[_k].Y * _delta));
                var pt2 = new IntPoint(InternalHelpers.Round(_p[_i][_j].X + _normals[_j].X * _delta), InternalHelpers.Round(_p[_i][_j].Y + _normals[_j].Y * _delta));
                AddPoint(pt1);
                AddPoint(_p[_i][_j]);
                AddPoint(pt2);
            }
        }

        private void DoRound(double limit)
        {
            Contract.Requires(_currentPoly != null);
            Contract.Requires(_k >= 0 && _k < _normals.Count);
            Contract.Requires(_j >= 0 && _j < _normals.Count);

            var pt1 = new IntPoint(InternalHelpers.Round(_p[_i][_j].X + _normals[_k].X * _delta), InternalHelpers.Round(_p[_i][_j].Y + _normals[_k].Y * _delta));
            var pt2 = new IntPoint(InternalHelpers.Round(_p[_i][_j].X + _normals[_j].X * _delta), InternalHelpers.Round(_p[_i][_j].Y + _normals[_j].Y * _delta));
            AddPoint(pt1);
            //round off reflex angles (ie > 180 deg) unless almost flat (ie < 10deg).
            //cross product normals < 0 . angle > 180 deg.
            //dot product normals == 1 . no angle
            if ((_normals[_k].X * _normals[_j].Y - _normals[_j].X * _normals[_k].Y) * _delta >= 0)
            {
                if (_normals[_j].X * _normals[_k].X + _normals[_j].Y * _normals[_k].Y < 0.985)
                {
                    var a1 = Math.Atan2(_normals[_k].Y, _normals[_k].X);
                    var a2 = Math.Atan2(_normals[_j].Y, _normals[_j].X);
                    if (_delta > 0 && a2 < a1)
                        a2 += Math.PI * 2;
                    else if (_delta < 0 && a2 > a1)
                        a2 -= Math.PI * 2;
                    var arc = InternalHelpers.BuildArc(_p[_i][_j], a1, a2, _delta, limit);
                    foreach (var point in arc)
                        AddPoint(point);
                }
            }
            else
                AddPoint(_p[_i][_j]);
            AddPoint(pt2);
        }
    }
}
