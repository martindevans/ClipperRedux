using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;




namespace ClipperRedux
{
    internal static class InternalHelpers
    {
        internal const double HORIZONTAL = -3.4E+38;
        internal const long LO_RANGE = 0x3FFFFFFF;
        internal const long HI_RANGE = 0x3FFFFFFFFFFFFFFFL;

        internal static void AddPolyNodeToPolygons(PolyNode polynode, ICollection<List<IntPoint>> polygons)
        {
            Contract.Requires(polynode != null);
            Contract.Requires(polygons != null);

            if (polynode.Contour.Count > 0)
                polygons.Add(polynode.Contour);
            foreach (var pn in polynode.Childs)
                AddPolyNodeToPolygons(pn, polygons);
        }

        internal static void Swap<T>(ref T a, ref T b)
        {
            var tmp = a;
            a = b;
            b = tmp;
        }

        internal static TEdge GetNextInAEL(TEdge e, Direction direction)
        {
            Contract.Requires(e != null);

            return direction == Direction.LeftToRight ? e.NextInAEL : e.PrevInAEL;
        }



        internal static bool IsMinima(TEdge e)
        {
            Contract.Requires(e != null);
            Contract.Requires(e.Prev != null);
            Contract.Requires(e.Next != null);

            return (e.Prev.NextInLML != e) && (e.Next.NextInLML != e);
        }



        internal static bool IsMaxima(TEdge e, double y)
        {
            Contract.Requires(e != null);

            return e.YTop == y && e.NextInLML == null;
        }



        internal static bool IsIntermediate(TEdge e, double y)
        {
            Contract.Requires(e != null);

            return e.YTop == y && e.NextInLML != null;
        }



        internal static TEdge GetMaximaPair(TEdge e)
        {
            Contract.Requires(e != null);
            Contract.Requires(e.Next != null);

            if (!IsMaxima(e.Next, e.YTop) || (e.Next.XTop != e.XTop))
                return e.Prev;
            else
                return e.Next;
        }

        internal static bool PointIsVertex(IntPoint pt, OutPt pp)
        {
            Contract.Requires(pp != null);

            var pp2 = pp;
            do
            {
                if (pp2.Pt.Equals(pt))
                    return true;
                pp2 = pp2.Next;
            } while (pp2 != pp);
            return false;
        }

        private static bool PointOnLineSegment(IntPoint pt, IntPoint linePt1, IntPoint linePt2)
        {
            return ((pt.X == linePt1.X) && (pt.Y == linePt1.Y)) ||
                    ((pt.X == linePt2.X) && (pt.Y == linePt2.Y)) ||
                    ((pt.X > linePt1.X == pt.X < linePt2.X) &&
                    (pt.Y > linePt1.Y == pt.Y < linePt2.Y) &&
                    new decimal(pt.X - linePt1.X) * new decimal(linePt2.Y - linePt1.Y) == new decimal(linePt2.X - linePt1.X) * new decimal(pt.Y - linePt1.Y));
        }

        internal static bool PointOnPolygon(IntPoint pt, OutPt pp)
        {
            Contract.Requires(pp != null);

            var pp2 = pp;
            while (true)
            {
                if (PointOnLineSegment(pt, pp2.Pt, pp2.Next.Pt))
                    return true;
                pp2 = pp2.Next;
                if (pp2 == pp)
                    break;
            }
            return false;
        }



        internal static bool PointInPolygon(IntPoint pt, OutPt pp)
        {
            Contract.Requires(pp != null);

            var pp2 = pp;
            var result = false;

            do
            {
                if ((((pp2.Pt.Y <= pt.Y) && (pt.Y < pp2.Prev.Pt.Y)) ||
                        ((pp2.Prev.Pt.Y <= pt.Y) && (pt.Y < pp2.Pt.Y))) &&
                    pt.X - pp2.Pt.X <
                    new decimal(pp2.Prev.Pt.X - pp2.Pt.X) * new decimal(pt.Y - pp2.Pt.Y) / new decimal(pp2.Prev.Pt.Y - pp2.Pt.Y))
                    result = !result;
                pp2 = pp2.Next;
            } while (pp2 != pp);

            return result;
        }



        internal static bool SlopesEqual(TEdge e1, TEdge e2)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e2 != null);

            return new decimal(e1.DeltaY) * new decimal(e2.DeltaX) == new decimal(e1.DeltaX) * new decimal(e2.DeltaY);
        }



        internal static bool SlopesEqual(IntPoint pt1, IntPoint pt2, IntPoint pt3)
        {
            return new decimal(pt1.Y - pt2.Y) * new decimal(pt2.X - pt3.X) == new decimal(pt1.X - pt2.X) * new decimal(pt2.Y - pt3.Y);
        }



        internal static bool SlopesEqual(IntPoint pt1, IntPoint pt2, IntPoint pt3, IntPoint pt4)
        {
            return new decimal(pt1.Y - pt2.Y) * new decimal(pt3.X - pt4.X) == new decimal(pt1.X - pt2.X) * new decimal(pt3.Y - pt4.Y);
        }

        



        internal static void RangeTest(IntPoint pt, ref long maxrange)
        {
            if (pt.X > maxrange)
            {
                if (pt.X > HI_RANGE)
                    throw new ClipperException("Coordinate exceeds range bounds");
                else
                    maxrange = HI_RANGE;
            }
            if (pt.Y > maxrange)
            {
                if (pt.Y > HI_RANGE)
                    throw new ClipperException("Coordinate exceeds range bounds");
                else
                    maxrange = HI_RANGE;
            }
        }

        internal static List<IntPoint> BuildArc(IntPoint pt, double a1, double a2, double r, double limit)
        {
            //see notes in clipper.pas regarding steps
            var arcFrac = Math.Abs(a2 - a1) / (2 * Math.PI);
            var steps = (int)(arcFrac * Math.PI / Math.Acos(1 - limit / Math.Abs(r)));
            if (steps < 2)
                steps = 2;
            else if (steps > (int)(222.0 * arcFrac))
                steps = (int)(222.0 * arcFrac);

            var x = Math.Cos(a1);
            var y = Math.Sin(a1);
            var c = Math.Cos((a2 - a1) / steps);
            var s = Math.Sin((a2 - a1) / steps);
            var result = new List<IntPoint>(steps + 1);
            for (var i = 0; i <= steps; ++i)
            {
                result.Add(new IntPoint(pt.X + Round(x * r), pt.Y + Round(y * r)));
                var x2 = x;
                x = x * c - s * y; //cross product
                y = x2 * s + y * c; //dot product
            }
            return result;
        }

        internal static long Round(double value)
        {
            return value < 0 ? (long)(value - 0.5) : (long)(value + 0.5);
        }

        private static DoublePoint ClosestPointOnLine(IntPoint pt, IntPoint linePt1, IntPoint linePt2)
        {
            var dx = (double)linePt2.X - linePt1.X;
            var dy = (double)linePt2.Y - linePt1.Y;
            if (dx == 0 && dy == 0)
                return new DoublePoint(linePt1.X, linePt1.Y);

            var q = ((pt.X - linePt1.X) * dx + (pt.Y - linePt1.Y) * dy) / (dx * dx + dy * dy);
            return new DoublePoint(
                (1 - q) * linePt1.X + q * linePt2.X,
                (1 - q) * linePt1.Y + q * linePt2.Y
            );
        }

        internal static bool SlopesNearColinear(IntPoint pt1, IntPoint pt2, IntPoint pt3, double distSqrd)
        {
            if (pt1.DistanceSquared(pt2) > pt1.DistanceSquared(pt3))
                return false;
            var cpol = ClosestPointOnLine(pt2, pt1, pt3);
            var dx = pt2.X - cpol.X;
            var dy = pt2.Y - cpol.Y;
            return dx * dx + dy * dy < distSqrd;
        }

        internal static bool FirstIsBottomPt(OutPt btmPt1, OutPt btmPt2)
        {
            Contract.Requires(btmPt1 != null);
            Contract.Requires(btmPt1.Prev != null);

            var p = btmPt1.Prev;
            while (p.Pt.Equals(btmPt1.Pt) && (p != btmPt1))
                p = p.Prev;
            var dx1P = Math.Abs(btmPt1.Pt.Dx(p.Pt));
            p = btmPt1.Next;
            while (p.Pt.Equals(btmPt1.Pt) && (p != btmPt1))
                p = p.Next;
            var dx1N = Math.Abs(btmPt1.Pt.Dx(p.Pt));

            p = btmPt2.Prev;
            while (p.Pt.Equals(btmPt2.Pt) && (p != btmPt2))
                p = p.Prev;
            var dx2P = Math.Abs(btmPt2.Pt.Dx(p.Pt));
            p = btmPt2.Next;
            while (p.Pt.Equals(btmPt2.Pt) && (p != btmPt2))
                p = p.Next;
            var dx2N = Math.Abs(btmPt2.Pt.Dx(p.Pt));
            return (dx1P >= dx2P && dx1P >= dx2N) || (dx1N >= dx2P && dx1N >= dx2N);
        }
    }
}
