using System;


namespace ClipperRedux
{
    public struct IntPoint
        : IEquatable<IntPoint>
    {
        public long X;
        public long Y;

        public IntPoint(long x, long y)
        {
            X = x;
            Y = y;
        }

        public bool Equals(IntPoint other)
        {
            return X == other.X && Y == other.Y;
        }

        public bool IsClose(IntPoint other, double distSqrd)
        {
            var dx = (double)X - other.X;
            var dy = (double)Y - other.Y;
            return dx * dx + dy * dy <= distSqrd;
        }

        public double DistanceSquared(IntPoint pt2)
        {
            var dx = (double)X - pt2.X;
            var dy = (double)Y - pt2.Y;
            return dx * dx + dy * dy;
        }

        public double Dx(IntPoint pt2)
        {
            if (Y == pt2.Y)
                return InternalHelpers.HORIZONTAL;
            else
                return (double)(pt2.X - X) / (pt2.Y - Y);
        }

        internal DoublePoint UnitNormal(IntPoint pt2)
        {
            double dx = pt2.X - X;
            double dy = pt2.Y - Y;

            if ((dx == 0) && (dy == 0))
                return new DoublePoint(0, 0);

            var f = 1 * 1.0 / Math.Sqrt(dx * dx + dy * dy);
            dx *= f;
            dy *= f;

            return new DoublePoint(dy, -dx);
        }
    }
}
