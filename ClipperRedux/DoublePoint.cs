
namespace ClipperRedux
{
    internal struct DoublePoint
    {
        private readonly double _x;
        public double X
        {
            get { return _x; }
        }

        private readonly double _y;
        public double Y
        {
            get { return _y; }
        }

        public DoublePoint(double x, double y)
        {
            _x = x;
            _y = y;
        }
    }
}
