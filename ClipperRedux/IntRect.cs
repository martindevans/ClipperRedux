
namespace ClipperRedux
{
    public struct IntRect
    {
        public long Left;
        public long Top;
        public long Right;
        public long Bottom;

        public IntRect(long l, long t, long r, long b)
        {
            Left = l;
            Top = t;
            Right = r;
            Bottom = b;
        }
    }
}
