using System;


namespace ClipperRedux
{
    internal class ClipperException : Exception
    {
        public ClipperException(string description)
            : base(description)
        {
        }
    }
}
