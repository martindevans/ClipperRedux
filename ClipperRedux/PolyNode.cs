using System.Collections.Generic;
using System.Diagnostics.Contracts;


namespace ClipperRedux
{
    public class PolyNode
    {
        internal readonly List<IntPoint> Polygon = new List<IntPoint>();
        private int _index;

        private readonly List<PolyNode> _children = new List<PolyNode>();
        public List<PolyNode> Children
        {
            get
            {
                Contract.Ensures(Contract.Result<List<PolyNode>>() != null);
                return _children;
            }
        }

        [ContractInvariantMethod]
        private void ContractInvariants()
        {
            Contract.Invariant(_children != null);
            Contract.Invariant(Polygon != null);
        }

        private bool IsHoleNode()
        {
            var result = true;
            var node = Parent;
            while (node != null)
            {
                result = !result;
                node = node.Parent;
            }
            return result;
        }

        public int ChildCount
        {
            get { return Children.Count; }
        }

        public List<IntPoint> Contour
        {
            get
            {
                Contract.Ensures(Contract.Result<List<IntPoint>>() != null);
                return Polygon;
            }
        }

        internal void AddChild(PolyNode child)
        {
            Contract.Requires(child != null);

            var cnt = Children.Count;
            Children.Add(child);
            child.Parent = this;
            child._index = cnt;
        }

        public PolyNode GetNext()
        {
            if (Children.Count > 0)
                return Children[0];
            else
                return GetNextSiblingUp();
        }

        private PolyNode GetNextSiblingUp()
        {
            if (Parent == null)
                return null;
            else if (_index == Parent.Children.Count - 1)
                return Parent.GetNextSiblingUp();
            else
                return Parent.Children[_index + 1];
        }

        public IEnumerable<PolyNode> Childs
        {
            get
            {
                Contract.Ensures(Contract.Result<IEnumerable<PolyNode>>() != null);
                return Children;
            }
        }

        private PolyNode Parent { get; set; }

        public bool IsHole
        {
            get
            {
                return IsHoleNode();
            }
        }
    }
}
