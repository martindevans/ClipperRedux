using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Linq;

namespace ClipperRedux
{
    public class PolyTree : PolyNode
    {
        private readonly List<PolyNode> _allPolys = new List<PolyNode>();
        public List<PolyNode> AllPolys
        {
            get
            {
                Contract.Ensures(Contract.Result<List<PolyNode>>() != null);
                return _allPolys;
            }
        }

        public int Total
        {
            get { return AllPolys.Count; }
        }

        [ContractInvariantMethod]
        private void ContractInvariant()
        {
            Contract.Invariant(_allPolys != null);
        }

        public void Clear()
        {
            for (var i = 0; i < AllPolys.Count; i++)
                AllPolys[i] = null;
            AllPolys.Clear();
            Children.Clear();
        }

        public PolyNode GetFirst()
        {
            return Children.FirstOrDefault();
        }
    }
}
