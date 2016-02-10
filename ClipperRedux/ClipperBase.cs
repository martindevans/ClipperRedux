using System.Collections.Generic;
using System.Diagnostics.Contracts;


namespace ClipperRedux
{
    public class ClipperBase
    {
        private const long HI_RANGE = InternalHelpers.HI_RANGE;
        internal const long LO_RANGE = InternalHelpers.LO_RANGE;
        internal const double HORIZONTAL = InternalHelpers.HORIZONTAL;

        internal LocalMinima MinimaList;
        internal LocalMinima CurrentLM;
        internal readonly List<List<TEdge>> Edges = new List<List<TEdge>>();

        public void AddPolygons(IEnumerable<List<IntPoint>> ppg, PolyType polyType)
        {
            Contract.Requires(ppg != null);
            Contract.Requires(Contract.ForAll(ppg, a => a != null));

            foreach (var poly in ppg)
                AddPolygon(poly, polyType);
        }

        public void AddPolygon(IReadOnlyList<IntPoint> pg, PolyType polyType)
        {
            Contract.Requires(pg != null);

            var len = pg.Count;
            if (len < 3)
                return;

            var maxVal = HI_RANGE;
            InternalHelpers.RangeTest(pg[0], ref maxVal);

            var p = new List<IntPoint>(len) { new IntPoint(pg[0].X, pg[0].Y) };
            var j = 0;
            for (var i = 1; i < len; ++i)
            {
                InternalHelpers.RangeTest(pg[i], ref maxVal);
                if (p[j].Equals(pg[i]))
                    continue;
                else if (j > 0 && InternalHelpers.SlopesEqual(p[j - 1], p[j], pg[i]))
                {
                    if (p[j - 1].Equals(pg[i]))
                        j--;
                }
                else
                    j++;
                if (j < p.Count)
                    p[j] = pg[i];
                else
                    p.Add(new IntPoint(pg[i].X, pg[i].Y));
            }
            if (j < 2)
                return;

            len = j + 1;
            while (len > 2)
            {
                //nb: test for point equality before testing slopes ...
                if (p[j].Equals(p[0]))
                    j--;
                else if (p[0].Equals(p[1]) || InternalHelpers.SlopesEqual(p[j], p[0], p[1]))
                    p[0] = p[j--];
                else if (InternalHelpers.SlopesEqual(p[j - 1], p[j], p[0]))
                    j--;
                else if (InternalHelpers.SlopesEqual(p[0], p[1], p[2]))
                {
                    for (var i = 2; i <= j; ++i)
                        p[i - 1] = p[i];
                    j--;
                }
                else
                    break;
                len--;
            }
            if (len < 3)
                return;

            //create a new edge array ...
            var edges = new List<TEdge>(len);
            for (var i = 0; i < len; i++)
                edges.Add(new TEdge());
            Edges.Add(edges);

            //convert vertices to a double-linked-list of edges and initialize ...
            edges[0].XCurr = p[0].X;
            edges[0].YCurr = p[0].Y;
            InitEdge(edges[len - 1], edges[0], edges[len - 2], p[len - 1], polyType);
            for (var i = len - 2; i > 0; --i)
                InitEdge(edges[i], edges[i + 1], edges[i - 1], p[i], polyType);
            InitEdge(edges[0], edges[1], edges[len - 1], p[0], polyType);

            //reset xcurr & ycurr and find 'eHighest' (given the Y axis coordinates
            //increase downward so the 'highest' edge will have the smallest ytop) ...
            var e = edges[0];
            var eHighest = e;
            do
            {
                e.XCurr = e.XBot;
                e.YCurr = e.YBot;
                if (e.YTop < eHighest.YTop)
                    eHighest = e;
                e = e.Next;
            } while (e != edges[0]);

            //make sure eHighest is positioned so the following loop works safely ...
            if (eHighest.WindDelta > 0)
                eHighest = eHighest.Next;
            if (eHighest.Dx == HORIZONTAL)
                eHighest = eHighest.Next;

            //finally insert each local minima ...
            e = eHighest;
            do
            {
                e = AddBoundsToLML(e);
            } while (e != eHighest);
        }

        private static void InitEdge(TEdge e, TEdge eNext, TEdge ePrev, IntPoint pt, PolyType polyType)
        {
            Contract.Requires(e != null);
            Contract.Requires(eNext != null);

            e.Next = eNext;
            e.Prev = ePrev;
            e.XCurr = pt.X;
            e.YCurr = pt.Y;
            if (e.YCurr >= e.Next.YCurr)
            {
                e.XBot = e.XCurr;
                e.YBot = e.YCurr;
                e.XTop = e.Next.XCurr;
                e.YTop = e.Next.YCurr;
                e.WindDelta = 1;
            }
            else
            {
                e.XTop = e.XCurr;
                e.YTop = e.YCurr;
                e.XBot = e.Next.XCurr;
                e.YBot = e.Next.YCurr;
                e.WindDelta = -1;
            }
            SetDx(e);
            e.PolyType = polyType;
            e.OutIdx = -1;
        }

        // ReSharper disable once UnusedMemberHiearchy.Global
        // ReSharper disable once MemberCanBeProtected.Global
        public virtual void Clear()
        {
            MinimaList = null;
            MinimaList = null;
            Edges.Clear();
        }

        private static void SetDx(TEdge e)
        {
            Contract.Requires(e != null);

            e.DeltaX = e.XTop - e.XBot;
            e.DeltaY = e.YTop - e.YBot;
            if (e.DeltaY == 0)
                e.Dx = HORIZONTAL;
            else
                e.Dx = (double)e.DeltaX / e.DeltaY;
        }

        private TEdge AddBoundsToLML(TEdge e)
        {
            Contract.Requires(e != null);
            Contract.Requires(e.Next != null);

            //Starting at the top of one bound we progress to the bottom where there's
            //a local minima. We then go to the top of the next bound. These two bounds
            //form the left and right (or right and left) bounds of the local minima.
            e.NextInLML = null;
            e = e.Next;
            for (; ; )
            {
                if (e.Dx == HORIZONTAL)
                {
                    //nb: proceed through horizontals when approaching from their right,
                    //    but break on horizontal minima if approaching from their left.
                    //    This ensures 'local minima' are always on the left of horizontals.
                    if (e.Next.YTop < e.YTop && e.Next.XBot > e.Prev.XBot)
                        break;
                    if (e.XTop != e.Prev.XBot)
                        SwapX(e);
                    e.NextInLML = e.Prev;
                }
                else if (e.YCurr == e.Prev.YCurr)
                    break;
                else
                    e.NextInLML = e.Prev;
                e = e.Next;
            }

            //e and e.prev are now at a local minima ...
            var newLm = new LocalMinima { Next = null, Y = e.Prev.YBot };

            if (e.Dx == HORIZONTAL) //horizontal edges never start a left bound
            {
                if (e.XBot != e.Prev.XBot)
                    SwapX(e);
                newLm.LeftBound = e.Prev;
                newLm.RightBound = e;
            }
            else if (e.Dx < e.Prev.Dx)
            {
                newLm.LeftBound = e.Prev;
                newLm.RightBound = e;
            }
            else
            {
                newLm.LeftBound = e;
                newLm.RightBound = e.Prev;
            }
            newLm.LeftBound.Side = EdgeSide.Left;
            newLm.RightBound.Side = EdgeSide.Right;
            InsertLocalMinima(newLm);

            for (; ; )
            {
                if (e.Next.YTop == e.YTop && e.Next.Dx != HORIZONTAL)
                    break;
                e.NextInLML = e.Next;
                e = e.Next;
                if (e.Dx == HORIZONTAL && e.XBot != e.Prev.XTop)
                    SwapX(e);
            }
            return e.Next;
        }

        private void InsertLocalMinima(LocalMinima newLm)
        {
            Contract.Requires(newLm != null);

            if (MinimaList == null)
            {
                MinimaList = newLm;
            }
            else if (newLm.Y >= MinimaList.Y)
            {
                newLm.Next = MinimaList;
                MinimaList = newLm;
            }
            else
            {
                var tmpLm = MinimaList;
                while (tmpLm.Next != null && (newLm.Y < tmpLm.Next.Y))
                    tmpLm = tmpLm.Next;
                newLm.Next = tmpLm.Next;
                tmpLm.Next = newLm;
            }
        }

        protected void PopLocalMinima()
        {
            if (CurrentLM == null)
                return;
            CurrentLM = CurrentLM.Next;
        }

        private static void SwapX(TEdge e)
        {
            Contract.Requires(e != null);

            //swap horizontal edges' top and bottom x's so they follow the natural
            //progression of the bounds - ie so their xbots will align with the
            //adjoining lower edge. [Helpful in the ProcessHorizontal() method.]
            e.XCurr = e.XTop;
            e.XTop = e.XBot;
            e.XBot = e.XCurr;
        }

        protected virtual void Reset()
        {
            CurrentLM = MinimaList;

            //reset all edges ...
            var lm = MinimaList;
            while (lm != null)
            {
                var e = lm.LeftBound;
                while (e != null)
                {
                    e.XCurr = e.XBot;
                    e.YCurr = e.YBot;
                    e.Side = EdgeSide.Left;
                    e.OutIdx = -1;
                    e = e.NextInLML;
                }
                e = lm.RightBound;
                while (e != null)
                {
                    e.XCurr = e.XBot;
                    e.YCurr = e.YBot;
                    e.Side = EdgeSide.Right;
                    e.OutIdx = -1;
                    e = e.NextInLML;
                }
                lm = lm.Next;
            }
        }

        public IntRect GetBounds()
        {
            if (MinimaList == null)
                return default(IntRect);

            var lm = MinimaList;
            var result = new IntRect(
                lm.LeftBound.XBot,
                lm.LeftBound.YBot,
                lm.LeftBound.XBot,
                lm.LeftBound.YBot
            );
            while (lm != null)
            {
                if (lm.LeftBound.YBot > result.Bottom)
                    result.Bottom = lm.LeftBound.YBot;
                var e = lm.LeftBound;
                for (; ; )
                {
                    var bottomE = e;
                    while (e.NextInLML != null)
                    {
                        if (e.XBot < result.Left)
                            result.Left = e.XBot;
                        if (e.XBot > result.Right)
                            result.Right = e.XBot;
                        e = e.NextInLML;
                    }
                    if (e.XBot < result.Left)
                        result.Left = e.XBot;
                    if (e.XBot > result.Right)
                        result.Right = e.XBot;
                    if (e.XTop < result.Left)
                        result.Left = e.XTop;
                    if (e.XTop > result.Right)
                        result.Right = e.XTop;
                    if (e.YTop < result.Top)
                        result.Top = e.YTop;

                    if (bottomE == lm.LeftBound)
                        e = lm.RightBound;
                    else
                        break;
                }
                lm = lm.Next;
            }
            return result;
        }
    }
}
