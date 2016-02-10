#region copyright
/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson                                                   *
* Version   :  5.1.6                                                           *
* Date      :  23 May 2013                                                     *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2013                                         *
*                                                                              *
* License:                                                                     *
* Use, modification & distribution is subject to Boost Software License Ver 1. *
* http://www.boost.org/LICENSE_1_0.txt                                         *
*                                                                              *
* Attributions:                                                                *
* The code in this library is an extension of Bala Vatti's clipping algorithm: *
* "A generic solution to polygon clipping"                                     *
* Communications of the ACM, Vol 35, Issue 7 (July 1992) pp 56-63.             *
* http://portal.acm.org/citation.cfm?id=129906                                 *
*                                                                              *
* Computer graphics and geometric modeling: implementation and algorithms      *
* By Max K. Agoston                                                            *
* Springer; 1 edition (January 4, 2005)                                        *
* http://books.google.com/books?q=vatti+clipping+agoston                       *
*                                                                              *
* See also:                                                                    *
* "Polygon Offsetting by Computing Winding Numbers"                            *
* Paper no. DETC2005-85513 pp. 565-575                                         *
* ASME 2005 International Design Engineering Technical Conferences             *
* and Computers and Information in Engineering Conference (IDETC/CIE2005)      *
* September 24-28, 2005 , Long Beach, California, USA                          *
* http://www.me.berkeley.edu/~mcmains/pubs/DAC05OffsetPolygon.pdf              *
*                                                                              *
*******************************************************************************/
#endregion

using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Linq;



namespace ClipperRedux
{
    public sealed class Clipper : ClipperBase
    {
        private readonly List<OutRec> _polyOuts = new List<OutRec>();
        private ClipType _clipType;
        private Scanbeam _scanbeam;
        private TEdge _activeEdges;
        private TEdge _sortedEdges;
        private IntersectNode _intersectNodes;
        private bool _executeLocked;
        private PolyFillType _clipFillType;
        private PolyFillType _subjFillType;
        private readonly List<JoinRec> _joins = new List<JoinRec>();
        private readonly List<HorzJoinRec> _horizJoins = new List<HorzJoinRec>();
        private bool _usingPolyTree;

        public bool ReverseSolution { get; set; }
        public bool ForceSimple { get; set; }

        [ContractInvariantMethod]
        private void ContractInvariants()
        {
            Contract.Invariant(_polyOuts != null);
            Contract.Invariant(Contract.ForAll(_polyOuts, a => a != null));

            Contract.Invariant(_joins != null);
            Contract.Invariant(Contract.ForAll(_joins, a => a != null));

            Contract.Invariant(_horizJoins != null);
            Contract.Invariant(Contract.ForAll(_horizJoins, a => a != null));
        }

        public override void Clear()
        {
            if (Edges.Count == 0)
                return; //avoids problems with ClipperBase destructor
            DisposeAllPolyPts();

            base.Clear();
        }

        protected override void Reset()
        {
            base.Reset();

            _scanbeam = null;
            _activeEdges = null;
            _sortedEdges = null;

            DisposeAllPolyPts();
            var lm = MinimaList;
            while (lm != null)
            {
                InsertScanbeam(lm.Y);
                lm = lm.Next;
            }
        }

        private void InsertScanbeam(long y)
        {
            if (_scanbeam == null)
            {
                _scanbeam = new Scanbeam(y, null);
            }
            else if (y > _scanbeam.Y)
            {
                var newSb = new Scanbeam(y, _scanbeam);
                _scanbeam = newSb;
            }
            else
            {
                var sb2 = _scanbeam;
                while (sb2.Next != null && (y <= sb2.Next.Y))
                    sb2 = sb2.Next;
                if (y == sb2.Y)
                    return; //ie ignores duplicates
                var newSb = new Scanbeam(y, sb2.Next);
                sb2.Next = newSb;
            }
        }


        #region execute
        public bool Execute(ClipType clipType, List<List<IntPoint>> solution, PolyFillType subjFillType = PolyFillType.EvenOdd, PolyFillType clipFillType = PolyFillType.EvenOdd)
        {
            Contract.Requires(solution != null);

            if (_executeLocked)
                return false;
            _executeLocked = true;

            solution.Clear();
            _subjFillType = subjFillType;
            _clipFillType = clipFillType;
            _clipType = clipType;
            _usingPolyTree = false;
            var succeeded = ExecuteInternal();

            //build the return polygons ...
            if (succeeded)
                BuildResultPointList(solution);

            _executeLocked = false;
            return succeeded;
        }

        public bool Execute(ClipType clipType, PolyTree polytree, PolyFillType subjFillType = PolyFillType.EvenOdd, PolyFillType clipFillType = PolyFillType.EvenOdd)
        {
            if (_executeLocked)
                return false;
            _executeLocked = true;

            _subjFillType = subjFillType;
            _clipFillType = clipFillType;
            _clipType = clipType;
            _usingPolyTree = true;
            var succeeded = ExecuteInternal();

            //build the return polygons ...
            if (succeeded)
                BuildResultPolytree(polytree);

            _executeLocked = false;
            return succeeded;
        }
        #endregion

        private static void FixHoleLinkage(OutRec outRec)
        {
            Contract.Requires(outRec != null);

            //skip if an outermost polygon or
            //already already points to the correct FirstLeft ...
            if (outRec.FirstLeft == null ||
                (outRec.IsHole != outRec.FirstLeft.IsHole &&
                 outRec.FirstLeft.Pts != null))
                return;

            var orfl = outRec.FirstLeft;
            while (orfl != null && ((orfl.IsHole == outRec.IsHole) || orfl.Pts == null))
                orfl = orfl.FirstLeft;
            outRec.FirstLeft = orfl;
        }



        private bool ExecuteInternal()
        {
            bool succeeded;
            try
            {
                Reset();
                if (CurrentLM == null)
                    return true;
                var botY = PopScanbeam();
                do
                {
                    InsertLocalMinimaIntoAEL(botY);
                    _horizJoins.Clear();
                    ProcessHorizontals();
                    var topY = PopScanbeam();
                    succeeded = ProcessIntersections(botY, topY);
                    if (!succeeded)
                        break;
                    ProcessEdgesAtTopOfScanbeam(topY);
                    botY = topY;
                } while (_scanbeam != null || CurrentLM != null);
            }
            catch
            {
                succeeded = false;
            }

            if (succeeded)
            {
                //tidy up output polygons and fix orientations where necessary ...
                foreach (var outRec in _polyOuts.Where(outRec => outRec.Pts != null))
                {
                    FixupOutPolygon(outRec);
                    if (outRec.Pts == null)
                        continue;
                    if ((outRec.IsHole ^ ReverseSolution) == outRec.Area() > 0)
                        ReversePolyPtLinks(outRec.Pts);
                }
                JoinCommonEdges();
                if (ForceSimple)
                    DoSimplePolygons();
            }
            _joins.Clear();
            _horizJoins.Clear();
            return succeeded;
        }



        private long PopScanbeam()
        {
            Contract.Requires(_scanbeam != null);

            var y = _scanbeam.Y;
            _scanbeam = _scanbeam.Next;
            return y;
        }

        private void DisposeAllPolyPts()
        {
            _polyOuts.Clear();
        }

        private static void DisposeOutPts(OutPt pp)
        {
            Contract.Requires(pp == null || pp.Prev != null);

            if (pp == null)
                return;
            pp.Prev.Next = null;
            while (pp != null)
            {
                pp = pp.Next;
            }
        }

        private void AddJoin(TEdge e1, TEdge e2, int e1OutIdx, int e2OutIdx)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e2 != null);

            var jr = new JoinRec {
                Poly1Idx = e1OutIdx >= 0 ? e1OutIdx : e1.OutIdx,
                Pt1A = new IntPoint(e1.XCurr, e1.YCurr),
                Pt1B = new IntPoint(e1.XTop, e1.YTop),
                Poly2Idx = e2OutIdx >= 0 ? e2OutIdx : e2.OutIdx,
                Pt2A = new IntPoint(e2.XCurr, e2.YCurr),
                Pt2B = new IntPoint(e2.XTop, e2.YTop)
            };
            _joins.Add(jr);
        }

        private void AddHorzJoin(TEdge e, int idx)
        {
            _horizJoins.Add(new HorzJoinRec {Edge = e, SavedIdx = idx});
        }

        private void InsertLocalMinimaIntoAEL(long botY)
        {
            while (CurrentLM != null && (CurrentLM.Y == botY))
            {
                var lb = CurrentLM.LeftBound;
                var rb = CurrentLM.RightBound;

                InsertEdgeIntoAEL(lb);
                InsertScanbeam(lb.YTop);
                InsertEdgeIntoAEL(rb);

                if (IsEvenOddFillType(lb))
                {
                    lb.WindDelta = 1;
                    rb.WindDelta = 1;
                }
                else
                {
                    rb.WindDelta = -lb.WindDelta;
                }
                SetWindingCount(lb);
                rb.WindCnt = lb.WindCnt;
                rb.WindCnt2 = lb.WindCnt2;

                if (rb.Dx == HORIZONTAL)
                {
                    //nb: only rightbounds can have a horizontal bottom edge
                    AddEdgeToSEL(rb);
                    InsertScanbeam(rb.NextInLML.YTop);
                }
                else
                    InsertScanbeam(rb.YTop);

                if (IsContributing(lb))
                    AddLocalMinPoly(lb, rb, new IntPoint(lb.XCurr, CurrentLM.Y));

                //if any output polygons share an edge, they'll need joining later ...
                if (rb.OutIdx >= 0 && rb.Dx == HORIZONTAL)
                {
                    foreach (var joinRec in _horizJoins)
                    {
                        IntPoint pt, pt2;
                        var hj = joinRec;
                        //if horizontals rb and hj.edge overlap, flag for joining later ...
                        if (GetOverlapSegment(new IntPoint(hj.Edge.XBot, hj.Edge.YBot),
                            new IntPoint(hj.Edge.XTop, hj.Edge.YTop),
                            new IntPoint(rb.XBot, rb.YBot),
                            new IntPoint(rb.XTop, rb.YTop),
                            out pt, out pt2))
                            AddJoin(hj.Edge, rb, hj.SavedIdx, -1);
                    }
                }


                if (lb.NextInAEL != rb)
                {
                    if (rb.OutIdx >= 0 && rb.PrevInAEL.OutIdx >= 0 &&
                        InternalHelpers.SlopesEqual(rb.PrevInAEL, rb))
                        AddJoin(rb, rb.PrevInAEL, -1, -1);

                    var e = lb.NextInAEL;
                    var pt = new IntPoint(lb.XCurr, lb.YCurr);
                    while (e != rb)
                    {
                        if (e == null)
                            throw new ClipperException("InsertLocalMinimaIntoAEL: missing rightbound!");
                        //nb: For calculating winding counts etc, IntersectEdges() assumes
                        //that param1 will be to the right of param2 ABOVE the intersection ...
                        IntersectEdges(rb, e, pt, Protects.None); //order important here
                        e = e.NextInAEL;
                    }
                }
                PopLocalMinima();
            }
        }

        private void InsertEdgeIntoAEL(TEdge edge)
        {
            Contract.Requires(edge != null);

            edge.PrevInAEL = null;
            edge.NextInAEL = null;
            if (_activeEdges == null)
            {
                _activeEdges = edge;
            }
            else if (E2InsertsBeforeE1(_activeEdges, edge))
            {
                edge.NextInAEL = _activeEdges;
                _activeEdges.PrevInAEL = edge;
                _activeEdges = edge;
            }
            else
            {
                var e = _activeEdges;
                while (e.NextInAEL != null && !E2InsertsBeforeE1(e.NextInAEL, edge))
                    e = e.NextInAEL;
                edge.NextInAEL = e.NextInAEL;
                if (e.NextInAEL != null)
                    e.NextInAEL.PrevInAEL = edge;
                edge.PrevInAEL = e;
                e.NextInAEL = edge;
            }
        }

        private static bool E2InsertsBeforeE1(TEdge e1, TEdge e2)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e2 != null);

            if (e2.XCurr == e1.XCurr)
            {
                if (e2.YTop > e1.YTop)
                    return e2.XTop < TopX(e1, e2.YTop);
                else
                    return e1.XTop > TopX(e2, e1.YTop);
            }
            else
                return e2.XCurr < e1.XCurr;
        }

        private bool IsEvenOddFillType(TEdge edge)
        {
            Contract.Requires(edge != null);

            if (edge.PolyType == PolyType.Subject)
                return _subjFillType == PolyFillType.EvenOdd;
            else
                return _clipFillType == PolyFillType.EvenOdd;
        }

        private bool IsEvenOddAltFillType(TEdge edge)
        {
            Contract.Requires(edge != null);

            if (edge.PolyType == PolyType.Subject)
                return _clipFillType == PolyFillType.EvenOdd;
            else
                return _subjFillType == PolyFillType.EvenOdd;
        }

        private bool IsContributing(TEdge edge)
        {
            Contract.Requires(edge != null);

            PolyFillType pft, pft2;
            if (edge.PolyType == PolyType.Subject)
            {
                pft = _subjFillType;
                pft2 = _clipFillType;
            }
            else
            {
                pft = _clipFillType;
                pft2 = _subjFillType;
            }

            switch (pft)
            {
                case PolyFillType.EvenOdd:
                case PolyFillType.NonZero:
                    if (Math.Abs(edge.WindCnt) != 1)
                        return false;
                    break;
                case PolyFillType.Positive:
                    if (edge.WindCnt != 1)
                        return false;
                    break;
                case PolyFillType.Negative:
                    if (edge.WindCnt != -1)
                        return false;
                    break;
                default:
                    throw new InvalidOperationException(string.Format("Unknown polyFillType '{0}'", pft));
            }

            switch (_clipType)
            {
                case ClipType.Intersection:
                    switch (pft2)
                    {
                        case PolyFillType.EvenOdd:
                        case PolyFillType.NonZero:
                            return edge.WindCnt2 != 0;
                        case PolyFillType.Positive:
                            return edge.WindCnt2 > 0;
                        default:
                            return edge.WindCnt2 < 0;
                    }
                case ClipType.Union:
                    switch (pft2)
                    {
                        case PolyFillType.EvenOdd:
                        case PolyFillType.NonZero:
                            return edge.WindCnt2 == 0;
                        case PolyFillType.Positive:
                            return edge.WindCnt2 <= 0;
                        default:
                            return edge.WindCnt2 >= 0;
                    }
                case ClipType.Difference:
                    if (edge.PolyType == PolyType.Subject)
                        switch (pft2)
                        {
                            case PolyFillType.EvenOdd:
                            case PolyFillType.NonZero:
                                return edge.WindCnt2 == 0;
                            case PolyFillType.Positive:
                                return edge.WindCnt2 <= 0;
                            default:
                                return edge.WindCnt2 >= 0;
                        }
                    else
                        switch (pft2)
                        {
                            case PolyFillType.EvenOdd:
                            case PolyFillType.NonZero:
                                return edge.WindCnt2 != 0;
                            case PolyFillType.Positive:
                                return edge.WindCnt2 > 0;
                            default:
                                return edge.WindCnt2 < 0;
                        }
            }
            return true;
        }

        private void SetWindingCount(TEdge edge)
        {
            Contract.Requires(edge != null);

            var e = edge.PrevInAEL;
            //find the edge of the same polytype that immediately preceeds 'edge' in AEL
            while (e != null && e.PolyType != edge.PolyType)
                e = e.PrevInAEL;
            if (e == null)
            {
                edge.WindCnt = edge.WindDelta;
                edge.WindCnt2 = 0;
                e = _activeEdges; //ie get ready to calc windCnt2
            }
            else if (IsEvenOddFillType(edge))
            {
                //even-odd filling ...
                edge.WindCnt = 1;
                edge.WindCnt2 = e.WindCnt2;
                e = e.NextInAEL; //ie get ready to calc windCnt2
            }
            else
            {
                //nonZero filling ...
                if (e.WindCnt * e.WindDelta < 0)
                {
                    if (Math.Abs(e.WindCnt) > 1)
                    {
                        if (e.WindDelta * edge.WindDelta < 0)
                            edge.WindCnt = e.WindCnt;
                        else
                            edge.WindCnt = e.WindCnt + edge.WindDelta;
                    }
                    else
                        edge.WindCnt = e.WindCnt + e.WindDelta + edge.WindDelta;
                }
                else
                {
                    if (Math.Abs(e.WindCnt) > 1 && e.WindDelta * edge.WindDelta < 0)
                        edge.WindCnt = e.WindCnt;
                    else if (e.WindCnt + edge.WindDelta == 0)
                        edge.WindCnt = e.WindCnt;
                    else
                        edge.WindCnt = e.WindCnt + edge.WindDelta;
                }
                edge.WindCnt2 = e.WindCnt2;
                e = e.NextInAEL; //ie get ready to calc windCnt2
            }

            //update windCnt2 ...
            if (IsEvenOddAltFillType(edge))
            {
                //even-odd filling ...
                while (e != edge)
                {
                    edge.WindCnt2 = edge.WindCnt2 == 0 ? 1 : 0;
                    e = e.NextInAEL;
                }
            }
            else
            {
                //nonZero filling ...
                while (e != edge)
                {
                    edge.WindCnt2 += e.WindDelta;
                    e = e.NextInAEL;
                }
            }
        }



        private void AddEdgeToSEL(TEdge edge)
        {
            Contract.Requires(edge != null);

            //SEL pointers in PEdge are reused to build a list of horizontal edges.
            //However, we don't need to worry about order with horizontal edge processing.
            if (_sortedEdges == null)
            {
                _sortedEdges = edge;
                edge.PrevInSEL = null;
                edge.NextInSEL = null;
            }
            else
            {
                edge.NextInSEL = _sortedEdges;
                edge.PrevInSEL = null;
                _sortedEdges.PrevInSEL = edge;
                _sortedEdges = edge;
            }
        }



        private void CopyAELToSEL()
        {
            var e = _activeEdges;
            _sortedEdges = e;
            while (e != null)
            {
                e.PrevInSEL = e.PrevInAEL;
                e.NextInSEL = e.NextInAEL;
                e = e.NextInAEL;
            }
        }



        private void SwapPositionsInAEL(TEdge edge1, TEdge edge2)
        {
            Contract.Requires(edge1 != null);
            Contract.Requires(edge2 != null);

            if (edge1.NextInAEL == edge2)
            {
                var next = edge2.NextInAEL;
                if (next != null)
                    next.PrevInAEL = edge1;
                var prev = edge1.PrevInAEL;
                if (prev != null)
                    prev.NextInAEL = edge2;
                edge2.PrevInAEL = prev;
                edge2.NextInAEL = edge1;
                edge1.PrevInAEL = edge2;
                edge1.NextInAEL = next;
            }
            else if (edge2.NextInAEL == edge1)
            {
                var next = edge1.NextInAEL;
                if (next != null)
                    next.PrevInAEL = edge2;
                var prev = edge2.PrevInAEL;
                if (prev != null)
                    prev.NextInAEL = edge1;
                edge1.PrevInAEL = prev;
                edge1.NextInAEL = edge2;
                edge2.PrevInAEL = edge1;
                edge2.NextInAEL = next;
            }
            else
            {
                var next = edge1.NextInAEL;
                var prev = edge1.PrevInAEL;
                edge1.NextInAEL = edge2.NextInAEL;
                if (edge1.NextInAEL != null)
                    edge1.NextInAEL.PrevInAEL = edge1;
                edge1.PrevInAEL = edge2.PrevInAEL;
                if (edge1.PrevInAEL != null)
                    edge1.PrevInAEL.NextInAEL = edge1;
                edge2.NextInAEL = next;
                if (edge2.NextInAEL != null)
                    edge2.NextInAEL.PrevInAEL = edge2;
                edge2.PrevInAEL = prev;
                if (edge2.PrevInAEL != null)
                    edge2.PrevInAEL.NextInAEL = edge2;
            }

            if (edge1.PrevInAEL == null)
                _activeEdges = edge1;
            else if (edge2.PrevInAEL == null)
                _activeEdges = edge2;
        }



        private void SwapPositionsInSEL(TEdge edge1, TEdge edge2)
        {
            Contract.Requires(edge1 != null);
            Contract.Requires(edge2 != null);

            if (edge1.NextInSEL == null && edge1.PrevInSEL == null)
                return;
            if (edge2.NextInSEL == null && edge2.PrevInSEL == null)
                return;

            if (edge1.NextInSEL == edge2)
            {
                var next = edge2.NextInSEL;
                if (next != null)
                    next.PrevInSEL = edge1;
                var prev = edge1.PrevInSEL;
                if (prev != null)
                    prev.NextInSEL = edge2;
                edge2.PrevInSEL = prev;
                edge2.NextInSEL = edge1;
                edge1.PrevInSEL = edge2;
                edge1.NextInSEL = next;
            }
            else if (edge2.NextInSEL == edge1)
            {
                var next = edge1.NextInSEL;
                if (next != null)
                    next.PrevInSEL = edge2;
                var prev = edge2.PrevInSEL;
                if (prev != null)
                    prev.NextInSEL = edge1;
                edge1.PrevInSEL = prev;
                edge1.NextInSEL = edge2;
                edge2.PrevInSEL = edge1;
                edge2.NextInSEL = next;
            }
            else
            {
                var next = edge1.NextInSEL;
                var prev = edge1.PrevInSEL;
                edge1.NextInSEL = edge2.NextInSEL;
                if (edge1.NextInSEL != null)
                    edge1.NextInSEL.PrevInSEL = edge1;
                edge1.PrevInSEL = edge2.PrevInSEL;
                if (edge1.PrevInSEL != null)
                    edge1.PrevInSEL.NextInSEL = edge1;
                edge2.NextInSEL = next;
                if (edge2.NextInSEL != null)
                    edge2.NextInSEL.PrevInSEL = edge2;
                edge2.PrevInSEL = prev;
                if (edge2.PrevInSEL != null)
                    edge2.PrevInSEL.NextInSEL = edge2;
            }

            if (edge1.PrevInSEL == null)
                _sortedEdges = edge1;
            else if (edge2.PrevInSEL == null)
                _sortedEdges = edge2;
        }




        private void AddLocalMaxPoly(TEdge e1, TEdge e2, IntPoint pt)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e1.OutIdx >= 0 && e1.OutIdx < _polyOuts.Count);
            Contract.Requires(e2 != null);

            AddOutPt(e1, pt);
            if (e1.OutIdx == e2.OutIdx)
            {
                e1.OutIdx = -1;
                e2.OutIdx = -1;
            }
            else if (e1.OutIdx < e2.OutIdx)
                AppendPolygon(e1, e2);
            else
                AppendPolygon(e2, e1);
        }



        private void AddLocalMinPoly(TEdge e1, TEdge e2, IntPoint pt)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e2 != null);

            TEdge e, prevE;
            if (e2.Dx == HORIZONTAL || (e1.Dx > e2.Dx))
            {
                AddOutPt(e1, pt);
                e2.OutIdx = e1.OutIdx;
                e1.Side = EdgeSide.Left;
                e2.Side = EdgeSide.Right;
                e = e1;
                prevE = e.PrevInAEL == e2 ? e2.PrevInAEL : e.PrevInAEL;
            }
            else
            {
                AddOutPt(e2, pt);
                e1.OutIdx = e2.OutIdx;
                e1.Side = EdgeSide.Right;
                e2.Side = EdgeSide.Left;
                e = e2;
                prevE = e.PrevInAEL == e1 ? e1.PrevInAEL : e.PrevInAEL;
            }

            if (prevE != null && prevE.OutIdx >= 0 &&
                (TopX(prevE, pt.Y) == TopX(e, pt.Y)) &&
                InternalHelpers.SlopesEqual(e, prevE))
                AddJoin(e, prevE, -1, -1);

        }



        private OutRec CreateOutRec()
        {
            var result = new OutRec {
                Idx = -1,
                IsHole = false,
                FirstLeft = null,
                Pts = null,
                BottomPt = null,
                PolyNode = null
            };
            _polyOuts.Add(result);
            result.Idx = _polyOuts.Count - 1;
            return result;
        }



        private void AddOutPt(TEdge e, IntPoint pt)
        {
            Contract.Requires(e != null);
            Contract.Requires(e.OutIdx < 0 || e.OutIdx < _polyOuts.Count);

            var toFront = e.Side == EdgeSide.Left;
            if (e.OutIdx < 0)
            {
                var outRec = CreateOutRec();
                e.OutIdx = outRec.Idx;
                var op = new OutPt();
                outRec.Pts = op;
                op.Pt = pt;
                op.Next = op;
                op.Prev = op;
                SetHoleState(e, outRec);
            }
            else
            {
                var outRec = _polyOuts[e.OutIdx];
                var op = outRec.Pts;
                if (toFront && pt.Equals(op.Pt) ||
                    (!toFront && pt.Equals(op.Prev.Pt)))
                    return;

                var op2 = new OutPt {
                    Pt = pt,
                    Next = op,
                    Prev = op.Prev
                };
                op2.Prev.Next = op2;
                op.Prev = op2;
                if (toFront)
                    outRec.Pts = op2;
            }
        }

        private static bool GetOverlapSegment(IntPoint pt1A, IntPoint pt1B, IntPoint pt2A, IntPoint pt2B, out IntPoint pt1, out IntPoint pt2)
        {
            //precondition: segments are colinear.
            if (Math.Abs(pt1A.X - pt1B.X) > Math.Abs(pt1A.Y - pt1B.Y))
            {
                if (pt1A.X > pt1B.X)
                    InternalHelpers.Swap(ref pt1A, ref pt1B);
                if (pt2A.X > pt2B.X)
                    InternalHelpers.Swap(ref pt2A, ref pt2B);
                pt1 = pt1A.X > pt2A.X ? pt1A : pt2A;
                pt2 = pt1B.X < pt2B.X ? pt1B : pt2B;
                return pt1.X < pt2.X;
            }
            else
            {
                if (pt1A.Y < pt1B.Y)
                    InternalHelpers.Swap(ref pt1A, ref pt1B);
                if (pt2A.Y < pt2B.Y)
                    InternalHelpers.Swap(ref pt2A, ref pt2B);
                pt1 = pt1A.Y < pt2A.Y ? pt1A : pt2A;
                pt2 = pt1B.Y > pt2B.Y ? pt1B : pt2B;
                return pt1.Y > pt2.Y;
            }
        }

        private static bool FindSegment(ref OutPt pp, ref IntPoint pt1, ref IntPoint pt2)
        {
            if (pp == null)
                return false;
            var pp2 = pp;
            var pt1A = pt1;
            var pt2A = pt2;
            do
            {
                if (InternalHelpers.SlopesEqual(pt1A, pt2A, pp.Pt, pp.Prev.Pt) &&
                    InternalHelpers.SlopesEqual(pt1A, pt2A, pp.Pt) &&
                    GetOverlapSegment(pt1A, pt2A, pp.Pt, pp.Prev.Pt, out pt1, out pt2))
                    return true;
                pp = pp.Next;
            } while (pp != pp2);
            return false;
        }

        private static bool Pt3IsBetweenPt1AndPt2(IntPoint pt1, IntPoint pt2, IntPoint pt3)
        {
            if (pt1.Equals(pt3) || pt2.Equals(pt3))
                return true;
            else if (pt1.X != pt2.X)
                return pt1.X < pt3.X == pt3.X < pt2.X;
            else
                return pt1.Y < pt3.Y == pt3.Y < pt2.Y;
        }



        private static OutPt InsertPolyPtBetween(OutPt p1, OutPt p2, IntPoint pt)
        {
            Contract.Requires(p1 != null);
            Contract.Requires(p2 != null);
            Contract.Ensures(Contract.Result<OutPt>() != null);

            var result = new OutPt {Pt = pt};
            if (p2 == p1.Next)
            {
                p1.Next = result;
                p2.Prev = result;
                result.Next = p2;
                result.Prev = p1;
            }
            else
            {
                p2.Next = result;
                p1.Prev = result;
                result.Next = p1;
                result.Prev = p2;
            }
            return result;
        }



        private void SetHoleState(TEdge e, OutRec outRec)
        {
            Contract.Requires(e != null);

            var isHole = false;
            var e2 = e.PrevInAEL;
            while (e2 != null)
            {
                if (e2.OutIdx >= 0)
                {
                    isHole = !isHole;
                    if (outRec.FirstLeft == null)
                        outRec.FirstLeft = _polyOuts[e2.OutIdx];
                }
                e2 = e2.PrevInAEL;
            }
            if (isHole)
                outRec.IsHole = true;
        }

        private static OutRec GetLowermostRec(OutRec outRec1, OutRec outRec2)
        {
            Contract.Requires(outRec2 != null);
            Contract.Requires(outRec1 != null);

            //work out which polygon fragment has the correct hole state ...
            if (outRec1.BottomPt == null)
                outRec1.BottomPt = outRec1.Pts.GetBottomPt();
            if (outRec2.BottomPt == null)
                outRec2.BottomPt = outRec2.Pts.GetBottomPt();
            var bPt1 = outRec1.BottomPt;
            var bPt2 = outRec2.BottomPt;
            if (bPt1.Pt.Y > bPt2.Pt.Y)
                return outRec1;
            else if (bPt1.Pt.Y < bPt2.Pt.Y)
                return outRec2;
            else if (bPt1.Pt.X < bPt2.Pt.X)
                return outRec1;
            else if (bPt1.Pt.X > bPt2.Pt.X)
                return outRec2;
            else if (bPt1.Next == bPt1)
                return outRec2;
            else if (bPt2.Next == bPt2)
                return outRec1;
            else if (InternalHelpers.FirstIsBottomPt(bPt1, bPt2))
                return outRec1;
            else
                return outRec2;
        }



        private static bool Param1RightOfParam2(OutRec outRec1, OutRec outRec2)
        {
            Contract.Requires(outRec1 != null);

            do
            {
                outRec1 = outRec1.FirstLeft;
                if (outRec1 == outRec2)
                    return true;
            } while (outRec1 != null);
            return false;
        }



        private OutRec GetOutRec(int idx)
        {
            Contract.Requires(idx >= 0 && idx < _polyOuts.Count);

            var outrec = _polyOuts[idx];
            while (outrec != _polyOuts[outrec.Idx])
                outrec = _polyOuts[outrec.Idx];
            return outrec;
        }



        private void AppendPolygon(TEdge e1, TEdge e2)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e1.OutIdx >= 0 && e1.OutIdx < _polyOuts.Count);
            Contract.Requires(e2 != null);
            Contract.Requires(e2.OutIdx >= 0 && e2.OutIdx < _polyOuts.Count);

            //get the start and ends of both output polygons ...
            var outRec1 = _polyOuts[e1.OutIdx];
            var outRec2 = _polyOuts[e2.OutIdx];

            OutRec holeStateRec;
            if (Param1RightOfParam2(outRec1, outRec2))
                holeStateRec = outRec2;
            else if (Param1RightOfParam2(outRec2, outRec1))
                holeStateRec = outRec1;
            else
                holeStateRec = GetLowermostRec(outRec1, outRec2);

            var p1Lft = outRec1.Pts;
            var p1Rt = p1Lft.Prev;
            var p2Lft = outRec2.Pts;
            var p2Rt = p2Lft.Prev;

            EdgeSide side;
            //join e2 poly onto e1 poly and delete pointers to e2 ...
            if (e1.Side == EdgeSide.Left)
            {
                if (e2.Side == EdgeSide.Left)
                {
                    //z y x a b c
                    ReversePolyPtLinks(p2Lft);
                    p2Lft.Next = p1Lft;
                    p1Lft.Prev = p2Lft;
                    p1Rt.Next = p2Rt;
                    p2Rt.Prev = p1Rt;
                    outRec1.Pts = p2Rt;
                }
                else
                {
                    //x y z a b c
                    p2Rt.Next = p1Lft;
                    p1Lft.Prev = p2Rt;
                    p2Lft.Prev = p1Rt;
                    p1Rt.Next = p2Lft;
                    outRec1.Pts = p2Lft;
                }
                side = EdgeSide.Left;
            }
            else
            {
                if (e2.Side == EdgeSide.Right)
                {
                    //a b c z y x
                    ReversePolyPtLinks(p2Lft);
                    p1Rt.Next = p2Rt;
                    p2Rt.Prev = p1Rt;
                    p2Lft.Next = p1Lft;
                    p1Lft.Prev = p2Lft;
                }
                else
                {
                    //a b c x y z
                    p1Rt.Next = p2Lft;
                    p2Lft.Prev = p1Rt;
                    p1Lft.Prev = p2Rt;
                    p2Rt.Next = p1Lft;
                }
                side = EdgeSide.Right;
            }

            outRec1.BottomPt = null;
            if (holeStateRec == outRec2)
            {
                if (outRec2.FirstLeft != outRec1)
                    outRec1.FirstLeft = outRec2.FirstLeft;
                outRec1.IsHole = outRec2.IsHole;
            }
            outRec2.Pts = null;
            outRec2.BottomPt = null;

            outRec2.FirstLeft = outRec1;

            var okIdx = e1.OutIdx;
            var obsoleteIdx = e2.OutIdx;

            e1.OutIdx = -1; //nb: safe because we only get here via AddLocalMaxPoly
            e2.OutIdx = -1;

            var e = _activeEdges;
            while (e != null)
            {
                if (e.OutIdx == obsoleteIdx)
                {
                    e.OutIdx = okIdx;
                    e.Side = side;
                    break;
                }
                e = e.NextInAEL;
            }
            outRec2.Idx = outRec1.Idx;
        }



        private static void ReversePolyPtLinks(OutPt pp)
        {
            if (pp == null)
                return;
            var pp1 = pp;
            do
            {
                var pp2 = pp1.Next;
                pp1.Next = pp1.Prev;
                pp1.Prev = pp2;
                pp1 = pp2;
            } while (pp1 != pp);
        }

        private void IntersectEdges(TEdge e1, TEdge e2, IntPoint pt, Protects protects)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e2 != null);

            //e1 will be to the left of e2 BELOW the intersection. Therefore e1 is before
            //e2 in AEL except when e1 is being inserted at the intersection point ...

            var e1Stops = (Protects.Left & protects) == 0 && e1.NextInLML == null && e1.XTop == pt.X && e1.YTop == pt.Y;
            var e2Stops = (Protects.Right & protects) == 0 && e2.NextInLML == null && e2.XTop == pt.X && e2.YTop == pt.Y;
            var e1Contributing = e1.OutIdx >= 0;
            var e2Contributing = e2.OutIdx >= 0;

            //update winding counts...
            //assumes that e1 will be to the right of e2 ABOVE the intersection
            if (e1.PolyType == e2.PolyType)
            {
                if (IsEvenOddFillType(e1))
                {
                    var oldE1WindCnt = e1.WindCnt;
                    e1.WindCnt = e2.WindCnt;
                    e2.WindCnt = oldE1WindCnt;
                }
                else
                {
                    if (e1.WindCnt + e2.WindDelta == 0)
                        e1.WindCnt = -e1.WindCnt;
                    else
                        e1.WindCnt += e2.WindDelta;
                    if (e2.WindCnt - e1.WindDelta == 0)
                        e2.WindCnt = -e2.WindCnt;
                    else
                        e2.WindCnt -= e1.WindDelta;
                }
            }
            else
            {
                if (!IsEvenOddFillType(e2))
                    e1.WindCnt2 += e2.WindDelta;
                else
                    e1.WindCnt2 = e1.WindCnt2 == 0 ? 1 : 0;
                if (!IsEvenOddFillType(e1))
                    e2.WindCnt2 -= e1.WindDelta;
                else
                    e2.WindCnt2 = e2.WindCnt2 == 0 ? 1 : 0;
            }

            PolyFillType e1FillType, e2FillType, e1FillType2, e2FillType2;
            if (e1.PolyType == PolyType.Subject)
            {
                e1FillType = _subjFillType;
                e1FillType2 = _clipFillType;
            }
            else
            {
                e1FillType = _clipFillType;
                e1FillType2 = _subjFillType;
            }
            if (e2.PolyType == PolyType.Subject)
            {
                e2FillType = _subjFillType;
                e2FillType2 = _clipFillType;
            }
            else
            {
                e2FillType = _clipFillType;
                e2FillType2 = _subjFillType;
            }

            int e1Wc, e2Wc;
            switch (e1FillType)
            {
                case PolyFillType.Positive:
                    e1Wc = e1.WindCnt;
                    break;
                case PolyFillType.Negative:
                    e1Wc = -e1.WindCnt;
                    break;
                default:
                    e1Wc = Math.Abs(e1.WindCnt);
                    break;
            }
            switch (e2FillType)
            {
                case PolyFillType.Positive:
                    e2Wc = e2.WindCnt;
                    break;
                case PolyFillType.Negative:
                    e2Wc = -e2.WindCnt;
                    break;
                default:
                    e2Wc = Math.Abs(e2.WindCnt);
                    break;
            }

            if (e1Contributing && e2Contributing)
            {
                if (e1Stops || e2Stops ||
                    (e1Wc != 0 && e1Wc != 1) || (e2Wc != 0 && e2Wc != 1) ||
                    (e1.PolyType != e2.PolyType && _clipType != ClipType.Xor))
                    AddLocalMaxPoly(e1, e2, pt);
                else
                {
                    AddOutPt(e1, pt);
                    AddOutPt(e2, pt);
                    InternalHelpers.Swap(ref e1.Side, ref e2.Side);
                    InternalHelpers.Swap(ref e1.OutIdx, ref e2.OutIdx);
                }
            }
            else if (e1Contributing)
            {
                if (e2Wc == 0 || e2Wc == 1)
                {
                    AddOutPt(e1, pt);
                    InternalHelpers.Swap(ref e1.Side, ref e2.Side);
                    InternalHelpers.Swap(ref e1.OutIdx, ref e2.OutIdx);
                }

            }
            else if (e2Contributing)
            {
                if (e1Wc == 0 || e1Wc == 1)
                {
                    AddOutPt(e2, pt);
                    InternalHelpers.Swap(ref e1.Side, ref e2.Side);
                    InternalHelpers.Swap(ref e1.OutIdx, ref e2.OutIdx);
                }
            }
            else if ((e1Wc == 0 || e1Wc == 1) &&
                     (e2Wc == 0 || e2Wc == 1) && !e1Stops && !e2Stops)
            {
                //neither edge is currently contributing ...
                long e1Wc2, e2Wc2;
                switch (e1FillType2)
                {
                    case PolyFillType.Positive:
                        e1Wc2 = e1.WindCnt2;
                        break;
                    case PolyFillType.Negative:
                        e1Wc2 = -e1.WindCnt2;
                        break;
                    default:
                        e1Wc2 = Math.Abs(e1.WindCnt2);
                        break;
                }
                switch (e2FillType2)
                {
                    case PolyFillType.Positive:
                        e2Wc2 = e2.WindCnt2;
                        break;
                    case PolyFillType.Negative:
                        e2Wc2 = -e2.WindCnt2;
                        break;
                    default:
                        e2Wc2 = Math.Abs(e2.WindCnt2);
                        break;
                }

                if (e1.PolyType != e2.PolyType)
                    AddLocalMinPoly(e1, e2, pt);
                else if (e1Wc == 1 && e2Wc == 1)
                    switch (_clipType)
                    {
                        case ClipType.Intersection:
                            if (e1Wc2 > 0 && e2Wc2 > 0)
                                AddLocalMinPoly(e1, e2, pt);
                            break;
                        case ClipType.Union:
                            if (e1Wc2 <= 0 && e2Wc2 <= 0)
                                AddLocalMinPoly(e1, e2, pt);
                            break;
                        case ClipType.Difference:
                            if (((e1.PolyType == PolyType.Clip) && (e1Wc2 > 0) && (e2Wc2 > 0)) ||
                                ((e1.PolyType == PolyType.Subject) && (e1Wc2 <= 0) && (e2Wc2 <= 0)))
                                AddLocalMinPoly(e1, e2, pt);
                            break;
                        case ClipType.Xor:
                            AddLocalMinPoly(e1, e2, pt);
                            break;
                    }
                else
                    InternalHelpers.Swap(ref e1.Side, ref e2.Side);
            }

            if ((e1Stops != e2Stops) &&
                ((e1Stops && (e1.OutIdx >= 0)) || (e2Stops && (e2.OutIdx >= 0))))
            {
                InternalHelpers.Swap(ref e1.Side, ref e2.Side);
                InternalHelpers.Swap(ref e1.OutIdx, ref e2.OutIdx);
            }

            //finally, delete any non-contributing maxima edges  ...
            if (e1Stops)
                DeleteFromAEL(e1);
            if (e2Stops)
                DeleteFromAEL(e2);
        }



        private void DeleteFromAEL(TEdge e)
        {
            Contract.Requires(e != null);

            var aelPrev = e.PrevInAEL;
            var aelNext = e.NextInAEL;
            if (aelPrev == null && aelNext == null && (e != _activeEdges))
                return; //already deleted
            if (aelPrev != null)
                aelPrev.NextInAEL = aelNext;
            else
                _activeEdges = aelNext;
            if (aelNext != null)
                aelNext.PrevInAEL = aelPrev;
            e.NextInAEL = null;
            e.PrevInAEL = null;
        }



        private void DeleteFromSEL(TEdge e)
        {
            Contract.Requires(e != null);

            var selPrev = e.PrevInSEL;
            var selNext = e.NextInSEL;
            if (selPrev == null && selNext == null && (e != _sortedEdges))
                return; //already deleted
            if (selPrev != null)
                selPrev.NextInSEL = selNext;
            else
                _sortedEdges = selNext;
            if (selNext != null)
                selNext.PrevInSEL = selPrev;
            e.NextInSEL = null;
            e.PrevInSEL = null;
        }



        private void UpdateEdgeIntoAEL(ref TEdge e)
        {
            Contract.Requires(e != null);

            if (e.NextInLML == null)
                throw new ClipperException("UpdateEdgeIntoAEL: invalid call");
            var aelPrev = e.PrevInAEL;
            var aelNext = e.NextInAEL;
            e.NextInLML.OutIdx = e.OutIdx;
            if (aelPrev != null)
                aelPrev.NextInAEL = e.NextInLML;
            else
                _activeEdges = e.NextInLML;
            if (aelNext != null)
                aelNext.PrevInAEL = e.NextInLML;
            e.NextInLML.Side = e.Side;
            e.NextInLML.WindDelta = e.WindDelta;
            e.NextInLML.WindCnt = e.WindCnt;
            e.NextInLML.WindCnt2 = e.WindCnt2;
            e = e.NextInLML;
            e.PrevInAEL = aelPrev;
            e.NextInAEL = aelNext;
            if (e.Dx != HORIZONTAL)
                InsertScanbeam(e.YTop);
        }



        private void ProcessHorizontals()
        {
            var horzEdge = _sortedEdges;
            while (horzEdge != null)
            {
                DeleteFromSEL(horzEdge);
                ProcessHorizontal(horzEdge);
                horzEdge = _sortedEdges;
            }
        }



        private void ProcessHorizontal(TEdge horzEdge)
        {
            Contract.Requires(horzEdge != null);

            Direction direction;
            long horzLeft, horzRight;

            if (horzEdge.XCurr < horzEdge.XTop)
            {
                horzLeft = horzEdge.XCurr;
                horzRight = horzEdge.XTop;
                direction = Direction.LeftToRight;
            }
            else
            {
                horzLeft = horzEdge.XTop;
                horzRight = horzEdge.XCurr;
                direction = Direction.RightToLeft;
            }

            var eMaxPair = horzEdge.NextInLML != null ? null : InternalHelpers.GetMaximaPair(horzEdge);

            var e = InternalHelpers.GetNextInAEL(horzEdge, direction);
            while (e != null)
            {
                if (e.XCurr == horzEdge.XTop && eMaxPair == null)
                {
                    if (InternalHelpers.SlopesEqual(e, horzEdge.NextInLML))
                    {
                        //if output polygons share an edge, they'll need joining later ...
                        if (horzEdge.OutIdx >= 0 && e.OutIdx >= 0)
                            AddJoin(horzEdge.NextInLML, e, horzEdge.OutIdx, -1);
                        break; //we've reached the end of the horizontal line
                    }
                    else if (e.Dx < horzEdge.NextInLML.Dx)
                        //we really have got to the end of the intermediate horz edge so quit.
                        //nb: More -ve slopes follow more +ve slopes ABOVE the horizontal.
                        break;
                }

                var eNext = InternalHelpers.GetNextInAEL(e, direction);
                if (eMaxPair != null ||
                    ((direction == Direction.LeftToRight) && (e.XCurr < horzRight)) ||
                    ((direction == Direction.RightToLeft) && (e.XCurr > horzLeft)))
                {
                    //so far we're still in range of the horizontal edge

                    if (e == eMaxPair)
                    {
                        //horzEdge is evidently a maxima horizontal and we've arrived at its end.
                        if (direction == Direction.LeftToRight)
                            IntersectEdges(horzEdge, e, new IntPoint(e.XCurr, horzEdge.YCurr), 0);
                        else
                            IntersectEdges(e, horzEdge, new IntPoint(e.XCurr, horzEdge.YCurr), 0);
                        if (eMaxPair.OutIdx >= 0)
                            throw new ClipperException("ProcessHorizontal error");
                        return;
                    }
                    else if (e.Dx == HORIZONTAL && !InternalHelpers.IsMinima(e) && !(e.XCurr > e.XTop))
                    {
                        if (direction == Direction.LeftToRight)
                            IntersectEdges(horzEdge, e, new IntPoint(e.XCurr, horzEdge.YCurr),
                                IsTopHorz(horzEdge, e.XCurr) ? Protects.Left : Protects.Both);
                        else
                            IntersectEdges(e, horzEdge, new IntPoint(e.XCurr, horzEdge.YCurr),
                                IsTopHorz(horzEdge, e.XCurr) ? Protects.Right : Protects.Both);
                    }
                    else if (direction == Direction.LeftToRight)
                    {
                        IntersectEdges(horzEdge, e, new IntPoint(e.XCurr, horzEdge.YCurr),
                            IsTopHorz(horzEdge, e.XCurr) ? Protects.Left : Protects.Both);
                    }
                    else
                    {
                        IntersectEdges(e, horzEdge, new IntPoint(e.XCurr, horzEdge.YCurr),
                            IsTopHorz(horzEdge, e.XCurr) ? Protects.Right : Protects.Both);
                    }
                    SwapPositionsInAEL(horzEdge, e);
                }
                else if ((direction == Direction.LeftToRight && e.XCurr >= horzRight) || (direction == Direction.RightToLeft && e.XCurr <= horzLeft))
                    break;
                e = eNext;
            } //end while ( e )

            if (horzEdge.NextInLML != null)
            {
                if (horzEdge.OutIdx >= 0)
                    AddOutPt(horzEdge, new IntPoint(horzEdge.XTop, horzEdge.YTop));
                UpdateEdgeIntoAEL(ref horzEdge);
            }
            else
            {
                if (horzEdge.OutIdx >= 0)
                    IntersectEdges(horzEdge, eMaxPair, new IntPoint(horzEdge.XTop, horzEdge.YCurr), Protects.Both);
                DeleteFromAEL(eMaxPair);
                DeleteFromAEL(horzEdge);
            }
        }



        private bool IsTopHorz(TEdge horzEdge, double xPos)
        {
            var e = _sortedEdges;
            while (e != null)
            {
                if ((xPos >= Math.Min(e.XCurr, e.XTop)) && (xPos <= Math.Max(e.XCurr, e.XTop)))
                    return false;
                e = e.NextInSEL;
            }
            return true;
        }

        private bool ProcessIntersections(long botY, long topY)
        {
            if (_activeEdges == null)
                return true;
            try
            {
                BuildIntersectList(botY, topY);
                if (_intersectNodes == null)
                    return true;
                if (_intersectNodes.Next == null || FixupIntersectionOrder())
                    ProcessIntersectList();
                else
                    return false;
            }
            catch
            {
                _sortedEdges = null;
                DisposeIntersectNodes();
                throw new ClipperException("ProcessIntersections error");
            }
            _sortedEdges = null;
            return true;
        }



        private void BuildIntersectList(long botY, long topY)
        {
            if (_activeEdges == null)
                return;

            //prepare for sorting ...
            var e = _activeEdges;
            _sortedEdges = e;
            while (e != null)
            {
                e.PrevInSEL = e.PrevInAEL;
                e.NextInSEL = e.NextInAEL;
                e.XCurr = TopX(e, topY);
                e = e.NextInAEL;
            }

            //bubblesort ...
            var isModified = true;
            while (isModified && _sortedEdges != null)
            {
                isModified = false;
                e = _sortedEdges;
                while (e.NextInSEL != null)
                {
                    var eNext = e.NextInSEL;
                    var pt = new IntPoint();
                    if (e.XCurr > eNext.XCurr)
                    {
                        if (!IntersectPoint(e, eNext, ref pt) && e.XCurr > eNext.XCurr + 1)
                            throw new ClipperException("Intersection error");
                        if (pt.Y > botY)
                        {
                            pt.Y = botY;
                            pt.X = TopX(e, pt.Y);
                        }
                        InsertIntersectNode(e, eNext, pt);
                        SwapPositionsInSEL(e, eNext);
                        isModified = true;
                    }
                    else
                        e = eNext;
                }
                if (e.PrevInSEL != null)
                    e.PrevInSEL.NextInSEL = null;
                else
                    break;
            }
            _sortedEdges = null;
        }



        private static bool EdgesAdjacent(IntersectNode inode)
        {
            Contract.Requires(inode != null);
            Contract.Requires(inode.Edge1 != null);

            return (inode.Edge1.NextInSEL == inode.Edge2) || (inode.Edge1.PrevInSEL == inode.Edge2);
        }

        private bool FixupIntersectionOrder()
        {
            //pre-condition: intersections are sorted bottom-most (then left-most) first.
            //Now it's crucial that intersections are made only between adjacent edges,
            //so to ensure this the order of intersections may need adjusting ...
            var inode = _intersectNodes;
            CopyAELToSEL();
            while (inode != null)
            {
                if (!EdgesAdjacent(inode))
                {
                    var nextNode = inode.Next;
                    while (nextNode != null && !EdgesAdjacent(nextNode))
                        nextNode = nextNode.Next;
                    if (nextNode == null)
                        return false;
                    SwapIntersectNodes(inode, nextNode);
                }
                SwapPositionsInSEL(inode.Edge1, inode.Edge2);
                inode = inode.Next;
            }
            return true;
        }

        private void ProcessIntersectList()
        {
            while (_intersectNodes != null)
            {
                var iNode = _intersectNodes.Next;
                {
                    IntersectEdges(_intersectNodes.Edge1, _intersectNodes.Edge2, _intersectNodes.Pt, Protects.Both);
                    SwapPositionsInAEL(_intersectNodes.Edge1, _intersectNodes.Edge2);
                }
                _intersectNodes = null;
                _intersectNodes = iNode;
            }
        }

        private static long TopX(TEdge edge, long currentY)
        {
            Contract.Requires(edge != null);

            if (currentY == edge.YTop)
                return edge.XTop;
            return edge.XBot + InternalHelpers.Round(edge.Dx * (currentY - edge.YBot));
        }

        private void InsertIntersectNode(TEdge e1, TEdge e2, IntPoint pt)
        {
            Contract.Requires(e1 != null);
            Contract.Requires(e2 != null);

            var newNode = new IntersectNode {
                Edge1 = e1,
                Edge2 = e2,
                Pt = pt,
                Next = null
            };
            if (_intersectNodes == null)
                _intersectNodes = newNode;
            else if (newNode.Pt.Y > _intersectNodes.Pt.Y)
            {
                newNode.Next = _intersectNodes;
                _intersectNodes = newNode;
            }
            else
            {
                var iNode = _intersectNodes;
                while (iNode.Next != null && newNode.Pt.Y < iNode.Next.Pt.Y)
                    iNode = iNode.Next;
                newNode.Next = iNode.Next;
                iNode.Next = newNode;
            }
        }

        private static void SwapIntersectNodes(IntersectNode int1, IntersectNode int2)
        {
            Contract.Requires(int1 != null);
            Contract.Requires(int2 != null);

            var e1 = int1.Edge1;
            var e2 = int1.Edge2;
            var p = int1.Pt;
            int1.Edge1 = int2.Edge1;
            int1.Edge2 = int2.Edge2;
            int1.Pt = int2.Pt;
            int2.Edge1 = e1;
            int2.Edge2 = e2;
            int2.Pt = p;
        }

        private static bool IntersectPoint(TEdge edge1, TEdge edge2, ref IntPoint ip)
        {
            Contract.Requires(edge1 != null);
            Contract.Requires(edge2 != null);

            double b1, b2;
            if (InternalHelpers.SlopesEqual(edge1, edge2))
            {
                ip.Y = edge2.YBot > edge1.YBot ? edge2.YBot : edge1.YBot;
                return false;
            }
            else if (edge1.Dx == 0)
            {
                ip.X = edge1.XBot;
                if (edge2.Dx == HORIZONTAL)
                {
                    ip.Y = edge2.YBot;
                }
                else
                {
                    b2 = edge2.YBot - edge2.XBot / edge2.Dx;
                    ip.Y = InternalHelpers.Round(ip.X / edge2.Dx + b2);
                }
            }
            else if (edge2.Dx == 0)
            {
                ip.X = edge2.XBot;
                if (edge1.Dx == HORIZONTAL)
                {
                    ip.Y = edge1.YBot;
                }
                else
                {
                    b1 = edge1.YBot - edge1.XBot / edge1.Dx;
                    ip.Y = InternalHelpers.Round(ip.X / edge1.Dx + b1);
                }
            }
            else
            {
                b1 = edge1.XBot - edge1.YBot * edge1.Dx;
                b2 = edge2.XBot - edge2.YBot * edge2.Dx;
                var q = (b2 - b1) / (edge1.Dx - edge2.Dx);
                ip.Y = InternalHelpers.Round(q);
                ip.X = Math.Abs(edge1.Dx) < Math.Abs(edge2.Dx) ? InternalHelpers.Round(edge1.Dx * q + b1) : InternalHelpers.Round(edge2.Dx * q + b2);
            }

            if (ip.Y < edge1.YTop || ip.Y < edge2.YTop)
            {
                if (edge1.YTop > edge2.YTop)
                {
                    ip.X = edge1.XTop;
                    ip.Y = edge1.YTop;
                    return TopX(edge2, edge1.YTop) < edge1.XTop;
                }
                else
                {
                    ip.X = edge2.XTop;
                    ip.Y = edge2.YTop;
                    return TopX(edge1, edge2.YTop) > edge2.XTop;
                }
            }
            else
                return true;
        }

        private void DisposeIntersectNodes()
        {
            while (_intersectNodes != null)
            {
                var iNode = _intersectNodes.Next;
                _intersectNodes = null;
                _intersectNodes = iNode;
            }
        }

        private void ProcessEdgesAtTopOfScanbeam(long topY)
        {
            var e = _activeEdges;
            while (e != null)
            {
                //1. process maxima, treating them as if they're 'bent' horizontal edges,
                //   but exclude maxima with horizontal edges. nb: e can't be a horizontal.
                if (InternalHelpers.IsMaxima(e, topY) && InternalHelpers.GetMaximaPair(e).Dx != HORIZONTAL)
                {
                    //'e' might be removed from AEL, as may any following edges so ...
                    var ePrev = e.PrevInAEL;
                    DoMaxima(e, topY);
                    e = ePrev == null ? _activeEdges : ePrev.NextInAEL;
                }
                else
                {
                    var intermediateVert = InternalHelpers.IsIntermediate(e, topY);
                    //2. promote horizontal edges, otherwise update xcurr and ycurr ...
                    if (intermediateVert && e.NextInLML.Dx == HORIZONTAL)
                    {
                        if (e.OutIdx >= 0)
                        {
                            AddOutPt(e, new IntPoint(e.XTop, e.YTop));

                            foreach (var horzJoin in _horizJoins)
                            {
                                IntPoint pt, pt2;
                                var hj = horzJoin;
                                if (GetOverlapSegment(new IntPoint(hj.Edge.XBot, hj.Edge.YBot),
                                    new IntPoint(hj.Edge.XTop, hj.Edge.YTop),
                                    new IntPoint(e.NextInLML.XBot, e.NextInLML.YBot),
                                    new IntPoint(e.NextInLML.XTop, e.NextInLML.YTop), out pt, out pt2))
                                    AddJoin(hj.Edge, e.NextInLML, hj.SavedIdx, e.OutIdx);
                            }

                            AddHorzJoin(e.NextInLML, e.OutIdx);
                        }
                        UpdateEdgeIntoAEL(ref e);
                        AddEdgeToSEL(e);
                    }
                    else
                    {
                        e.XCurr = TopX(e, topY);
                        e.YCurr = topY;
                        if (ForceSimple && e.PrevInAEL != null &&
                            e.PrevInAEL.XCurr == e.XCurr &&
                            e.OutIdx >= 0 && e.PrevInAEL.OutIdx >= 0)
                        {
                            AddOutPt(intermediateVert ? e.PrevInAEL : e, new IntPoint(e.XCurr, topY));
                        }
                    }
                    e = e.NextInAEL;
                }
            }

            //3. Process horizontals at the top of the scanbeam ...
            ProcessHorizontals();

            //4. Promote intermediate vertices ...
            e = _activeEdges;
            while (e != null)
            {
                if (InternalHelpers.IsIntermediate(e, topY))
                {
                    if (e.OutIdx >= 0)
                        AddOutPt(e, new IntPoint(e.XTop, e.YTop));
                    UpdateEdgeIntoAEL(ref e);

                    //if output polygons share an edge, they'll need joining later ...
                    var ePrev = e.PrevInAEL;
                    var eNext = e.NextInAEL;
                    if (ePrev != null && ePrev.XCurr == e.XBot &&
                        ePrev.YCurr == e.YBot && e.OutIdx >= 0 &&
                        ePrev.OutIdx >= 0 && ePrev.YCurr > ePrev.YTop &&
                        InternalHelpers.SlopesEqual(e, ePrev))
                    {
                        AddOutPt(ePrev, new IntPoint(e.XBot, e.YBot));
                        AddJoin(e, ePrev, -1, -1);
                    }
                    else if (eNext != null && eNext.XCurr == e.XBot &&
                             eNext.YCurr == e.YBot && e.OutIdx >= 0 &&
                             eNext.OutIdx >= 0 && eNext.YCurr > eNext.YTop &&
                             InternalHelpers.SlopesEqual(e, eNext))
                    {
                        AddOutPt(eNext, new IntPoint(e.XBot, e.YBot));
                        AddJoin(e, eNext, -1, -1);
                    }
                }
                e = e.NextInAEL;
            }
        }

        private void DoMaxima(TEdge e, long topY)
        {
            Contract.Requires(e != null);
            Contract.Requires(e.Next != null);

            var eMaxPair = InternalHelpers.GetMaximaPair(e);
            var x = e.XTop;
            var eNext = e.NextInAEL;
            while (eNext != eMaxPair)
            {
                if (eNext == null)
                    throw new ClipperException("DoMaxima error");
                IntersectEdges(e, eNext, new IntPoint(x, topY), Protects.Both);
                SwapPositionsInAEL(e, eNext);
                eNext = e.NextInAEL;
            }
            if (e.OutIdx < 0 && eMaxPair.OutIdx < 0)
            {
                DeleteFromAEL(e);
                DeleteFromAEL(eMaxPair);
            }
            else if (e.OutIdx >= 0 && eMaxPair.OutIdx >= 0)
            {
                IntersectEdges(e, eMaxPair, new IntPoint(x, topY), Protects.None);
            }
            else
                throw new ClipperException("DoMaxima error");
        }

        



        private static int PointCount(OutPt pts)
        {
            if (pts == null)
                return 0;

            var result = 0;
            var p = pts;
            do
            {
                result++;
                p = p.Next;
            } while (p != pts);
            return result;
        }



        private void BuildResultPointList(List<List<IntPoint>> polyg)
        {
            Contract.Requires(polyg != null);

            polyg.Clear();
            foreach (var outRec in _polyOuts)
            {
                if (outRec.Pts == null)
                    continue;
                var p = outRec.Pts;
                var cnt = PointCount(p);
                if (cnt < 3)
                    continue;
                var pg = new List<IntPoint>(cnt);
                for (var j = 0; j < cnt; j++)
                {
                    pg.Add(p.Pt);
                    p = p.Prev;
                }
                polyg.Add(pg);
            }
        }



        private void BuildResultPolytree(PolyTree polytree)
        {
            Contract.Requires(polytree != null);

            polytree.Clear();

            //add each output polygon/contour to polytree ...
            foreach (var outRec in _polyOuts)
            {
                var cnt = PointCount(outRec.Pts);
                if (cnt < 3)
                    continue;
                FixHoleLinkage(outRec);
                var pn = new PolyNode();
                polytree.AllPolys.Add(pn);
                outRec.PolyNode = pn;
                var op = outRec.Pts;
                for (var j = 0; j < cnt; j++)
                {
                    pn.Polygon.Add(op.Pt);
                    op = op.Prev;
                }
            }

            //fixup PolyNode links etc ...
            foreach (var outRec in _polyOuts.Where(outRec => outRec.PolyNode != null))
            {
                if (outRec.FirstLeft == null)
                    polytree.AddChild(outRec.PolyNode);
                else
                    outRec.FirstLeft.PolyNode.AddChild(outRec.PolyNode);
            }
        }



        private static void FixupOutPolygon(OutRec outRec)
        {
            Contract.Requires(outRec != null);
            Contract.Requires(outRec.Pts != null);

            //FixupOutPolygon() - removes duplicate points and simplifies consecutive
            //parallel edges by removing the middle vertex.
            OutPt lastOk = null;
            outRec.BottomPt = null;
            var pp = outRec.Pts;
            for (;;)
            {
                if (pp.Prev == pp || pp.Prev == pp.Next)
                {
                    DisposeOutPts(pp);
                    outRec.Pts = null;
                    return;
                }
                //test for duplicate points and for same slope (cross-product) ...
                if (pp.Pt.Equals(pp.Next.Pt) || InternalHelpers.SlopesEqual(pp.Prev.Pt, pp.Pt, pp.Next.Pt))
                {
                    lastOk = null;
                    pp.Prev.Next = pp.Next;
                    pp.Next.Prev = pp.Prev;
                    pp = pp.Prev;
                }
                else if (pp == lastOk)
                    break;
                else
                {
                    if (lastOk == null)
                        lastOk = pp;
                    pp = pp.Next;
                }
            }
            outRec.Pts = pp;
        }



        private bool JoinPoints(JoinRec j, out OutPt p1, out OutPt p2)
        {
            Contract.Requires(j != null);
            Contract.Requires(j.Poly1Idx >= 0 && j.Poly1Idx < _polyOuts.Count);
            Contract.Requires(j.Poly2Idx >= 0 && j.Poly2Idx < _polyOuts.Count);

            p1 = null;
            p2 = null;
            var outRec1 = _polyOuts[j.Poly1Idx];
            var outRec2 = _polyOuts[j.Poly2Idx];
            if (outRec1 == null || outRec2 == null)
                return false;
            var pp1A = outRec1.Pts;
            var pp2A = outRec2.Pts;
            IntPoint pt1 = j.Pt2A, pt2 = j.Pt2B;
            IntPoint pt3 = j.Pt1A, pt4 = j.Pt1B;
            if (!FindSegment(ref pp1A, ref pt1, ref pt2))
                return false;
            if (outRec1 == outRec2)
            {
                //we're searching the same polygon for overlapping segments so
                //segment 2 mustn't be the same as segment 1 ...
                pp2A = pp1A.Next;
                if (!FindSegment(ref pp2A, ref pt3, ref pt4) || (pp2A == pp1A))
                    return false;
            }
            else if (!FindSegment(ref pp2A, ref pt3, ref pt4))
                return false;

            if (!GetOverlapSegment(pt1, pt2, pt3, pt4, out pt1, out pt2))
                return false;

            OutPt p3, p4, prev = pp1A.Prev;
            //get p1 & p2 polypts - the overlap start & endpoints on poly1
            if (pp1A.Pt.Equals(pt1))
                p1 = pp1A;
            else if (prev.Pt.Equals(pt1))
                p1 = prev;
            else
                p1 = InsertPolyPtBetween(pp1A, prev, pt1);

            if (pp1A.Pt.Equals(pt2))
                p2 = pp1A;
            else if (prev.Pt.Equals(pt2))
                p2 = prev;
            else if ((p1 == pp1A) || (p1 == prev))
                p2 = InsertPolyPtBetween(pp1A, prev, pt2);
            else if (Pt3IsBetweenPt1AndPt2(pp1A.Pt, p1.Pt, pt2))
                p2 = InsertPolyPtBetween(pp1A, p1, pt2);
            else
                p2 = InsertPolyPtBetween(p1, prev, pt2);

            //get p3 & p4 polypts - the overlap start & endpoints on poly2
            prev = pp2A.Prev;
            if (pp2A.Pt.Equals(pt1))
                p3 = pp2A;
            else if (prev.Pt.Equals(pt1))
                p3 = prev;
            else
                p3 = InsertPolyPtBetween(pp2A, prev, pt1);

            if (pp2A.Pt.Equals(pt2))
                p4 = pp2A;
            else if (prev.Pt.Equals(pt2))
                p4 = prev;
            else if ((p3 == pp2A) || (p3 == prev))
                p4 = InsertPolyPtBetween(pp2A, prev, pt2);
            else if (Pt3IsBetweenPt1AndPt2(pp2A.Pt, p3.Pt, pt2))
                p4 = InsertPolyPtBetween(pp2A, p3, pt2);
            else
                p4 = InsertPolyPtBetween(p3, prev, pt2);

            //p1.pt == p3.pt and p2.pt == p4.pt so join p1 to p3 and p2 to p4 ...
            if (p1.Next == p2 && p3.Prev == p4)
            {
                p1.Next = p3;
                p3.Prev = p1;
                p2.Prev = p4;
                p4.Next = p2;
                return true;
            }
            else if (p1.Prev == p2 && p3.Next == p4)
            {
                p1.Prev = p3;
                p3.Next = p1;
                p2.Next = p4;
                p4.Prev = p2;
                return true;
            }
            else
                return false; //an orientation is probably wrong
        }

        

        private void FixupJoinRecs(JoinRec j, OutPt pt, int startIdx)
        {
            for (var k = startIdx; k < _joins.Count; k++)
            {
                var j2 = _joins[k];
                if (j2.Poly1Idx == j.Poly1Idx && InternalHelpers.PointIsVertex(j2.Pt1A, pt))
                    j2.Poly1Idx = j.Poly2Idx;
                if (j2.Poly2Idx == j.Poly1Idx && InternalHelpers.PointIsVertex(j2.Pt2A, pt))
                    j2.Poly2Idx = j.Poly2Idx;
            }
        }

        

        private static bool Poly2ContainsPoly1(OutPt outPt1, OutPt outPt2)
        {
            Contract.Requires(outPt1 != null);
            Contract.Requires(outPt2 != null);

            var pt = outPt1;
            //Because the polygons may be touching, we need to find a vertex that
            //isn't touching the other polygon ...
            if (InternalHelpers.PointOnPolygon(pt.Pt, outPt2))
            {
                pt = pt.Next;
                while (pt != outPt1 && InternalHelpers.PointOnPolygon(pt.Pt, outPt2))
                    pt = pt.Next;
                if (pt == outPt1)
                    return true;
            }
            return InternalHelpers.PointInPolygon(pt.Pt, outPt2);
        }

        

        private void FixupFirstLefts1(OutRec oldOutRec, OutRec newOutRec)
        {
            foreach (var outRec in _polyOuts.Where(outRec => outRec.Pts != null && outRec.FirstLeft == oldOutRec && Poly2ContainsPoly1(outRec.Pts, newOutRec.Pts)))
                outRec.FirstLeft = newOutRec;
        }

        

        private void FixupFirstLefts2(OutRec oldOutRec, OutRec newOutRec)
        {
            foreach (var outRec in _polyOuts.Where(outRec => outRec.FirstLeft == oldOutRec))
                outRec.FirstLeft = newOutRec;
        }

        

        private void JoinCommonEdges()
        {
            for (var i = 0; i < _joins.Count; i++)
            {
                var j = _joins[i];

                var outRec1 = GetOutRec(j.Poly1Idx);
                var outRec2 = GetOutRec(j.Poly2Idx);

                if (outRec1.Pts == null || outRec2.Pts == null)
                    continue;

                //get the polygon fragment with the correct hole state (FirstLeft)
                //before calling JoinPoints() ...
                OutRec holeStateRec;
                if (outRec1 == outRec2)
                    holeStateRec = outRec1;
                else if (Param1RightOfParam2(outRec1, outRec2))
                    holeStateRec = outRec2;
                else if (Param1RightOfParam2(outRec2, outRec1))
                    holeStateRec = outRec1;
                else
                    holeStateRec = GetLowermostRec(outRec1, outRec2);

                OutPt p1, p2;
                if (!JoinPoints(j, out p1, out p2))
                    continue;

                if (outRec1 == outRec2)
                {
                    //instead of joining two polygons, we've just created a new one by
                    //splitting one polygon into two.
                    outRec1.Pts = p1;
                    outRec1.BottomPt = null;
                    outRec2 = CreateOutRec();
                    outRec2.Pts = p2;

                    if (Poly2ContainsPoly1(outRec2.Pts, outRec1.Pts))
                    {
                        //outRec2 is contained by outRec1 ...
                        outRec2.IsHole = !outRec1.IsHole;
                        outRec2.FirstLeft = outRec1;

                        FixupJoinRecs(j, p2, i + 1);

                        //fixup FirstLeft pointers that may need reassigning to OutRec1
                        if (_usingPolyTree)
                            FixupFirstLefts2(outRec2, outRec1);

                        FixupOutPolygon(outRec1); //nb: do this BEFORE testing orientation
                        FixupOutPolygon(outRec2); //    but AFTER calling FixupJoinRecs()

                        if ((outRec2.IsHole ^ ReverseSolution) == outRec2.Area() > 0)
                            ReversePolyPtLinks(outRec2.Pts);

                    }
                    else if (Poly2ContainsPoly1(outRec1.Pts, outRec2.Pts))
                    {
                        //outRec1 is contained by outRec2 ...
                        outRec2.IsHole = outRec1.IsHole;
                        outRec1.IsHole = !outRec2.IsHole;
                        outRec2.FirstLeft = outRec1.FirstLeft;
                        outRec1.FirstLeft = outRec2;

                        FixupJoinRecs(j, p2, i + 1);

                        //fixup FirstLeft pointers that may need reassigning to OutRec1
                        if (_usingPolyTree)
                            FixupFirstLefts2(outRec1, outRec2);

                        FixupOutPolygon(outRec1); //nb: do this BEFORE testing orientation
                        FixupOutPolygon(outRec2); //    but AFTER calling FixupJoinRecs()

                        if ((outRec1.IsHole ^ ReverseSolution) == outRec1.Area() > 0)
                            ReversePolyPtLinks(outRec1.Pts);
                    }
                    else
                    {
                        //the 2 polygons are completely separate ...
                        outRec2.IsHole = outRec1.IsHole;
                        outRec2.FirstLeft = outRec1.FirstLeft;

                        FixupJoinRecs(j, p2, i + 1);

                        //fixup FirstLeft pointers that may need reassigning to OutRec2
                        if (_usingPolyTree)
                            FixupFirstLefts1(outRec1, outRec2);

                        FixupOutPolygon(outRec1); //nb: do this BEFORE testing orientation
                        FixupOutPolygon(outRec2); //    but AFTER calling FixupJoinRecs()
                    }
                }
                else
                {
                    //joined 2 polygons together ...

                    //cleanup redundant edges ...
                    FixupOutPolygon(outRec1);

                    outRec2.Pts = null;
                    outRec2.BottomPt = null;
                    outRec2.Idx = outRec1.Idx;

                    outRec1.IsHole = holeStateRec.IsHole;
                    if (holeStateRec == outRec2)
                        outRec1.FirstLeft = outRec2.FirstLeft;
                    outRec2.FirstLeft = outRec1;

                    //fixup FirstLeft pointers that may need reassigning to OutRec1
                    if (_usingPolyTree)
                        FixupFirstLefts2(outRec2, outRec1);
                }
            }
        }

        private static void UpdateOutPtIdxs(OutRec outRec)
        {
            Contract.Requires(outRec != null);
            Contract.Requires(outRec.Pts != null);

            var op = outRec.Pts;
            do
            {
                op = op.Prev;
            } while (op != outRec.Pts);
        }

        private void DoSimplePolygons()
        {
            var i = 0;
            while (i < _polyOuts.Count)
            {
                var outrec = _polyOuts[i++];
                var op = outrec.Pts;
                if (op == null)
                    continue;
                do //for each Pt in Polygon until duplicate found do ...
                {
                    var op2 = op.Next;
                    while (op2 != outrec.Pts)
                    {
                        if (op.Pt.Equals(op2.Pt) && op2.Next != op && op2.Prev != op)
                        {
                            //split the polygon into two ...
                            var op3 = op.Prev;
                            var op4 = op2.Prev;
                            op.Prev = op4;
                            op4.Next = op;
                            op2.Prev = op3;
                            op3.Next = op2;

                            outrec.Pts = op;
                            var outrec2 = CreateOutRec();
                            outrec2.Pts = op2;
                            UpdateOutPtIdxs(outrec2);
                            if (Poly2ContainsPoly1(outrec2.Pts, outrec.Pts))
                            {
                                //OutRec2 is contained by OutRec1 ...
                                outrec2.IsHole = !outrec.IsHole;
                                outrec2.FirstLeft = outrec;
                            }
                            else if (Poly2ContainsPoly1(outrec.Pts, outrec2.Pts))
                            {
                                //OutRec1 is contained by OutRec2 ...
                                outrec2.IsHole = outrec.IsHole;
                                outrec.IsHole = !outrec2.IsHole;
                                outrec2.FirstLeft = outrec.FirstLeft;
                                outrec.FirstLeft = outrec2;
                            }
                            else
                            {
                                //the 2 polygons are separate ...
                                outrec2.IsHole = outrec.IsHole;
                                outrec2.FirstLeft = outrec.FirstLeft;
                            }
                            op2 = op; //ie get ready for the next iteration
                        }
                        op2 = op2.Next;
                    }
                    op = op.Next;
                } while (op != outrec.Pts);
            }
        }

        private static bool UpdateBotPt(IntPoint pt, ref IntPoint botPt)
        {
            if (pt.Y > botPt.Y || (pt.Y == botPt.Y && pt.X < botPt.X))
            {
                botPt = pt;
                return true;
            }
            else
                return false;
        }

        #region public static helpers
        public static void ReversePolygons(IReadOnlyList<List<IntPoint>> polys)
        {
            Contract.Requires(polys != null);
            Contract.Requires(Contract.ForAll(polys, a => a != null));

            foreach (var poly in polys)
                poly.Reverse();
        }

        public static bool Orientation(IReadOnlyList<IntPoint> poly)
        {
            Contract.Requires(poly != null);

            return Area(poly) >= 0;
        }

        public static double Area(IReadOnlyList<IntPoint> poly)
        {
            Contract.Requires(poly != null);

            var highI = poly.Count - 1;
            if (highI < 2)
                return 0;

            var a = new decimal(poly[highI].X + poly[0].X) * new decimal(poly[0].Y - poly[highI].Y);
            for (var i = 1; i <= highI; ++i)
                a += new decimal(poly[i - 1].X + poly[i].X) * new decimal(poly[i].Y - poly[i - 1].Y);
            return decimal.ToDouble(a) / 2;

        }

        public static List<IntPoint> CleanPolygon(List<IntPoint> poly, double distance = 1.415)
        {
            Contract.Requires(poly != null);
            Contract.Ensures(Contract.Result<List<IntPoint>>() != null);

            //distance = proximity in units/pixels below which vertices
            //will be stripped. Default ~= sqrt(2) so when adjacent
            //vertices have both x & y coords within 1 unit, then
            //the second vertex will be stripped.
            var distSqrd = distance * distance;
            var highI = poly.Count - 1;
            var result = new List<IntPoint>(highI + 1);
            while (highI > 0 && poly[highI].IsClose(poly[0], distSqrd))
                highI--;
            if (highI < 2)
                return result;
            var pt = poly[highI];
            var i = 0;
            for (; ; )
            {
                while (i < highI && pt.IsClose(poly[i], distSqrd))
                    i += 2;
                var i2 = i;
                while (i < highI && (poly[i].IsClose(poly[i + 1], distSqrd) || InternalHelpers.SlopesNearColinear(pt, poly[i], poly[i + 1], distSqrd)))
                    i++;
                if (i >= highI)
                    break;
                else if (i != i2)
                    continue;
                pt = poly[i++];
                result.Add(pt);
            }
            if (i <= highI)
                result.Add(poly[i]);
            i = result.Count;
            if (i > 2 && InternalHelpers.SlopesNearColinear(result[i - 2], result[i - 1], result[0], distSqrd))
                result.RemoveAt(i - 1);
            if (result.Count < 3)
                result.Clear();
            return result;
        }

        public static List<List<IntPoint>> CleanPolygons(IEnumerable<List<IntPoint>> polys, double distance = 1.415)
        {
            Contract.Requires(polys != null);
            Contract.Requires(Contract.ForAll(polys, a => a != null));
            Contract.Ensures(Contract.Result<List<List<IntPoint>>>() != null);

            return new List<List<IntPoint>>(polys.Select(t => CleanPolygon(t, distance)));
        }

        public static void PolyTreeToPolygons(PolyTree polytree, List<List<IntPoint>> polygons)
        {
            Contract.Requires(polytree != null);
            Contract.Requires(polygons != null);

            polygons.Clear();
            InternalHelpers.AddPolyNodeToPolygons(polytree, polygons);
        }

        /// <summary>
        /// Convert a self intersecting polygon into simple polygons
        /// </summary>
        /// <param name="poly"></param>
        /// <param name="fillType"></param>
        /// <returns></returns>
        public static List<List<IntPoint>> SimplifyPolygon(List<IntPoint> poly, PolyFillType fillType = PolyFillType.EvenOdd)
        {
            Contract.Requires(poly != null);
            Contract.Ensures(Contract.Result<List<List<IntPoint>>>() != null);

            var result = new List<List<IntPoint>>();
            var c = new Clipper { ForceSimple = true };
            c.AddPolygon(poly, PolyType.Subject);
            c.Execute(ClipType.Union, result, fillType, fillType);
            return result;
        }

        /// <summary>
        /// Convert self intersecting polygons into simple polygons
        /// </summary>
        /// <param name="polys"></param>
        /// <param name="fillType"></param>
        /// <returns></returns>
        public static List<List<IntPoint>> SimplifyPolygons(IEnumerable<List<IntPoint>> polys, PolyFillType fillType = PolyFillType.EvenOdd)
        {
            Contract.Requires(polys != null);
            Contract.Requires(Contract.ForAll(polys, a => a != null));
            Contract.Ensures(Contract.Result<List<List<IntPoint>>>() != null);

            var result = new List<List<IntPoint>>();
            var c = new Clipper { ForceSimple = true };
            c.AddPolygons(polys, PolyType.Subject);
            c.Execute(ClipType.Union, result, fillType, fillType);
            return result;
        }

        public static List<List<IntPoint>> OffsetPolygons(IReadOnlyList<IReadOnlyList<IntPoint>> poly, double delta, JoinType jointype = JoinType.Square, double miterLimit = 0, bool autoFix = true)
        {
            Contract.Requires(poly != null);
            Contract.Requires(Contract.ForAll(poly, p => p != null));
            Contract.Ensures(Contract.Result<List<List<IntPoint>>>() != null);

            var result = new List<List<IntPoint>>();

            //AutoFix - fixes polygon orientation if necessary and removes 
            //duplicate vertices. Can be set false when you're sure that polygon
            //orientation is correct and that there are no duplicate vertices.
            if (!autoFix)
                PolyOffsetBuilder.Offset(poly, result, true, delta, jointype, EndType.Closed, miterLimit);
            else
            {
                var fixedPolys = poly.Select(a => a.ToList()).ToList();

                int len = fixedPolys.Count, botI = 0;
                while (botI < len && fixedPolys[botI].Count == 0)
                    botI++;
                if (botI == len)
                    return result;

                //botPt: used to find the lowermost (in inverted Y-axis) & leftmost point
                //This point (on pts[botI]) must be on an outer polygon ring and if 
                //its orientation is false (counterclockwise) then assume all polygons 
                //need reversing ...
                var botPt = fixedPolys[botI][0];
                for (var i = botI; i < len; ++i)
                {
                    if (fixedPolys[i].Count == 0)
                        continue;
                    if (UpdateBotPt(fixedPolys[i][0], ref botPt))
                        botI = i;
                    for (var j = fixedPolys[i].Count - 1; j > 0; j--)
                    {
                        if (fixedPolys[i][j].Equals(fixedPolys[i][j - 1]))
                            fixedPolys[i].RemoveAt(j);
                        else if (UpdateBotPt(fixedPolys[i][j], ref botPt))
                            botI = i;
                    }
                }
                if (!Orientation(fixedPolys[botI]))
                    ReversePolygons(fixedPolys);

                PolyOffsetBuilder.Offset(fixedPolys, result, true, delta, jointype, EndType.Closed, miterLimit);
            }
            return result;
        }

        //public static List<List<IntPoint>> OffsetPolyLines(IReadOnlyList<IReadOnlyList<IntPoint>> lines, double delta, JoinType jointype, EndType endtype, double limit)
        //{
        //    Contract.Ensures(Contract.Result<List<List<IntPoint>>>() != null);
        //    Contract.Requires(lines != null);

        //    var result = new List<List<IntPoint>>();

        //    //automatically strip duplicate points because it gets complicated with
        //    //open and closed lines and when to strip duplicates across begin-end ...
        //    var pts = new List<List<IntPoint>>(lines.Select(a => a.ToList()));
        //    foreach (var pt in pts)
        //    {
        //        for (var j = pt.Count - 1; j > 0; j--)
        //            if (pt[j].Equals(pt[j - 1]))
        //                pt.RemoveAt(j);
        //    }

        //    if (endtype == EndType.Closed)
        //    {
        //        var sz = pts.Count;
        //        pts.Capacity = sz * 2;
        //        for (var i = 0; i < sz; ++i)
        //        {
        //            var line = new List<IntPoint>(pts[i]);
        //            line.Reverse();
        //            pts.Add(line);
        //        }
        //        PolyOffsetBuilder.Offset(pts, result, true, delta, jointype, endtype, limit);
        //    }
        //    else
        //        PolyOffsetBuilder.Offset(pts, result, false, delta, jointype, endtype, limit);

        //    return result;
        //}
        #endregion
    }
}
