using System;
using System.Diagnostics.Contracts;

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


namespace ClipperRedux
{
    [Flags]
    internal enum EdgeSide
    {
        Left = 1,
        Right = 2
    }

    [Flags]
    internal enum Protects
    {
        None = 0,
        Left = 1,
        Right = 2,
        Both = 3
    }

    internal enum Direction
    {
        RightToLeft,
        LeftToRight
    }

    internal class TEdge
    {
        public long XBot;
        public long YBot;
        public long XCurr;
        public long YCurr;
        public long XTop;
        public long YTop;
        public double Dx;
        public long DeltaX;
        public long DeltaY;
        public PolyType PolyType;
        public EdgeSide Side;
        public int WindDelta; //1 or -1 depending on winding direction
        public int WindCnt;
        public int WindCnt2; //winding count of the opposite polytype
        public int OutIdx;
        public TEdge Next;
        public TEdge Prev;
        public TEdge NextInLML;
        public TEdge NextInAEL;
        public TEdge PrevInAEL;
        public TEdge NextInSEL;
        public TEdge PrevInSEL;
    }

    internal class IntersectNode
    {
        public TEdge Edge1;
        public TEdge Edge2;
        public IntPoint Pt;
        public IntersectNode Next;
    }

    internal class LocalMinima
    {
        public long Y;
        public TEdge LeftBound;
        public TEdge RightBound;
        public LocalMinima Next;
    }

    internal class Scanbeam
    {
        public readonly long Y;
        public Scanbeam Next;

        public Scanbeam(long y, Scanbeam next)
        {
            Y = y;
            Next = next;
        }
    }

    internal class OutRec
    {
        public int Idx;
        public bool IsHole;
        public OutRec FirstLeft; //see comments in clipper.pas
        public OutPt Pts;
        public OutPt BottomPt;
        public PolyNode PolyNode;

        internal double Area()
        {
            var op = Pts;
            if (op == null)
                return 0;

            var a = 0m;
            do
            {
                Contract.Assume(op != null);

                a += new decimal(op.Pt.X + op.Prev.Pt.X) * new decimal(op.Prev.Pt.Y - op.Pt.Y);
                op = op.Next;
            } while (op != Pts);
            return (double)a / 2;
        }
    }

    internal class OutPt
    {
        public IntPoint Pt;
        public OutPt Next;
        public OutPt Prev;

        internal OutPt GetBottomPt()
        {
            var pp = this;

            OutPt dups = null;
            OutPt p = Next;
            while (p != pp)
            {
                if (p.Pt.Y > pp.Pt.Y)
                {
                    pp = p;
                    dups = null;
                }
                else if (p.Pt.Y == pp.Pt.Y && p.Pt.X <= pp.Pt.X)
                {
                    if (p.Pt.X < pp.Pt.X)
                    {
                        dups = null;
                        pp = p;
                    }
                    else
                    {
                        if (p.Next != pp && p.Prev != pp)
                            dups = p;
                    }
                }
                p = p.Next;
            }
            if (dups != null)
            {
                //there appears to be at least 2 vertices at bottomPt so ...
                while (dups != p)
                {
                    if (!InternalHelpers.FirstIsBottomPt(p, dups))
                        pp = dups;
                    dups = dups.Next;
                    while (!dups.Pt.Equals(pp.Pt))
                        dups = dups.Next;
                }
            }
            return pp;
        }
    }

    internal class JoinRec
    {
        public IntPoint Pt1A;
        public IntPoint Pt1B;
        public int Poly1Idx;
        public IntPoint Pt2A;
        public IntPoint Pt2B;
        public int Poly2Idx;
    }

    internal class HorzJoinRec
    {
        public TEdge Edge;
        public int SavedIdx;
    }
}
