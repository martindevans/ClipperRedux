using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using PrimitiveSvgBuilder;

namespace ClipperRedux.Tests
{
    /// <summary>
    /// The output most of the tests here will be SVG representing the output of the operation (paste it into something like jsfiddle)
    /// </summary>
    [TestClass]
    public class ExampleUsage
    {
        private static string Svg(IEnumerable<IEnumerable<IntPoint>> shapes)
        {
            var b = new SvgBuilder();

            foreach (var shape in shapes)
                b = b.Outline(shape.Select(a => new Vector2(a.X, a.Y)).ToArray());

            return b.ToString();
        }

        [TestMethod]
        public void Orientation()
        {
            Assert.IsFalse(Clipper.Orientation(new List<IntPoint>
            {
                new IntPoint(0, 0),
                new IntPoint(0, 10),
                new IntPoint(10, 10),
                new IntPoint(10, 0)
            }));
        }

        [TestMethod]
        public void OffsetPolygon()
        {
            //Grow a polygon (10x10) by 10
            var result = Clipper.OffsetPolygons(new List<List<IntPoint>> {
                new List<IntPoint> {
                    new IntPoint(0, 0),
                    new IntPoint(0, 10),
                    new IntPoint(10, 10),
                    new IntPoint(10, 0)
                }
            }, 20, JoinType.Miter);

            //Rather than checking all the points just do a quick area check.
            //We've offset a 10x10 shape by +20 in all directions so now it's 50x50
            //50x50=2500 area
            Assert.AreEqual(2500, Clipper.Area(result.Single()));

            Console.WriteLine(Svg(result));
        }

        //[TestMethod]
        //public void OffsetPolyline()
        //{
        //    var result = Clipper.OffsetPolyLines(new List<List<IntPoint>> {
        //        new List<IntPoint> {
        //            new IntPoint(0, 0),
        //            new IntPoint(0, 50),
        //            new IntPoint(50, 50),
        //        }
        //    }, 10, JoinType.Miter, EndType.Round, 0.01);

        //    Console.WriteLine(Svg(result));
        //}
    }
}
