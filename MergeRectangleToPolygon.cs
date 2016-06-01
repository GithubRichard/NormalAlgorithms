using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Tencent.algorithms
{
    internal class MergeRectangleToPolygon
    {


        // Return the union of the two polygons.
        public List<PointF> FindPolygonUnion(List<PointF>[] polygons)
        {
            // Create the result polygon.
            List<PointF> union = new List<PointF>();
            if (polygons.Length > 2)
            {
                List<PointF>[] twoPolygon = new List<PointF>[]{polygons[0], polygons[1]};
                union = FindPolygonUnion(twoPolygon);
                for (int ix = 2; ix < polygons.Length; ix++)
                {
                    twoPolygon[0] = union;
                    twoPolygon[1] = polygons[ix];
                    union = FindPolygonUnion(twoPolygon);
                }
            }
            else if (polygons.Length == 0)
            {
                return null;
            }
            else if (polygons.Length == 1)
            {
                return polygons[0];
            }
            else
            {
                // Find the lower-leftmost point in either polygon.
                int cur_pgon = 0;
                int cur_index = 0;
                PointF cur_point = polygons[cur_pgon][cur_index];
                for (int pgon = 0; pgon < 2; pgon++)
                {
                    for (int index = 0; index < polygons[pgon].Count; index++)
                    {
                        PointF test_point = polygons[pgon][index];
                        if ((test_point.X < cur_point.X) ||
                            ((test_point.X == cur_point.X) &&
                             (test_point.Y > cur_point.Y)))
                        {
                            cur_pgon = pgon;
                            cur_index = index;
                            cur_point = polygons[cur_pgon][cur_index];
                        }
                    }
                }

                

                // Start here.
                PointF start_point = cur_point;
                union.Add(start_point);

                // Start traversing the polygons.
                // Repeat until we return to the starting point.
                for (int ix = 0; ix < 2000; ix++)
                {
                    // Find the next point.
                    int next_index = (cur_index + 1) % polygons[cur_pgon].Count;
                    PointF next_point = polygons[cur_pgon][next_index];

                    // Each time through the loop:
                    //      cur_pgon is the index of the polygon we're following
                    //      cur_point is the last point added to the union
                    //      next_point is the next point in the current polygon
                    //      next_index is the index of next_point

                    // See if this segment intersects
                    // any of the other polygon's segments.
                    int other_pgon = (cur_pgon + 1) % 2;

                    // Keep track of the closest intersection.
                    PointF best_intersection = new PointF(0, 0);
                    int best_index1 = -1;
                    float best_t = 2f;

                    for (int index1 = 0; index1 < polygons[other_pgon].Count; index1++)
                    {
                        // Get the index of the next point in the polygon.
                        int index2 = (index1 + 1) % polygons[other_pgon].Count;

                        // See if the segment between points index1
                        // and index2 intersect the current segment.
                        PointF point1 = polygons[other_pgon][index1];
                        PointF point2 = polygons[other_pgon][index2];
                        bool lines_intersect, segments_intersect;
                        PointF intersection, close_p1, close_p2;
                        float t1, t2;
                        FindIntersection(cur_point, next_point, point1, point2,
                            out lines_intersect, out segments_intersect,
                            out intersection, out close_p1, out close_p2, out t1, out t2);

                        if ((segments_intersect) && // The segments intersect
                            (t1 > 0.001) &&         // Not at the previous intersection
                            (t1 < best_t))          // Better than the last intersection found
                        {
                            // See if this is an improvement.
                            if (t1 < best_t)
                            {
                                // Save this intersection.
                                best_t = t1;
                                best_index1 = index1;
                                best_intersection = intersection;
                            }
                        }
                    }

                    // See if we found any intersections.
                    if (best_t < 2f)
                    {
                        // We found an intersection. Use it.
                        union.Add(best_intersection);

                        // Prepare to search for the next point from here.
                        // Start following the other polygon.
                        cur_pgon = (cur_pgon + 1) % 2;
                        cur_point = best_intersection;
                        cur_index = best_index1;
                    }
                    else
                    {
                        // We didn't find an intersection.
                        // Move to the next point in this polygon.
                        cur_point = next_point;
                        cur_index = next_index;

                        // If we've returned to the starting point, we're done.
                        if (cur_point == start_point) break;

                        // Add the current point to the union.
                        union.Add(cur_point);
                    }
                }
            }
            union = filterPoint(union);
            return union;
        }


        // Find the point of intersection between
        // the lines p1 --> p2 and p3 --> p4.
        private void FindIntersection(PointF p1, PointF p2, PointF p3, PointF p4,
            out bool lines_intersect, out bool segments_intersect,
            out PointF intersection, out PointF close_p1, out PointF close_p2,
            out float t1, out float t2)
        {
            // Get the segments' parameters.
            float dx12 = p2.X - p1.X;
            float dy12 = p2.Y - p1.Y;
            float dx34 = p4.X - p3.X;
            float dy34 = p4.Y - p3.Y;

            // Solve for t1 and t2
            float denominator = (dy12 * dx34 - dx12 * dy34);
            t1 = ((p1.X - p3.X) * dy34 + (p3.Y - p1.Y) * dx34) / denominator;
            if (float.IsInfinity(t1))
            {
                // The lines are parallel (or close enough to it).
                lines_intersect = false;
                segments_intersect = false;
                intersection = new PointF(float.NaN, float.NaN);
                close_p1 = new PointF(float.NaN, float.NaN);
                close_p2 = new PointF(float.NaN, float.NaN);
                t2 = float.PositiveInfinity;
                return;
            }
            lines_intersect = true;

            t2 = ((p3.X - p1.X) * dy12 + (p1.Y - p3.Y) * dx12) / -denominator;

            // Find the point of intersection.
            intersection = new PointF(p1.X + dx12 * t1, p1.Y + dy12 * t1);

            // The segments intersect if t1 and t2 are between 0 and 1.
            segments_intersect = ((t1 >= 0) && (t1 <= 1) && (t2 >= 0) && (t2 <= 1));

            // Find the closest points on the segments.
            if (t1 < 0) t1 = 0;
            else if (t1 > 1) t1 = 1;

            if (t2 < 0) t2 = 0;
            else if (t2 > 1) t2 = 1;

            close_p1 = new PointF(p1.X + dx12 * t1, p1.Y + dy12 * t1);
            close_p2 = new PointF(p3.X + dx34 * t2, p3.Y + dy34 * t2);
        }

        private List<PointF> filterPoint(List<PointF> points)
        {
            List<PointF> filterSame = new List<PointF>();
            string pos = "";
            bool has;
            for (int ix = 0; ix < points.Count; ix++)
            {
                has = false;
                for (int iy = 0; iy < filterSame.Count; iy++)
                {
                    if (points[ix].Equals(filterSame[iy]))
                    {
                        has = true;
                        break;
                    }
                }
                if (!has)
                {
                    filterSame.Add(points[ix]);
                }
            }
            return filterSame;
        }

    }


    public class PointF
    {
        public float X;
        public float Y;

        public PointF(float x, float y)
        {
            X = x;
            Y = y;
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
            {
                return false;
            }
            if (obj is PointF)
            {
                return (obj as PointF).X == X && (obj as PointF).Y == Y;
            }
            return base.Equals(obj);
        }

        public override string ToString()
        {
            return X+"_"+Y;
        }
    }



    public class UnitPolygonsUtils
    {
        

        private List<Vector2[]> polygons = new List<Vector2[]>();
        private int curPgon;
        private Vector2 startPoint;
        private int curIndex;

        public UnitPolygonsUtils()
        {
            
        }

        private Vector2 dotV1 = Vector2.zero;
        private Vector2 dotV2 = Vector2.zero;
        private void findIntersectionVertex(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, ref List<Vector2> intersections, out float t1)
        {
            float t2 = 0;
            float dx12 = p2.x - p1.x;
            float dy12 = p2.y - p1.y;
            float dx34 = p4.x - p3.x;
            float dy34 = p4.y - p3.y;

            // Solve for t1 and t2
            float denominator = (dy12 * dx34 - dx12 * dy34);
            t1 = ((p1.x - p3.x) * dy34 + (p3.y - p1.y) * dx34) / denominator;
            if (float.IsInfinity(t1))
            {
                return;
            }

            float dotNum = 0;
            float maxNum = 0;
            //直线重叠
            if (float.IsNaN(t1))
            {
                dotV1.Set(p2.x - p1.x, p2.y - p1.y);
                dotV2.Set(p2.x - p1.x, p2.y - p1.y);
                maxNum = Vector2.Dot(dotV1, dotV2);

                dotV1.Set(p3.x - p1.x, p3.y - p1.y); 
                dotV2.Set(p2.x - p1.x, p2.y - p1.y);
                dotNum = Vector2.Dot(dotV1, dotV2);
                if (dotNum > 0 && dotNum < maxNum)
                    addIntersectionsPoint(ref intersections, p3, p1, p2);
                dotV1.Set(p4.x - p1.x, p4.y - p1.y);
                dotV2.Set(p2.x - p1.x, p2.y - p1.y);
                dotNum = Vector2.Dot(dotV1, dotV2);
                if (dotNum > 0 && dotNum < maxNum)
                    addIntersectionsPoint(ref intersections, p4, p1, p2);
            }

            // Find the point of intersection.
            t2 = ((p3.x - p1.x) * dy12 + (p1.y - p3.y) * dx12) / -denominator;
            // The segments intersect if t1 and t2 are between 0 and 1.
            bool segments_intersect = ((t1 >= 0) && (t1 <= 1) && (t2 >= 0) && (t2 <= 1));
            if (t1 > 0.001 && segments_intersect)
            {
                Vector2 intersectP = new Vector2(p1.x + dx12 * t1, p1.y + dy12 * t1);
                addIntersectionsPoint(ref intersections, intersectP, p1, p2);
            }
            
        }

        private void addIntersectionsPoint(ref List<Vector2> intersections, Vector2 intersectP, Vector2 p1, Vector2 p2)
        {
            if (indexListItem(ref intersections, intersectP) == -1)
            {
                addWithOrde(ref intersections, p1, p2, intersectP);
            }
        }

        private int indexListItem(ref List<Vector2> list, Vector2 item)
        {
            int index = -1;
            for(int ix = 0; ix < list.Count; ix++)
            {
                if (Vector2.Distance(list[ix],item) <= 0.01f)
                {
                    index = ix;
                    break;
                }
            }
            return index;
        }

        private void addWithOrde(ref List<Vector2> list, Vector2 p1, Vector2 p2, Vector2 intersecP)
        {
            if (intersecP.Equals(p1) || intersecP.Equals(p2)) return;
            bool insert = false;
            for (int ix = 0; ix < list.Count; ix++)
            {
                if (Vector2.Distance(list[ix], p1) > Vector2.Distance(intersecP, p1))
                {
                    list.Insert(ix, intersecP);
                    insert = true;
                    break;
                }
            }
            if (!insert)
            {
                list.Add(intersecP);
            }
        }

        private float dotProduct(PointF p1, PointF p2, PointF p3, PointF p4)
        {
            return (p2.X - p1.X) * (p4.X - p3.X) + (p2.Y - p1.Y) * (p4.Y - p3.Y);
        }

        private void findLeftBottomVertex(List<List<Vector2>> polygons)
        {
            int cur_index = 0;
            curPgon = 0;
            startPoint = polygons[curPgon][cur_index];
            for (int pgon = 0; pgon < polygons.Count; pgon++)
            {
                for (int index = 0; index < polygons[pgon].Count; index++)
                {
                    Vector2 test_point = polygons[pgon][index];
                    if ((test_point.x < startPoint.x) ||
                        ((test_point.x == startPoint.x) &&
                         (test_point.y < startPoint.y)))
                    {
                        curPgon = pgon;
                        cur_index = index;
                        startPoint = polygons[curPgon][cur_index];
                    }
                }
            }
        }

        public List<Vector2> UnitPoligons(List<Vector2[]> polygons)
        {
            if (polygons.Count == 1)
            {
                return polygons[0].ToList();
            }

            List<Vector2> units = new List<Vector2>();
            int cur_index = 0, cur_pgon = 0, cur_index2 = 0;
            Vector2 p1, p2, p3, p4;
            List<List<Vector2>> intersectPoints = new List<List<Vector2>>();
            List<List<bool>> intersectVisitPoints = new List<List<bool>>();
            List<Vector2> pointFs;
            List<bool> visits;
            List<Vector2> intersectPos = new List<Vector2>();
            float t1;
            int ix = 0;
            int pgon = 0;
            int pgon2 = 0;
            for (pgon = 0; pgon < polygons.Count; pgon++)
            {
                pointFs = new List<Vector2>();
                intersectPoints.Add(pointFs);
                visits = new List<bool>();
                intersectVisitPoints.Add(visits);
                for (cur_index = 0; cur_index < polygons[pgon].Length; cur_index++)
                {
                    p1 = polygons[pgon][cur_index];
                    p2 = cur_index != polygons[pgon].Length - 1 ? polygons[pgon][cur_index + 1] : polygons[pgon][0];
                    intersectPos.Clear();
                    pointFs.Add(p1);
                    visits.Add(false);
                    for (pgon2 = 0; pgon2 < polygons.Count; pgon2++)
                    {
                        if (pgon != pgon2)
                        {
                            for (cur_index2 = 0; cur_index2 < polygons[pgon2].Length; cur_index2++)
                            {
                                p3 = polygons[pgon2][cur_index2];
                                p4 = cur_index2 != polygons[pgon2].Length - 1 ? polygons[pgon2][cur_index2 + 1] : polygons[pgon2][0];
                                findIntersectionVertex(p1, p2, p3, p4, ref intersectPos, out t1);
                            }
                        }
                    }

                    for (ix = 0; ix < intersectPos.Count; ix++)
                    {
                        pointFs.Add(intersectPos[ix]);
                        visits.Add(false);
                    }
                }
            }

            findLeftBottomVertex(intersectPoints);

            Vector2 curPoint = new Vector2(startPoint.x, startPoint.y);
            Vector2 nextPoint = new Vector2(float.NaN, float.NaN);// = curIndex != intersectPoints[curPgon].Count - 1? intersectPoints[curPgon][curIndex+1]:intersectPoints[curPgon][0];
            float maxAngle = -1;
            float tempAngle = 0;
            int nextPgon = 0;
            Vector2 lastPoint = curPoint;
            int p1Index = 0;
            int p2Index = 0;

            bool firstPos = true;
            for (ix = 0; ix < 2000; ix++)
            {
                maxAngle = -1;
                for (pgon = 0; pgon < intersectPoints.Count; pgon++)
                {
                    for (cur_index = 0; cur_index < intersectPoints[pgon].Count; cur_index++)
                    {
                        if (Vector2.Distance(curPoint,intersectPoints[pgon][cur_index]) <= 0.01f)
                        {
                            p1Index = cur_index == 0 ? intersectPoints[pgon].Count - 1 : cur_index - 1;
                            p2Index = cur_index < intersectPoints[pgon].Count - 1 ? cur_index + 1 : 0;
                            p1 = intersectPoints[pgon][p1Index];
                            p2 = intersectPoints[pgon][p2Index];
                            if (curPgon != pgon && !intersectVisitPoints[pgon][p1Index] && !firstPos)
                            {
                                tempAngle = internalAngle(p1, lastPoint, curPoint);
                                if (tempAngle > maxAngle)
                                {
                                    maxAngle = tempAngle;
                                    nextPgon = pgon;
                                    nextPoint = p1;
                                }
                                intersectVisitPoints[pgon][p1Index] = true;
                            }

                            tempAngle = internalAngle(p2, lastPoint, curPoint);
                            if (tempAngle > maxAngle && !intersectVisitPoints[pgon][cur_index])
                            {
                                maxAngle = tempAngle;
                                nextPgon = pgon;
                                nextPoint = p2;
                                
                            }
                            intersectVisitPoints[pgon][cur_index] = true;
                        }
                    }
                }
                curPgon = nextPgon;
                lastPoint.Set(curPoint.x, curPoint.y);
                firstPos = false;
                if (float.IsNaN(nextPoint.x))
                {
                    return null;
                }
                else
                {
                    units.Add(curPoint);
                }
                curPoint = nextPoint;
                if (curPoint.Equals(startPoint))
                    break;
            }
            return units;
        }

        private Vector2 angleP1 = Vector2.zero;
        private Vector2 angleP2 = Vector2.zero;
        private float internalAngle(Vector2 p1, Vector2 p2, Vector2 startP)
        {
            float angle = 0;
            angleP1.Set(p1.x - startP.x, p1.y - startP.y);
            angleP2.Set(startP.x - p2.x, startP.y - p2.y);
            if (angleP2.x == 0 && angleP2.y == 0)
            {
                angleP2.x = 10;
                return Vector2.Angle(angleP1, angleP2);
            }
            angle = Vector2.Angle(angleP1, angleP2);
            Vector3 crossVec = Vector3.Cross(angleP1, angleP2);
            if (angle != 0 && crossVec.z < 0)
            {
                angle = 360 - angle;
            }
            else
            {
                angle = 180 - angle;
            }
            return angle;
        }

        public bool pnpoly(int nvert, Vector2[] verts, Vector2 test)
        {
            int i, j;
            double minX = verts[0].x;
            double maxX = verts[0].x;
            double minY = verts[0].y;
            double maxY = verts[0].y;
            for (i = 1; i < verts.Length; i++)
            {
                Vector2 q = verts[i];
                minX = Math.Min(q.x, minX);
                maxX = Math.Max(q.x, maxX);
                minY = Math.Min(q.y, minY);
                maxY = Math.Max(q.y, maxY);
            }

            if (test.x < minX || test.x > maxX || test.y < minY || test.y > maxY)
            {
                return false;
            }

            bool c = false;
            for (i = 0, j = nvert - 1; i < nvert; j = i++)
            {
                if (((verts[i].y > test.y) != (verts[j].y > test.y)) &&
                 (test.x < (verts[j].x - verts[i].x) * (test.y - verts[i].y) / (verts[j].y - verts[i].y) + verts[i].x))
                    c = !c;
            }
            return c;
        }
    }
}
