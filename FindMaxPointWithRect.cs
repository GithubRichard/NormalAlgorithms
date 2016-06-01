using System.Collections.Generic;
using Tencent.Modules.TD.UnitAdapter;
using UnityEngine;

namespace Tencent.algorithms
{
        public class MaxPointInRect
        {
                public static Rect placeRectangle(List<Vector2> p, float width, float height)
                {
                    List<Vector2> optimal = null;
                    List<Vector2> points = p.GetRange(0, p.Count);
                    int max = 0;
                    points.Sort(horizontal);
                    int left = 0; 
                    int right = 0;
                    int top = 0; 
                    int bottom = 0;
                    for (left = 0, right = 0; left < points.Count; left++) {
                        while (right < points.Count && points[right].x <= points[left].x + width) ++right;
                        
                        List<Vector2> column = points.GetRange(left, right - left);
                        column.Sort(vertical);
                    
                        for (top = 0, bottom = 0; top < column.Count; top++) {
                            while (bottom < column.Count && column[bottom].y <= column[top].y + height) ++bottom;
                            if (bottom - top > max) {
                                max = bottom - top;
                                optimal = column.GetRange(top, bottom - top);
                            }
                            if (bottom == column.Count) break;
                        }
                        if (right == points.Count) break;
                    }
        
                    float leftF = 0f;
                    float rightF = 0f;
                    if (optimal == null) return new Rect(0, 0, 1, 1);
                    float topF = optimal[0].y; 
                    float bottomF = optimal[optimal.Count - 1].y;
                    for (var i = 0; i < optimal.Count; i++) {
                        var x = optimal[i].x;
                        if (leftF == 0 || x < leftF) leftF = x;
                        if (rightF == 0 || x > rightF) rightF = x;
                    }
                    return new Rect(leftF, topF, width, height);
                }
        
                public static int horizontal(Vector2 a, Vector2 b) 
                {
                    if (a.x == b.x)
                    {
                        return 0;
                    }
                    else if (a.x > b.x)
                    {
                        return 1;
                    }
                    return -1;
                }
        
                public static int vertical(Vector2 a, Vector2 b)
                {
                    if (a.y == b.y)
                    {
                        return 0;
                    }
                    else if (a.y > b.y)
                    {
                        return 1;
                    }
                    return -1;
                }
        }
}
