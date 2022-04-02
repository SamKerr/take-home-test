
from typing import Tuple, List
import re
import sys
from dataclasses import dataclass
import numpy as np
from enum import Enum
from geometry import do_intersect

@dataclass
class BoundingBox:
    min_x: int
    max_x: int
    min_y: int
    max_y: int

class LabelOrientation(Enum):
    BOTTOM_LEFT = 'bottom-left'
    BOTTOM_RIGHT = 'bottom-right'
    TOP_LEFT = 'top-left'
    TOP_RIGHT = 'top-right'


class Point:
    def __init__(self, coords: Tuple[int,int]) -> None:
        self.vector = np.array(list(coords))
        self.coords = coords
        self.x = coords[0]
        self.y = coords[1]
        self.dist_to_center  = None
        self.is_convex = None
        self.previous_point = None
        self.next_point = None

    def add_prev_and_next(self, prev, next):
        self.previous_point = prev 
        self.next_point = next 

    def find_label_orientation(self) -> LabelOrientation:
        """
            Uses a very simple heuristic to find which of the 4 label types to use


            /\ -> Top Right
            \/ -> Bottom Right
            > -> Top Left
            < -> Bottom Left
        """
        if(self.previous_point is not None and self.next_point is not None):
            prev_p = self.previous_point
            next_p = self.next_point
            if(self.y > prev_p.y and self.y > next_p.y):
                return LabelOrientation.TOP_RIGHT
            elif(self.y < prev_p.y and self.y < next_p.y):
                return LabelOrientation.BOTTOM_RIGHT
            elif(self.x > prev_p.x and self.x > next_p.x):
                return LabelOrientation.TOP_LEFT
            else:
                return LabelOrientation.BOTTOM_LEFT


PolyLine = List[Point]
class PolyLineLabelPlacement:
    def __init__(self, poly_lines: List[PolyLine]) -> None:
        self.poly_lines = poly_lines
        self.bounding_box = None 
        self.center  = None

    def _find_bounding_box_and_center(self):
        """
            Finds the bounding box for all polylines
            Also calculates and sets the most central point
        """
        min_x = self.poly_lines[0][0].x
        max_x = 0
        min_y = self.poly_lines[0][0].y
        max_y = 0
        
        for line in self.poly_lines:
            for point in line:
                x,y = point.coords
                min_x=min(min_x, x)
                max_x=max(max_x, x)
                min_y=min(min_y, y)
                max_y=max(max_y, y)
        
        assert min_x>=0 and min_y>=0, 'Invalid input - all x,y coordinates must be >= 0'

        self.bounding_box = BoundingBox(min_x, max_x, min_y, max_y)
        
        # Rounded the center coordinates to integer values as a Point takes a tuple of integers in its constructor
        self.center = Point(((min_x+max_x)//2, (min_y+max_y)//2))

    def _euclidean_distance(self, p1: Point, p2: Point) -> float:
        """
            Calculates the euclidean between two points
                - https://en.wikipedia.org/wiki/Euclidean_distance#:~:text=In%20mathematics%2C%20the%20Euclidean%20distance,being%20called%20the%20Pythagorean%20distance.
        """
        return np.linalg.norm(p1.vector-p2.vector)

    def _calc_distance_to_center_for_points(self) -> None:
        """
            This calculates the distance from the center each point is
            Distance to the center is 1 measurement of how aestetic a solution is
        """
        for line in self.poly_lines:
            for point in line:
                point.dist_to_center = self._euclidean_distance(point, self.center)

    def _generate_label(self, point: Point) -> str:
        """
            Returns a string with expected format of a label

            Limitations:
                Does not take into account intersections with the same line or other lines
            
            Extension if given more time: 
                I would write an algorithm that checks for intersections with the same line just by looking N (eg N=10) hops forward
                and backward from this point (using self.previous_point and self.next_point) and seeing if any of those line segments 
                intersect with the 4 line segments of the label.

                This would actually be O(1) for fixed N, and N ~ 10 is reasonable to me.

                To handle the case of intersection with other lines it becomes O(N), but that is still fine really.

                We can try all 4 orientations and if all of them cause an intersection we can continue to try points further from the center.
        """
        return f'{point.x} {point.y} {point.find_label_orientation().value}'

    def  _generate_unique_line_segments(self, grouping_factor: int) -> List[PolyLine]:
        """
            Returns a list of polylines such that:
                for each line:
                    for each line segment in that line:
                        that line segment does not intersect with any other line segment returned
            
            This algorithm is O(n^2) so I have included a grouping factor argument to allow it to run on faster inputs

            Arguments:
                grouping_factor: an integer that can be changed to allow for faster processing by downsampling the input
        """

        poly_lines_with_grouped_points = np.array(self.poly_lines)
        for i in range(len(poly_lines_with_grouped_points)):
            poly_lines_with_grouped_points[i] = poly_lines_with_grouped_points[i][::grouping_factor]

        # we need to update the next and previous pointers again as we have downsampled
        for line in poly_lines_with_grouped_points:
            for i, point in enumerate(line):
                if i==0:
                    point.add_prev_and_next(None, line[i+1])
                elif i==len(line)-1:
                    point.add_prev_and_next(line[i-1], None)
                else:
                    point.add_prev_and_next(line[i-1], line[i+1])


        # as we cannot afford to remove each point individually (O(n) operation) we store all points to be removed in a set 
        # then we rebuild the unique line segments
        # this allows the whole algorithm to run in just O(n^2)
        points_to_remove = set()
        for i,line in enumerate(poly_lines_with_grouped_points):
            for point in line:
                if(point.next_point is not None):
                    line_segment = (point, point.next_point)
                    for j, line_2 in enumerate(poly_lines_with_grouped_points):
                        if(i==j):
                            # we do not care if a segment on a line intersects with a segment on the same line
                            break
                        else:
                            for point_2 in line_2:
                                if(point_2.next_point is not None):
                                    line_segment_2 = (point_2, point_2.next_point)
                                    if(do_intersect(line_segment, line_segment_2)):
                                        points_to_remove.add(point)
                                        points_to_remove.add(point_2)

        # rebuild the polylines and only include the non-intersecting points
        unique_poly_lines = []
        for line in poly_lines_with_grouped_points:
            unique_line = []
            for point in line:
                if point not in points_to_remove:
                    unique_line.append(point)
            unique_poly_lines.append(unique_line)
        
        return unique_poly_lines
                

    def greedy_strategy(self) -> List[str]:
        """
            Implements a greedy algorithm for this problem.

            Outline:
                Try each of the points in order of distance to the center
                Take the first valid point you find for each line
                If no valid point exists on a line, just take the point closest to the center
        """
        output = []
        
        # using a grouping factor of 3 since it gives a trade off between speed and accuracy
        grouping_factor = 3
        unique_poly_line_segments = self._generate_unique_line_segments(grouping_factor)
        
        for line in unique_poly_line_segments:
            points_closest_to_center = sorted(line, key=lambda x: x.dist_to_center)
            label_found = False
            for point in points_closest_to_center:
                potential_label = self._generate_label(point)
                if(potential_label is not None):
                    output.append(potential_label)
                    label_found = True
                    break

            # if no valid label can be found, just take the first point as the label
            if(not label_found):
                output.append(self._generate_invalid_label(points_closest_to_center[0]))
        
        return output


    def solve(self):
        """
            Main solver method
        """
        self._find_bounding_box_and_center()
        self._calc_distance_to_center_for_points()
        return self.greedy_strategy()


def parse_input() -> List[PolyLine]:
    """
        Parses space seperated integers from stdin and returns a List of PolyLine's 
    """
    poly_lines = []
    for line in sys.stdin:
        # use regex to grab all the numerical-valued continuous substrings
        numbers = re.findall(r'\d+', line)

        # Convert each X Y pair if integers into an instance of Point 
        poly_line = [Point(tuple(map(int, [numbers[i], numbers[i + 1]]))) for i in range(0, len(numbers) - 1, 2)]
        
        assert len(poly_line)>1, 'Invalid input - all lines must have more than 1 point'
        
        # Populate the previous_point and next_point fields for each Point so that each point has some knowledge of its surrounding points
        for i, point in enumerate(poly_line):
            if i==0:
                point.add_prev_and_next(None, poly_line[i+1])
            elif i==len(poly_line)-1:
                point.add_prev_and_next(poly_line[i-1], None)
            else:
                point.add_prev_and_next(poly_line[i-1], poly_line[i+1])

        poly_lines.append(poly_line)
    
    return poly_lines

def main():
    poly_lines = parse_input()
    solver = PolyLineLabelPlacement(poly_lines)
    solution = solver.solve()
    
    for line in solution:
        print(line)

    with open('output/answer.txt', 'w') as f:
        for line in solution:
            f.write(line)
            f.write('\n')

if __name__ == '__main__':
    main()