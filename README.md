# Take Home Test

Work completed:

    - Program written that takes the expected input through stdin and prints result to stdout, as well as generating the text file output/answer.txt
    - Geometry algorithms implemented to find if two line segments intersect and to calculate the euclidean distance between two points
    - Basic greedy approach implemented:
      - Lines pruned to remove any segments that intersect with other segments. This is O(N^2) however a downsampling paramater was also included.
      - Line segement vertexes tried in order of distance from the center
        - When trying to generate a label for a vertex, a very quick and rough heuristic is used with just looks at the two neighbouring points of a vertex


Extensions if given more time:

    - 1. Extend the label generator to atleast consider intersections along the same line. 
    - 2. Further extend the label orientation to handle intersections with other lines. This could be done very similarly to how the unique line segments are generated. Essentially try all line segements and see if there is ever an intersection between them and the smallest zoom. 
      - Take the first label that does not intersect any lines at smallest zoom
    - 3. Extend the label orientation to handle zoom levels:
      - run extension-2 implementation 5 times, with a zoom_level parameter, starting at the highest zoom.
      - When a label is allocated make sure to include those line segments in the detection of future label intersections 
      - This would be O(N) for a given point and could in theory occur for O(N) points. However, the bottleneck is still finding the unique line segments which is O(N^2).
