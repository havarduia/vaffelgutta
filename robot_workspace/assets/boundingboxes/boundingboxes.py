"""
Naming structure:
name = ([
    [ origin_x, origin_y, origin_z],        #origin
    [ offset_x, offset_y, offset_z]         #offset from origin
    [ distance_x, distance_y, distance_z]   #size of the rectangle
    ])

Creates a list of lists defining a rectangle with bottom-left-lower corner [(origin_x+offset_x), (origin_x+offset_x), (origin_x+offset_x)]
and extending [distance_x,distance_y,distance_z] units outwards from that point.
"""
floor = ([
    [-10,-10,-10],
    [0,0,0],
    [10.1,10.1,10.1]
])