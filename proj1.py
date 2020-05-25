import racetrack,math
import sys

infinity = float('inf')     # alternatively, we could import math.inf

g_fline = False
g_walls = False
best_pt = (-1, -1)
best_dis = infinity
grid = []
ref_pts = []

# compute edistw_to_finish from corners and midpoints of corners, then fill out the rest of the values based on that
def proj1(state, fline, walls):
    global g_fline, g_walls, best_pt, best_dis, ref_pts

    ((x,y),(u,v)) = state

    #if we have a direct path to the finish, just return the distance, no need for extra computations
    d = edistw_to_finish((x,y), fline, walls)
    if d != infinity:
        hval = float(d)
    # there is an obstacle in between the state and finish line
    else:
        if fline != g_fline or walls != g_walls or ref_pts == []:
            references(fline, walls)
        
        hval = infinity
        for (d1, x1, y1) in ref_pts:
            hval = min(hval, d1 + dist((x1,y1), (x,y), walls))

        ref_pts.append((hval, x, y))


    # add a small penalty to favor short stopping distances
    au = abs(u); av = abs(v); 
    sdu = au*(au-1)/2.0
    sdv = av*(av-1)/2.0
    sd = max(sdu,sdv)
    penalty = sd/10.0

    # compute location after fastest stop, and add a penalty if it goes through a wall
    if u < 0: sdu = -sdu
    if v < 0: sdv = -sdv
    sx = x + sdu
    sy = y + sdv
    if racetrack.crash([(x,y),(sx,sy)],walls):
        penalty += math.sqrt(au**2 + av**2)
    hval = max(hval+penalty,sd)
    return hval

# store all the distances between reference points and finish line
def references(fline, walls):
    global ref_pts, xmax, ymax, g_fline, g_walls
    xmax = max([max(x,x1) for ((x,y),(x1,y1)) in walls])
    ymax = max([max(y,y1) for ((x,y),(x1,y1)) in walls])

    important_x = [1, int(xmax-1), int(xmax/2)]
    important_y = [1, int(ymax-1), int(ymax/2)]

    best_dis = infinity
    for x in important_x:
        for y in important_y:
            dis = edistw_to_finish((x,y), fline, walls)
            ref_pts.append((dis, x, y))
            best_dis = min(dis, best_dis)

    xmin = 0
    ymin = 0
    midways = []

    # iterate through each wall, for every wall, cache points midway between where wall ends and end of course
    for ((x1, y1), (x2, y2)) in walls:
        if x1 == x2:    # wall is vertical
            my1 = (max(y1, y2) + ymax)/2    # midway between top of course and top of wall
            if my1 != 0 and my1 != ymax:
                d1 = edistw_to_finish((x1,my1), fline, walls)

                # direct path from this midway point, just save that
                if d1 != infinity:
                    ref_pts.append((d1, x1, my1))
                # otherwise save it for later
                else:
                    midways.append((x1, my1))

            my2 = (min(y1, y2) + ymin)/2    # midway between bottom of course and bottom of wall
            if my2 != 0 and my2 != ymin:
                d2 = edistw_to_finish((x1,my2), fline, walls)

                if d2 != infinity:
                    ref_pts.append((d2, x1, my2))
                else:
                    midways.append((x1, my2))
            # print ((d1, x1, my1))
            # print ((d2, x1, my2))

        else:   # wall is horizontal
            mx1 = (max(x1, x2) + xmax)/2 - 1
            if mx1 != 0 and mx1 != xmax:
                d1 = edistw_to_finish((mx1,y1), fline, walls)

                if d1 != infinity:
                    ref_pts.append((d1, mx1, y1))
                else:
                    midways.append((mx1, y1))

            mx2 = (min(x1, x2) + xmin)/2
            if mx2 != 0 and mx2 != xmin:
                d2 = edistw_to_finish((mx2,y1), fline, walls)

                if d2 != infinity:
                    ref_pts.append((d2, mx2, y1))
                else:
                    midways.append((mx2, y1))
            # print((d1, mx1, y1))
            # print((d1, mx1, y1))
    
    again = 1

    while again:
        again = 0
        check_again = []

        # for every point in midways
        for (midx, midy) in midways:
            # print( (midx, midy))
            est = infinity
            # go through the reference points and see if theres an estimate we can use for this midway
            for (d, x, y) in ref_pts:
                est = min(est, d + dist((midx, midy), (x,y), walls))
            # if yes, add this to reference points, and signal that we should check again
            if est != infinity:
                ref_pts.append((est, midx, midy))
                again = 1
            # if no, add to list of points to check again (maybe)
            else:
                check_again.append((midx, midy))
        midways = check_again.copy()


    # surround each point adjacent to finish line with h-val of 1 (idk if I need this)

    # ((x1,y1),(x2,y2)) = fline
    # if x1 == x2:           # fline is vertical, so iterate over y
    #     for y in range(min(y1, y2), max(y1, y2)):
    #         ref_pts.append((1, x1-1, y))
    #         ref_pts.append((1, x1+1, y))
    # else:                  # fline is horizontal, so iterate over x
    #     for x in range(min(x1, x2), max(x1, x2)):
    #         ref_pts.append((1, x, y1-1))
    #         ref_pts.append((1, x, y1+1))

    # print (ref_pts)
    g_fline = fline
    g_walls = walls

# return distance between two points, infinity if there is a crash
def dist(pt1, pt2, walls):
    if not racetrack.crash((pt1, pt2), walls):
        (x1,y1) = pt1
        (x2, y2) = pt2
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    else:
        return infinity

def h_walldist(state, fline, walls):
    """
    The first time this function is called, for each gridpoint that's not inside a wall
    it will cache a rough estimate of the length of the shortest path to the finish line.
    The computation is done by a breadth-first search going backwards from the finish 
    line, one gridpoint at a time.
    
    On all subsequent calls, this function will retrieve the cached value and add an
    estimate of how long it will take to stop. 
    """
    global g_fline, g_walls
    if fline != g_fline or walls != g_walls or grid == []:
        edist_grid(fline, walls)
    ((x,y),(u,v)) = state
    hval = float(grid[x][y])
    
    # add a small penalty to favor short stopping distances
    au = abs(u); av = abs(v); 
    sdu = au*(au-1)/2.0
    sdv = av*(av-1)/2.0
    sd = max(sdu,sdv)
    penalty = sd/10.0

    # compute location after fastest stop, and add a penalty if it goes through a wall
    if u < 0: sdu = -sdu
    if v < 0: sdv = -sdv
    sx = x + sdu
    sy = y + sdv
    if racetrack.crash([(x,y),(sx,sy)],walls):
        penalty += math.sqrt(au**2 + av**2)
    hval = max(hval+penalty,sd)
    return hval


# populates some sort of grid/matrix with distances in
def edist_grid(fline,walls):
    global grid, g_fline, g_walls, xmax, ymax
    xmax = max([max(x,x1) for ((x,y),(x1,y1)) in walls])
    ymax = max([max(y,y1) for ((x,y),(x1,y1)) in walls])
    grid = [[edistw_to_finish((x,y), fline, walls) for y in range(ymax+1)] for x in range(xmax+1)]
    flag = True
    print('computing edist grid', end=' '); sys.stdout.flush()
    while flag:
        print('.', end=''); sys.stdout.flush()
        flag = False
        for x in range(xmax+1):
            for y in range(ymax+1):
                for y1 in range(max(0,y-1),min(ymax+1,y+2)):
                    for x1 in range(max(0,x-1),min(xmax+1,x+2)):
                        if grid[x1][y1] != infinity and not racetrack.crash(((x,y),(x1,y1)),walls):
                            if x == x1 or y == y1:
                                d = grid[x1][y1] + 1
                            else:
                                # In principle, it seems like a taxicab metric should be just as
                                # good, but Euclidean seems to work a little better in my tests.
                                d = grid[x1][y1] + 1.4142135623730951
                            if d < grid[x][y]:
                                grid[x][y] = d
                                flag = True
    print(' done')
    g_fline = fline
    g_walls = walls
    return grid


def edistw_to_finish(point, fline, walls):
    """
    straight-line distance from (x,y) to the finish line ((x1,y1),(x2,y2)).
    Return infinity if there's no way to do it without intersecting a wall
    """
#   if min(x1,x2) <= x <= max(x1,x2) and  min(y1,y2) <= y <= max(y1,y2):
#       return 0
    (x,y) = point
    ((x1,y1),(x2,y2)) = fline
    # make a list of distances to each reachable point in fline
    if x1 == x2:           # fline is vertical, so iterate over y
        ds = [math.sqrt((x1-x)**2 + (y3-y)**2) \
            for y3 in range(min(y1,y2),max(y1,y2)+1) \
            if not racetrack.crash(((x,y),(x1,y3)), walls)]
    else:                  # fline is horizontal, so iterate over x
        ds = [math.sqrt((x3-x)**2 + (y1-y)**2) \
            for x3 in range(min(x1,x2),max(x1,x2)+1) \
            if not racetrack.crash(((x,y),(x3,y1)), walls)]
    ds.append(infinity)    # for the case where ds is empty
    return min(ds)

def h_edist(state, fline, walls):
    """Euclidean distance from state to fline, ignoring walls."""
    (x,y) = state[0]
    ((x1,y1),(x2,y2)) = fline
    
    # find the smallest and largest coordinates
    xmin = min(x1,x2); xmax = max(x1,x2)
    ymin = min(y1,y2); ymax = max(y1,y2)

    return min([math.sqrt((xx-x)**2 + (yy-y)**2)
        for xx in range(xmin,xmax+1) for yy in range(ymin,ymax+1)])

# compute edistw_to_finish from corners and midpoints of corners, then fill out the rest of the values based on that
# def proj1_grid(fline, walls):
#     global grid, g_fline, g_walls, xmax, ymax, best_pt, best_dis, ref_pts

#     xmax = max([max(x,x1) for ((x,y),(x1,y1)) in walls])
#     ymax = max([max(y,y1) for ((x,y),(x1,y1)) in walls])
#     grid = [[infinity for y in range(ymax+1)] for x in range(xmax+1)]

#     important_x = [1, int(xmax-1), int(xmax/2)]
#     important_y = [1, int(ymax-1), int(ymax/2)]
    
#     for x in important_x:
#         for y in important_y:
#             dis = edistw_to_finish((x,y), fline, walls)
#             grid[x][y] = dis
#             if dis < best_dis:
#                 best_dis = dis
#                 best_pt = (x,y)

#     print(best_pt, best_dis)
    # need something to handle if all the important_pts are inf, maybe just x+/-1, and y+/-1
    # if min = infinity:

    
    

    # g_fline = fline
    # g_walls = walls

