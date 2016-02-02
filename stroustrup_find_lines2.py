#!/usr/bin/env python

import numpy as np
import cv2
from math import hypot, pi, copysign
import copy

_ROLLER_Y_MIN = 160
_ROLLER_Y_MAX = 340#herke
_ROLLER_X_MIN = 200
_ROLLER_X_MAX = 440
_R_INDEX = 2
_G_INDEX = 1
_B_INDEX = 0
_Y_COLOR_THRESH = 0.25
_SHOW_DEBUG = False

_TEST_PATH='/tmp/images/'
#_TEST_PATH='/tmp/'
# _TEST_PATH = '/home/icub/tmp_massimo/tactile-control/gaussian-process-controller/'
#_TEST_PATH = '/u/thermans/sandbox/icub_vision_test/test2/'

_HOUGH_THRESH = 50
_MASK_THRESH = 0.7
_BLACK_LINE_THRESH = 200
_END_SEARCH_EDGE_BUFFER = 20
_CLOSE_BW_IMG = False
_LINE_INLIEAR_DIST = 5

_KULER_RED = (78./255, 18./255, 178./255)
_KULER_YELLOW = (25./255, 252./255, 1.)
_KULER_GREEN = (51./255., 178./255, 0.)
_KULER_BLUE = (204./255, 133./255, 20./255)
_KULER_RED1 = (0., 0., 1.)
_KULER_BLUE1 = (178./255, 113./255, 9./255)
_KULER_GREEN1 = (72./255, 255./255, 0.)

_MORPH_KERNEL = np.ones((3,3),np.uint8)
_MORPH_KERNEL[0,0] = 0
_MORPH_KERNEL[0,2] = 0
_MORPH_KERNEL[2,0] = 0
_MORPH_KERNEL[2,2] = 0

def test_on_imgs(t):
    results = np.zeros([10,100])
    prevtheta = 0
    for j in range(0,10):
        for i in range(0,100):
            try:
                path = _TEST_PATH+'test_'+str(j)+'_'+str(i)+'.tiff'
                if _SHOW_DEBUG:
                    print path
                img_i = cv2.imread(path)
                theta = run_system(img_i,t)
                if _SHOW_DEBUG:
                    cv2.waitKey(300)
                results[j,i]=theta
                print 'theta', theta
            except:
                pass  
            #    cv2.waitKey(100)  
    print 'results', np.concatenate([results,np.zeros([10,1])],1).T - np.concatenate([np.zeros([10,1]),results],1).T
    return np.concatenate([results,np.zeros([10,1])],1).T - np.concatenate([np.zeros([10,1]),results],1).T

def load_t_matrix():
    return np.loadtxt('tmatrix.txt')
    
def run_system(img_i,t ):
    img_t = crop_img(img_i)
    blob_i, blob_mask = find_roller(img_t)
    line_info = find_lines(img_t, blob_mask)
    lines_i = line_info[0]
    pts_img = line_info[1] 
    
    p0_img = np.asmatrix([pts_img[0][0] , pts_img[0][1] , 1]).H
    p0_world = t.dot(p0_img)
    p0_world = p0_world / p0_world[2]
    
    a0 = copy.copy(p0_world[0])
    a1 = copy.copy(p0_world[1]) #to avoid arctan messing with these
    theta_world = np.arctan2(a1, a0)

    if(_SHOW_DEBUG):
        world = np.zeros([100,100,3])
        p1_img = np.asmatrix([pts_img[1][0] , pts_img[1][1] , 1]).H
        p1_world = t.dot(p1_img)
        p1_world = p1_world / p1_world[2]

        p2_img = np.asmatrix([pts_img[2][0] , pts_img[2][1] , 1]).H
        p2_world = t.dot(p2_img)
        p2_world = p2_world / p2_world[2]

        p3_img = np.asmatrix([pts_img[3][0] , pts_img[3][1] , 1]).H
        p3_world = t.dot(p3_img)
        p3_world = p3_world / p3_world[2]
    
        #img_center = p3_world[1]
        cv2.line(world, (50,50), (50 + 10*p0_world[0],50 + 10*p0_world[1]), _KULER_RED, 2)
        cv2.line(world, (50,50), (50 + 10*p1_world[0],50 + 10*p1_world[1]), _KULER_BLUE, 2)
        cv2.line(world, (50,50), (50 + 10*p2_world[0],50 + 10*p2_world[1]), _KULER_GREEN, 2)
        cv2.line(world, (50,50), (50 + 10*p3_world[0],50 + 10*p3_world[1]), _KULER_YELLOW, 2)
        cv2.imshow('world',world)
        cv2.imshow('img_i', img_i)
        cv2.imshow('img_t', img_t)
        cv2.imshow('roller', blob_i)
        cv2.imshow('lines', lines_i)
        cv2.waitKey(3)
    return theta_world
    
def run_system_get_transform(img_i):
    img_t = crop_img(img_i)
    blob_i, blob_mask = find_roller(img_t)
    line_info = find_lines(img_t, blob_mask)   
    pts_img = line_info[1]
    if _SHOW_DEBUG:
        print 'pts imgs', pts_img
    t = estimate_transformation(pts_img)
    return t

def crop_img(img_in):
    # Ensure image is float 0 to 1
    if img_in.dtype == np.uint8:
        img_t = int_img2float(img_in)
    else:
        img_t = img_in[:]
    roll_img = img_t

    # Crop image
    img_t = img_t[_ROLLER_Y_MIN:_ROLLER_Y_MAX,
                  _ROLLER_X_MIN:_ROLLER_X_MAX,:]
    return img_t

def find_roller(img_t):
    # Herke -> select white pixels
    img_bw = cv2.cvtColor(img_t, cv2.COLOR_BGR2GRAY)
    mask_b = (img_bw > _MASK_THRESH).astype(np.uint8)

    mask_i = cv2.erode(mask_b, _MORPH_KERNEL,iterations = 2)
    mask_i = cv2.dilate(mask_i, _MORPH_KERNEL,iterations = 10)
    mask_i = cv2.erode(mask_i, _MORPH_KERNEL,iterations = 9)

    mask = mask_i.astype(np.bool)

    blob_img = np.zeros(img_t.shape, dtype=np.float32)
    blob_img[mask] = img_t[mask]

    return blob_img, mask

def find_lines(img_in, mask):
    img_bw = cv2.cvtColor(img_in, cv2.COLOR_BGR2GRAY)

    img_bw_u8 = (img_bw*255).astype(np.uint8)
    
    img_bw_u8[np.logical_not(mask)] = 255
    img_bw_u8 = cv2.GaussianBlur(img_bw_u8, (0,0), 2)
    threshold = _BLACK_LINE_THRESH
    img_bw_u8[img_bw_u8 > threshold] = 255
    img_bw_u8[img_bw_u8 <= threshold] = 0
    lines = cv2.HoughLines(255 - img_bw_u8, 3, np.pi/90., _HOUGH_THRESH)

    disp_img = np.zeros(img_in.shape, np.float32)
    disp_img[mask] = img_in[mask]

    idx = 0
    first_found = False
    second_found = False
    first_line = []
    first_line_polar = []
    second_line = []
    second_line_polar = []
    theta_first = 0
    if lines is not None:
        if _SHOW_DEBUG:
            print 'Num hough lines found',len(lines[0])
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 500*(-b))
            y1 = int(y0 + 500*(a))
            x2 = int(x0 - 500*(-b))
            y2 = int(y0 - 500*(a))
            g = 1 - idx * 0.04

            idx = idx + 1
            if not(first_found):
              first_found = True
              cv2.line(disp_img, (x1,y1), (x2,y2), _KULER_RED, 1)
              theta_first = theta
              first_line = [(x1,y1), (x2,y2)]
              first_line_polar = [rho, theta]
              continue
            dif = theta - theta_first
            if(dif > 0.5 * pi):
              dif = dif - pi
            if(dif < -0.5 * pi):
              dif = dif + pi
            if not(second_found) and abs(dif) > 0.25 * pi:
              second_found = True
              cv2.line(disp_img, (x1,y1), (x2,y2), _KULER_YELLOW, 1)
              second_line = [(x1,y1), (x2,y2)]
              second_line_polar = [rho, theta]
              continue

    if _CLOSE_BW_IMG:
        img_bw_u8 = cv2.dilate(img_bw_u8, _MORPH_KERNEL, iterations = 1)
        img_bw_u8 = cv2.erode(img_bw_u8, _MORPH_KERNEL, iterations = 2)

    if(_SHOW_DEBUG):
        cv2.imshow('bw', img_bw_u8)
        cv2.imshow("mask", mask.astype(np.uint8)*255)
        cv2.imshow("disp", disp_img)
        cv2.waitKey(3)
    
    if first_found and second_found:
        c_pt_img = line_line_intersection(first_line, second_line)
        if _SHOW_DEBUG:
            print c_pt_img
            cv2.circle(disp_img, c_pt_img, 5, _KULER_BLUE,2)
        # Find end points and perform data association to model
        (p1_pt_img, p2_pt_img, p3_pt_img) = get_end_pts(first_line, second_line, c_pt_img, img_bw_u8)

    
    # cv2.imshow('edges_full', edge_img)
    # cv2.imshow('edges', edge_img_m)
    
    return (disp_img, [p1_pt_img, p2_pt_img, p3_pt_img, c_pt_img])

def get_end_pts(first_line, second_line, c_pt_img, img_bw):
    '''
    first_line - endpoints defining first line (not segment)
    second_line - endpoints defining second line (not segment)
    c_pt_img - location of center point in image coordinates
    img_bw - bw image with lines in black
    '''
    fl_pts = line_inliers(first_line, img_bw, _LINE_INLIEAR_DIST)
    sl_pts = line_inliers(second_line, img_bw, _LINE_INLIEAR_DIST)

    # print 'len(fl_pts)',len(fl_pts)
    # print 'len(sl_pts)',len(sl_pts)

    # Find the extrema for both lines
    fl_ends = get_line_end_points(fl_pts)
    sl_ends = get_line_end_points(sl_pts)


    
    # Figure out which of the four points is closest to the center,
    # we use this as the end point of the minor axis against the center
    min_fl_c_dist = 1000
    min_fl_c_idx = len(sl_ends)
    for i, p in enumerate(fl_ends):
        d = pt_pt_dist(p, c_pt_img)
        if d < min_fl_c_dist:
            min_fl_c_dist = d
            min_fl_c_idx = i

    min_sl_c_dist = 1000
    min_sl_c_idx = len(sl_ends)
    for i, p in enumerate(sl_ends):
        d = pt_pt_dist(p, c_pt_img)
        if d < min_sl_c_dist:
            min_sl_c_dist = d
            min_sl_c_idx = i
    if min_fl_c_dist < min_sl_c_dist:
        if _SHOW_DEBUG:
            print 'Swapping based on center point!'
        pts_long = sl_ends
        all_long = sl_pts
        if min_fl_c_idx == 1:
            pt_short = fl_ends[0]
        else:
            pt_short = fl_ends[1]
    else:
        pts_long = fl_ends
        all_long = fl_pts
        if min_sl_c_idx == 1:
            pt_short = sl_ends[0]
        else:
            pt_short = sl_ends[1]

    # Associate the points with the correct model locations
    pts = data_association(pts_long, pt_short)

    if(_SHOW_DEBUG):
        # Display stuff
        disp_img = cv2.cvtColor(img_bw, cv2.COLOR_GRAY2BGR)
        for p in fl_pts:
            disp_img[p[1],p[0],:] = np.array(_KULER_RED)*255
        for p in sl_pts:
            disp_img[p[1],p[0],:] = np.array(_KULER_YELLOW)*255

        # Display points
        cv2.circle(disp_img, pts[0], 3, np.array(_KULER_RED1)*255, 2)
        cv2.circle(disp_img, pts[1], 3, np.array(_KULER_GREEN)*255, 2)
        cv2.circle(disp_img, pts[2], 3, np.array(_KULER_BLUE)*255, 2)
    print 'showing three points'
        

    
    #print 'fl', fl_pts    
    #print 'fl shape', np.asmatrix(fl_pts).shape
    c = np.cov( np.asmatrix(all_long).T)
    [w,v] = np.linalg.eig(c)
    maxidx = np.argmax(w)
    line = v[:,maxidx]
    mu = np.mean(np.asmatrix(all_long).T)
    
    
    old_vector = np.asarray(pts[0]) -   np.asmatrix(c_pt_img  )
    #print 'line', line.shape, 'old_vector', old_vector.shape
    #dotp = (old_vector).dot(line)
    dotp = np.dot(old_vector, line)
    if(dotp < 0):
        line = - line
    #length = np.sqrt(old_vector.dot(old_vector))
    length = np.sqrt(np.dot(old_vector, old_vector.T))
    #print 'length', length
    line = line * float(length)
    newpoint = c_pt_img + line #TODO was mu + line
    pts[0] = (float(newpoint[0]), float(newpoint[1]) )
    print 'showing extra point'
    if(_SHOW_DEBUG):
        cv2.circle(disp_img, (int(pts[0][0]), int(pts[0][1])), 3, (0.,0.,0.), 2)
        cv2.imshow('inliers', disp_img)
        cv2.waitKey()
    return pts

def get_line_end_points(line_pts):
    min_y_idx = len(line_pts)
    max_y_idx = len(line_pts)
    min_x_idx = len(line_pts)
    max_x_idx = len(line_pts)
    min_y_pt = []
    max_y_pt = []
    min_x_pt = []
    max_x_pt = []

    min_y = 10000
    max_y = -1
    min_x = 10000
    max_x = -1

    # Find all extrema in space
    for i, p in enumerate(line_pts):
        if p[0] < min_x:
            min_x = p[0]
            min_x_idx = i
            min_x_pt = p
        if p[0] > max_x:
            max_x = p[0]
            max_x_idx = 1
            max_x_pt = p
        if p[1] < min_y:
            min_y = p[1]
            min_y_idx = i
            min_y_pt = p
        if p[1] > max_y:
            max_y = p[1]
            max_y_idx = i
            max_y_pt = p

    # Find longest dist of the 6 possible point pairs (we compare more then necessary here)
    extrema_pts = [min_x_pt, max_x_pt, min_y_pt, max_y_pt]
    end_pts = []
    all_pts = []
    max_dist = 0
    for p1 in extrema_pts:
        for p2 in extrema_pts:
            d = pt_pt_dist(p1, p2)
            if d  > max_dist:
                max_dist = d
                end_pts = [p1, p2]

    return end_pts

def line_inliers(line, img_bw, inlier_dist):
    y_start = _END_SEARCH_EDGE_BUFFER
    y_end = img_bw.shape[0] - _END_SEARCH_EDGE_BUFFER
    x_start =  _END_SEARCH_EDGE_BUFFER
    x_end = img_bw.shape[1] - _END_SEARCH_EDGE_BUFFER

    line_pts = []
    for y in range(y_start, y_end):
        for x in range(x_start, x_end):
            if img_bw[y,x] != 0:
                continue
            elif point_line_dist((x,y), line) < inlier_dist:
                line_pts.append((x,y))
    return line_pts

def point_line_dist(pt, line):
    X1 = np.array([line[0][0], line[0][1]],dtype=np.float32)
    X2 = np.array([line[1][0], line[1][1]],dtype=np.float32)
    Q  = np.array([pt[0],  pt[1]],dtype=np.float32)

    #if X1[0] == X2[0]:
    #    return abs(X1[1]-Q[1]) 
    # doesn't work with vertical lines - do we need it?

    v = X2 - X1
    w = Q - X1

    c1 = np.dot(w,v)
    c2 = np.dot(v,v)
    b = c1/c2

    P = X1 + b*v
    d = hypot(P[0] - Q[0], P[1] - Q[1])
    return d

def pt_pt_dist(a,b):
    return hypot(a[0] - b[0], a[1] - b[1])

def line_line_intersection(a, b):
    a1_x = a[0][0]
    a1_y = a[0][1]
    a2_x = a[1][0]
    a2_y = a[1][1]
    b1_x = b[0][0]
    b1_y = b[0][1]
    b2_x = b[1][0]
    b2_y = b[1][1]
    denom = (a1_x - a2_x)*(b1_y - b2_y) - (a1_y - a2_y)*(b1_x - b2_x)
    if denom == 0: # Parallel lines
        return None
    pt_x = ((a1_x*a2_y - a1_y*a2_x)*(b1_x-b2_x) -
                    (a1_x - a2_x)*(b1_x*b2_y - b1_y*b2_x))/denom
    pt_y = ((a1_x*a2_y - a1_y*a2_x)*(b1_y-b2_y) -
                    (a1_y - a2_y)*(b1_x*b2_y - b1_y*b2_x))/denom
    pt = (pt_x,pt_y)
    return pt

def data_association(pts_long, pt_short):
    # Determine side of line the short point is on
    line_d = ((pts_long[1][0] - pts_long[0][0])*(pt_short[1]-pts_long[0][1]) -
              (pts_long[1][1] - pts_long[0][1])*(pt_short[0]-pts_long[0][0]))
    
    side = copysign(1, line_d)
    if(_SHOW_DEBUG):
        print 'line_d', line_d
        print 'line_side', side
    # Negative sign implies start point is at top when short facing right
    if side < 0:
        pts_all = [pts_long[0], pt_short, pts_long[1]]
    else:
        pts_all = [pts_long[1], pt_short, pts_long[0]]

    return pts_all

def estimate_transformation(pts_img):
    p0_world = np.matrix([0.,4.5])
    p1_world = np.matrix([4.5,4.5])
    p2_world = np.matrix([4.5,0.])
    p3_world = np.matrix([4.5,-4.5])
    p4_world = np.matrix([0.,-4.5])
    p5_world = np.matrix([-4.5,-4.5])
    #c_pt_world = np.matrix([0.,0.])
    p6_world = np.matrix([-4.5,0.])
    
    p0_img =   np.matrix(pts_img[0])
    p1_img =   np.matrix(pts_img[1])
    p2_img =   np.matrix(pts_img[2])
    #c_pt_img = np.matrix(pts_img[3])
    p3_img = np.matrix(pts_img[3])
    p4_img = np.matrix(pts_img[4])
    p5_img = np.matrix(pts_img[5])
    p6_img = np.matrix(pts_img[6])
    #p3_img = p1_img + p1_img - c_pt_img
    
    ax = []
    ay = []
    
    world = [p0_world.T,p1_world.T,p2_world.T,p3_world.T,p4_world.T,p5_world.T,p6_world.T ]
    img =   [p0_img.T  ,p1_img.T  ,p2_img.T  ,p3_img.T  ,p4_img.T  ,p5_img.T  ,p6_img.T]
    
    for i in range(len(img)):
        wi = np.asarray(world[i])
        ii = np.asarray(img[i])
        #print wi
        #print ii
        ax = ax + [[-ii[0], -ii[1], -1,      0,      0,  0, ii[0]*wi[0], ii[1]*wi[0], wi[0]]]
        ay = ay + [[0     ,      0,  0, -ii[0], -ii[1], -1, ii[0]*wi[1], ii[1]*wi[1], wi[1]]]
    A = np.concatenate([ax, ay])
    #print A
    #print ax[0]
    #print type(ax[0])
    #print A
    #print A.shape
    
    U, s, V = np.linalg.svd(A)
    
    
    #print 'V', V
    
    #print 'Vshape', V.shape, 'Ushape', U.shape
  
    # t is smallest right singular vector
    #t = V[:,8]
    t = V[8,:]
    
    #print 't', t
    #print 's', s
    T = np.reshape(t, [3,3]) #np.zeros((3,3))
    mat_in = np.concatenate([np.concatenate(img,1), [[1,1,1,1,1,1,1]]])
    #print mat_in
    res = T.dot(mat_in)
    #print 'r0', res
    res = res / np.concatenate([res[2],res[2],res[2]])
    #print 'result', res.T
    return T

def transform_pts(pts_a, pts_b, transform):
    pass

def get_state(pts):
    pass

def get_reward(state):
    pass

def int_img2float(img_in):
    img_out = img_in.astype(np.float32)
    img_out *= 1./255
    return img_out

def get_y_img(img_in):
    y_abs_img = np.abs(img_in[:,:,_R_INDEX]*0.5 - img_in[:,:,_G_INDEX]*0.5)
    y_img = (img_in[:,:,_R_INDEX]*0.5 + img_in[:,:,_G_INDEX]*0.5) - y_abs_img - img_in[:,:,_B_INDEX]
    return y_img
