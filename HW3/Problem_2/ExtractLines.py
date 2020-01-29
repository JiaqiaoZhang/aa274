#!/usr/bin/env python

############################################################
# ExtractLines.py
#
# This script reads in range data from a csv file, and
# implements a split-and-merge to extract meaningful lines
# in the environment.
############################################################

# Imports
import numpy as np
from PlotFunctions import *


############################################################
# functions
############################################################

def ExtractLines(RangeData, params):
    '''
    This function implements a split-and-merge line extraction algorithm.

    Inputs:
        RangeData: (x_r, y_r, theta, rho)
            x_r: robot's x position (m).
            y_r: robot's y position (m).
            theta: (1D) np array of angle 'theta' from data (rads).
            rho: (1D) np array of distance 'rho' from data (m).
        params: dictionary of parameters for line extraction.
    Outputs:
        alpha: (1D) np array of 'alpha' for each fitted line (rads).
        r: (1D) np array of 'r' for each fitted line (m).
        segend: np array (N_lines, 4) of line segment endpoints. Each row represents [x1, y1, x2, y2].
        pointIdx: (N_lines,2) segment's first and last point index.
    '''

    # Extract useful variables from RangeData
    x_r = RangeData[0]
    y_r = RangeData[1]
    theta = RangeData[2]
    rho = RangeData[3]

    ### Split Lines ###
    N_pts = len(rho)
    r = np.zeros(0)
    alpha = np.zeros(0)
    pointIdx = np.zeros((0, 2), dtype=np.int)

    # This implementation pre-prepartitions the data according to the "MAX_P2P_DIST"
    # parameter. It forces line segmentation at sufficiently large range jumps.
    rho_diff = np.abs(rho[1:] - rho[:(len(rho)-1)])
    LineBreak = np.hstack((np.where(rho_diff > params['MAX_P2P_DIST'])[0]+1, N_pts))
    startIdx = 0
    for endIdx in LineBreak:
        alpha_seg, r_seg, pointIdx_seg = SplitLinesRecursive(theta, rho, startIdx, endIdx, params)
        N_lines = r_seg.size

        ### Merge Lines ###
        if (N_lines > 1):
            alpha_seg, r_seg, pointIdx_seg = MergeColinearNeigbors(theta, rho, alpha_seg, r_seg, pointIdx_seg, params)
        r = np.append(r, r_seg)
        alpha = np.append(alpha, alpha_seg)
        pointIdx = np.vstack((pointIdx, pointIdx_seg))
        startIdx = endIdx

    N_lines = alpha.size

    ### Compute endpoints/lengths of the segments ###
    segend = np.zeros((N_lines, 4))
    seglen = np.zeros(N_lines)
    for i in range(N_lines):
        rho1 = r[i]/np.cos(theta[pointIdx[i, 0]]-alpha[i])
        rho2 = r[i]/np.cos(theta[pointIdx[i, 1]-1]-alpha[i])
        x1 = rho1*np.cos(theta[pointIdx[i, 0]])
        y1 = rho1*np.sin(theta[pointIdx[i, 0]])
        x2 = rho2*np.cos(theta[pointIdx[i, 1]-1])
        y2 = rho2*np.sin(theta[pointIdx[i, 1]-1])
        segend[i, :] = np.hstack((x1, y1, x2, y2))
        seglen[i] = np.linalg.norm(segend[i, 0:2] - segend[i, 2:4])

    ### Filter Lines ###
    # Find and remove line segments that are too short
    goodSegIdx = np.where((seglen >= params['MIN_SEG_LENGTH']) &
                          (pointIdx[:, 1] - pointIdx[:, 0] >= params['MIN_POINTS_PER_SEGMENT']))[0]
    pointIdx = pointIdx[goodSegIdx, :]
    alpha = alpha[goodSegIdx]
    r = r[goodSegIdx]
    segend = segend[goodSegIdx, :]

    # change back to scene coordinates
    segend[:, (0, 2)] = segend[:, (0, 2)] + x_r
    segend[:, (1, 3)] = segend[:, (1, 3)] + y_r

    return alpha, r, segend, pointIdx


def SplitLinesRecursive(theta, rho, startIdx, endIdx, params):
    '''
    This function executes a recursive line-slitting algorithm, which
    recursively sub-divides line segments until no further splitting is
    required.

    Inputs:
        theta: (1D) np array of angle 'theta' from data (rads).
        rho: (1D) np array of distance 'rho' from data (m).
        startIdx: starting index of segment to be split.
        endIdx: ending index of segment to be split.
        params: dictionary of parameters.
    Outputs:
        alpha: (1D) np array of 'alpha' for each fitted line (rads).
        r: (1D) np array of 'r' for each fitted line (m).
        idx: (N_lines,2) segment's first and last point index.

    HINT: Call FitLine() to fit individual line segments.
    HINT: Call FindSplit() to find an index to split at.
    '''
    ########## Code starts here ##########
    alpha, r, idx = [], [], []
    SplitHelper(theta, rho, startIdx, endIdx, params, alpha, r, idx)
    alpha, r, idx = np.array(alpha), np.array(r), np.array(idx)

    ########## Code ends here ##########
    return alpha, r, idx

# the helper function used to recursively generate the alpha, r, idx of line segments
def SplitHelper(theta, rho, startIdx, endIdx, params, alpha, r, idx):

    alpha_fit, r_fit = FitLine(theta[startIdx:endIdx], rho[startIdx:endIdx])
    if (endIdx - startIdx) <= params['MIN_POINTS_PER_SEGMENT']:
        alpha.append(alpha_fit)
        r.append(r_fit)
        idx.append([startIdx, endIdx])
        return

    splitIdx = FindSplit(theta[startIdx:endIdx], rho[startIdx:endIdx], alpha_fit, r_fit, params)
    if splitIdx == -1:
        alpha.append(alpha_fit)
        r.append(r_fit)
        idx.append([startIdx, endIdx])
        return
    else:
        SplitHelper(theta, rho, startIdx, startIdx + splitIdx, params, alpha, r, idx)
        SplitHelper(theta, rho, startIdx + splitIdx, endIdx, params, alpha, r, idx)

def FindSplit(theta, rho, alpha, r, params):
    '''
    This function takes in a line segment and outputs the best index at which to
    split the segment, or -1 if no split should be made.

    The best point to split at is the one whose distance from the line is
    the farthest, as long as this maximum distance exceeds
    LINE_POINT_DIST_THRESHOLD and also does not divide the line into segments
    smaller than MIN_POINTS_PER_SEGMENT. Otherwise, no split should be made.

    Inputs:
        theta: (1D) np array of angle 'theta' from data (rads).
        rho: (1D) np array of distance 'rho' from data (m).
        alpha: 'alpha' of input line segment (1 number).
        r: 'r' of input line segment (1 number).
        params: dictionary of parameters.
    Output:
        splitIdx: idx at which to split line (return -1 if it cannot be split).
    '''
    ########## Code starts here ##########
    dist = np.absolute(rho * np.cos(theta - alpha) - r)

    # let the minimum number of points at beginning and end
    # be 0 and only find the split index at the remaining part
    dist[:params['MIN_POINTS_PER_SEGMENT']] = 0
    dist[-params['MIN_POINTS_PER_SEGMENT']:] = 0

    # the split index is the index where the dist is max
    max_idx = np.argmax(dist)
    if dist[max_idx] > params['LINE_POINT_DIST_THRESHOLD']:
        splitIdx = max_idx
    else:
        splitIdx = -1

    ########## Code ends here ##########
    return splitIdx

def FitLine(theta, rho):
    '''
    This function outputs a least squares best fit line to a segment of range
    data, expressed in polar form (alpha, r).

    Inputs:
        theta: (1D) np array of angle 'theta' from data (rads).
        rho: (1D) np array of distance 'rho' from data (m).
    Outputs:dist[:params["MIN_POINTS_PER_SEGEMENT"]] = 0
        alpha: 'alpha' of best fit for range data (1 number) (rads).
        r: 'r' of best fit for range data (1 number) (m).
        dist[:params["MIN_POINTS_PER_SEGEMENT"]] = 0
    '''
    ########## Code starts here ##########
    n = np.size(theta)  # number of points

    # calculate the parameter alpha
    sum_1, sum_2 = 0, 0
    for i in range(n):
        for j in range(n):
            sum_1 += rho[i] * rho[j] * np.cos(theta[i]) * np.sin(theta[j])
            sum_2 += rho[i] * rho[j] * np.cos(theta[i]+theta[j])

    numerator = np.sum(rho**2 * np.sin(2*theta)) - 2./n * sum_1
    denominator = np.sum(rho**2 * np.cos(2*theta)) - 1./n * sum_2

    alpha = 1./2 * np.arctan2(numerator, denominator) + 1./2 * np.pi

    # calculate the parameter r
    r = 1./n * np.sum(rho * np.cos(theta - alpha))

    ########## Code ends here ##########
    return alpha, r

def MergeColinearNeigbors(theta, rho, alpha, r, pointIdx, params):
    '''
    This function merges neighboring segments that are colinear and outputs a
    new set of line segments.

    Inputs:
        theta: (1D) np array of angle 'theta' from data (rads).
        rho: (1D) np array of distance 'rho' from data (m).
        alpha: (1D) np array of 'alpha' for each fitted line (rads).
        r: (1D) np array of 'r' for each fitted line (m).
        pointIdx: (N_lines,2) segment's first and last point indices.
        params: dictionary of parameters.
    Outputs:
        alphaOut: output 'alpha' of merged lines (rads).
        rOut: output 'r' of merged lines (m).
        pointIdxOut: output start and end indices of merged line segments.

    HINT: loop through line segments and try to fit a line to data points from
          two adjacent segments. If this line cannot be split, then accept the
          merge. If it can be split, do not merge.
    '''
    ########## Code starts here ##########
    alphaOut, rOut, pointIdxOut = [], [], []
    num_lines = pointIdx.shape[0]
    for i in range(1, num_lines):
         # fit one line for data from two line segments
        start = min(pointIdx[i-1][0], pointIdx[i][0])
        end = max(pointIdx[i-1][1], pointIdx[i][1])

        alpha_fit, r_fit = FitLine(theta[start:end], rho[start:end])
        splitIdx = FindSplit(theta[start:end], rho[start:end], alpha_fit, r_fit, params)

        if splitIdx == -1:
            # merge two line segments
            pointIdxOut.append([start, end])
            alphaOut.append(alpha_fit)
            rOut.append(r_fit)
        else:
            # keep two line segments
            pointIdxOut.append([pointIdx[i-1][0], pointIdx[i-1][1]])
            pointIdxOut.append([pointIdx[i][0], pointIdx[i][1]])
            alphaOut.append(alpha[i-1])
            alphaOut.append(alpha[i])
            rOut.append(r[i-1])
            rOut.append(r[i])

    alphaOut, rOut, pointIdxOut = np.array(alphaOut), np.array(rOut), np.array(pointIdxOut)
    ########## Code ends here ##########
    return alphaOut, rOut, pointIdxOut


#----------------------------------
# ImportRangeData
def ImportRangeData(filename):

    data = np.genfromtxt('./RangeData/'+filename, delimiter=',')
    x_r = data[0, 0]
    y_r = data[0, 1]
    theta = data[1:, 0]
    rho = data[1:, 1]
    return (x_r, y_r, theta, rho)
#----------------------------------


############################################################
# Main
############################################################
def main():
    # parameters for line extraction (mess with these!)
    MIN_SEG_LENGTH = 0.02  # minimum length of each line segment (m)
    LINE_POINT_DIST_THRESHOLD = 0.03  # max distance of pt from line to split
    MIN_POINTS_PER_SEGMENT = 2  # minimum number of points per line segment
    MAX_P2P_DIST = 0.3  # max distance between two adjent pts within a segment

    # Data files are formated as 'rangeData_<x_r>_<y_r>_N_pts.csv
    # where x_r is the robot's x position
    #       y_r is the robot's y position
    #       N_pts is the number of beams (e.g. 180 -> beams are 2deg apart)

    filename = 'rangeData_5_5_180.csv'
    # filename = 'rangeData_4_9_360.csv'
    # filename = 'rangeData_7_2_90.csv'

    # Import Range Data
    RangeData = ImportRangeData(filename)

    params = {'MIN_SEG_LENGTH': MIN_SEG_LENGTH,
              'LINE_POINT_DIST_THRESHOLD': LINE_POINT_DIST_THRESHOLD,
              'MIN_POINTS_PER_SEGMENT': MIN_POINTS_PER_SEGMENT,
              'MAX_P2P_DIST': MAX_P2P_DIST}

    alpha, r, segend, pointIdx = ExtractLines(RangeData, params)

    ax = PlotScene()
    ax = PlotData(RangeData, ax)
    ax = PlotRays(RangeData, ax)
    ax = PlotLines(segend, ax)

    plt.show(ax)

############################################################

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
