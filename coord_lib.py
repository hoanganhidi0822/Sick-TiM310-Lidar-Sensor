from math import cos, sin, pi
import numpy as np

def ang2cartezian(axis, distance):
    """
    Convert angular coordinates and distances to Cartesian coordinates.
    
    Parameters:
        axis (array-like): Angular coordinates in degrees.
        distance (array-like): Distances corresponding to the angles.
    
    Returns:
        tuple: (x, y) where x and y are arrays of Cartesian coordinates.
    """
    axis = np.asarray(axis, dtype=float)
    distance = np.asarray(distance, dtype=float)
    
    if len(axis) != len(distance):
        raise ValueError(f"Error: Inputs have different lengths: axis length = {len(axis)}, distance length = {len(distance)}")
    
    x = np.zeros(len(distance), dtype=np.float64)
    y = np.zeros(len(distance), dtype=np.float64)
    
    for j in range(len(distance)):
        x[j] = cos(axis[j] / 180.0 * pi) * distance[j]
        y[j] = sin(axis[j] / 180.0 * pi) * distance[j]
    
    return x, y

def ang_segmentation(scan, max_diff=150):
    """
    Segment the angular scan data based on a maximum difference threshold.
    
    Parameters:
        scan (array-like): Array of scan data.
        max_diff (int): Maximum allowed difference between adjacent points to be considered part of the same segment.
    
    Returns:
        list: A list of segments, where each segment is represented as [start_id, len_of_segment, segment_data].
    """
    scan = np.asarray(scan, dtype=float)
    segments = []
    segm_start = [0]
    start_segm = 0
    
    for j in range(len(scan) - 1):
        diff = abs(scan[j] - scan[j + 1])
        if diff > max_diff:
            if start_segm != j:
                end_segm = j
                segments.append([segm_start[-1], len(scan[start_segm:end_segm + 1]), scan[start_segm:end_segm + 1]])
                segm_start.append(j + 1)
            start_segm = j + 1
    
    # Add the final segment
    segments.append([start_segm, len(scan[start_segm:]), scan[start_segm:]])
    
    return segments
