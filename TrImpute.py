import numpy as np
from numpy.core.fromnumeric import mean
from scipy.spatial import cKDTree
import sys
from collections import defaultdict
from os import listdir, mkdir, path
from os.path import join
from bearing import calculate_bearing, next_point, haversine, angledist, printProgressBar
import numpy as np
from queue import PriorityQueue
from time import time


def process_input(input_folder):
    files = list(listdir(input_folder))

    trajs = defaultdict(list)  # {trip_name: [trip_info]}
    data = []  # [trip_info,...]

    for fi in files:
        with open(join(input_folder, fi)) as f:
            for i, line in enumerate(f):
                # format: id, lat, lng, timestamp
                id_, lat, lng, ts = line.strip().split(',')
                lat = lat[:7]
                lng = lng[:7]
                lat, lng, ts = float(lat), float(lng), int(float(ts))
                if i:
                    # compute angle
                    a = calculate_bearing((trajs[fi][-1][1],
                                           trajs[fi][-1][0]), (lat, lng))
                    trajs[fi].append((lng, lat, ts, a))
                    data.append((lng, lat, ts, a))
                else:
                    trajs[fi].append((lng, lat, ts, 0))
                    data.append((lng, lat, ts, 0))
            if len(trajs[fi]) > 1:
                trajs[fi][0] = (trajs[fi][0][0], trajs[fi][0][1], trajs[fi][0][2],
                                trajs[fi][1][3])

    return trajs, data


def impute_trajectories(trajs):
    visited = set([])
    totalTrajs = len(list(trajs.values()))
    currentTraj = 0
    for k, traj in list(trajs.items()):
        # consider angle of first point is same as second one.
        # For each consecutive pair of points in the trajectory:
        trimpute_traj = []
        printProgressBar(currentTraj, totalTrajs, 'Progress', 'Completed')
        if len(traj) and path.getsize(input_folder + '/' + k) > 0:
            inside_traj = False
            start_time = time()
            for i, (a, b) in enumerate(zip(traj, traj[1:])):
                inside_traj = True

                s = (a[0], a[1], a[2], a[3])
                d = (b[0], b[1], b[2], b[3])

                seg_dist = haversine(s[0], s[1], d[0], d[1])
                cid = centroids.index((s[0], s[1]))
                nns = c_nns[cid]
                angles = [data[_][3] for _ in nns]

                hist, bins = np.histogram(angles, bins=[_*ANGLE_BIN for _ in
                                                        range(int(360/ANGLE_BIN)+1)], density=True)

                relevant_directions = []
                FOUND = False
                dense_segment = []
                for bearing, pr in zip(bins[:-1], hist):
                    if pr*ANGLE_BIN > CROWD_THRESHOLD\
                            and angledist(bearing, a[3]) < ANGLE_THRESHOLD:
                        # filter bins and find the average angle inside each bin

                        bin_angles = list(
                            filter(lambda ang: bearing <= ang < bearing + ANGLE_BIN, angles))
                        bin_average = sum(bin_angles)/len(bin_angles)
                        relevant_directions.append((bin_average, pr*ANGLE_BIN))

                paths = PriorityQueue()
                visited.add(s)
                for rd in relevant_directions:
                    new_pt = next_point(s[1], s[0], DISTANCE_THRESHOLD, rd[0])
                    new_pt = (round(new_pt[0], 4), round(
                        new_pt[1], 4))  # epsilon = 10m
                    new_pt_angle = calculate_bearing((s[1], s[0]),
                                                     (new_pt[1], new_pt[0]))

                    new_pt = (new_pt[0], new_pt[1], a[2], new_pt_angle)
                    remaining_dist = haversine(
                        new_pt[0], new_pt[1], d[0], d[1])

                    if remaining_dist > RADIUS_METER\
                            and new_pt not in visited:
                        paths.put((haversine(s[0], s[1], new_pt[0], new_pt[1]) +
                                   remaining_dist, (DISTANCE_THRESHOLD, [s, new_pt])))
                        visited.add(new_pt)

                    elif new_pt not in visited:
                        FOUND = True
                        dense_segment = [s, new_pt]
                        break

                while not paths.empty() and not FOUND:
                    path_weight, (leng, curr_path) = paths.get()

                    if leng >= LENGTH_FACTOR * seg_dist:
                        continue

                    curr_pt = curr_path[-1]
                    curr_angle = calculate_bearing((curr_path[-2][1], curr_path[-2][0]),
                                                   (curr_pt[1], curr_pt[0]))
                    nns = idx.query_ball_point(
                        x=(curr_pt[0], curr_pt[1]), r=RADIUS_DEGREE, p=2)

                    # Discard points that have no neighbors
                    if len(nns) < MIN_NNS:
                        continue

                    angles = [data[_][3] for _ in nns]
                    hist, bins = np.histogram(angles, bins=[_*ANGLE_BIN for _ in
                                                            range(int(360/ANGLE_BIN)+1)], density=True)
                    relevant_directions = []
                    # Angle condition:
                    for bearing, pr in zip(bins[:-1], hist):
                        if pr*ANGLE_BIN > CROWD_THRESHOLD\
                                and angledist(bearing, curr_angle) < ANGLE_THRESHOLD:
                            # filter bins and find the average angle inside each bin

                            bin_angles = list(
                                filter(lambda ang: bearing <= ang < bearing + ANGLE_BIN, angles))
                            bin_average = sum(bin_angles)/len(bin_angles)
                            relevant_directions.append(
                                (bin_average, pr*ANGLE_BIN))

                    for rd in relevant_directions:
                        new_pt = next_point(
                            curr_pt[1], curr_pt[0], DISTANCE_THRESHOLD, rd[0])
                        new_pt = (round(new_pt[0], 4), round(new_pt[1], 4))
                        new_pt_angle = calculate_bearing((curr_pt[1], curr_pt[0]),
                                                         (new_pt[1], new_pt[0]))

                        new_pt = (new_pt[0], new_pt[1],
                                  curr_pt[2], new_pt_angle)
                        dist = haversine(new_pt[0], new_pt[1], d[0], d[1])

                        # Candidate point, consider rest of the path
                        if dist > RADIUS_METER and new_pt not in visited:
                            paths.put((leng+DISTANCE_THRESHOLD + dist, (leng + haversine(curr_pt[0], curr_pt[1],
                                                                                         new_pt[0], new_pt[1]), curr_path + [new_pt])))
                            visited.add(new_pt)

                        # We found the last point of the imputation
                        elif new_pt not in visited:
                            FOUND = True
                            dense_segment = curr_path + [new_pt]
                            break
                # No imputation
                if len(dense_segment) == 0:
                    dense_segment.append(a)
                    trimpute_traj += dense_segment
                if FOUND:
                    # append dense segment points to the densified trajectory
                    time_diff = d[2] - s[2]
                    delta_time = int(
                        time_diff/max(1, (len(dense_segment) - 1)))
                    ts = s[2]
                    dense_seg_temporal = [dense_segment[0]]
                    for j, _ in enumerate(dense_segment[1:]):
                        dense_seg_temporal.append((_[0], _[1], ts + (j+1) *
                                                   delta_time, _[3]))
                    trimpute_traj += dense_seg_temporal

            # Write the densified trimpute_traj to the output file
            with open(output_folder+'/%s' % k, 'w') as g:
                for pt in trimpute_traj:
                    g.write('%s,%s,%s,%s\n' % pt)
                # Add the last point in the sparse trajectory
                if inside_traj:
                    g.write('%s,%s,%s,%s\n' % (d[0], d[1], d[2], d[3]))
            timeTaken = time() - start_time
            with open(output_folder+'/results.csv', 'a') as h:
                h.write("%s,%s,%s,%s\n" % (k, len(traj), len(trimpute_traj), timeTaken))
            currentTraj += 1


if __name__ == '__main__':

    try:
        input_folder = "./datasets/input/%s" % (sys.argv[1])
        output_folder = "./datasets/output/%s" % (sys.argv[2])
    except Exception as e:
        print("Not enough arguments!")
        exit()

    try:
        mkdir(output_folder)
    except Exception as e:
        print(e)
        exit()

    start_time = time()
    print('started')

    # Main parameters
    CANDIDATE_POINTS = 6  # N
    CROWD_THRESHOLD = 0.005  # alpha
    ANGLE_THRESHOLD = 120  # delta
    DISTANCE_THRESHOLD = 50  # d

    # Global values
    RADIUS_METER = DISTANCE_THRESHOLD
    ANGLE_BIN = 360/CANDIDATE_POINTS
    RADIUS_DEGREE = RADIUS_METER * 10e-6
    LENGTH_FACTOR = 3  # path should not exceed 3 times the birdfly distance.
    MIN_NNS = 1  # at least one neighbor

    trajs, data = process_input(input_folder)

    print('Indexing points')

    points = [(_[0], _[1]) for _ in data]
    idx = cKDTree(list(points))
    centroids = list(set(points))

    # store the neighbors within RADIUS meters for every centorid
    c_nns = idx.query_ball_point(x=list(centroids), r=RADIUS_DEGREE, p=2)

    print('Imputation started')

    impute_trajectories(trajs)

    print("--- %s seconds ---" % (time() - start_time))
