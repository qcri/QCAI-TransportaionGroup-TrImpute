## TrImpute 
TrImpute is a novel framework for trajectory imputation that inserts artificial GPS points between the real ones in a way that the imputed trajectories end up to be very similar to the case if such trajectories were collected with a much higher sampling rate. Unlike all prior trajectory imputation techniques, TrImpute does not assume the knowledge of the underlying road network. 

## Input
The input is a folder containing the sparse trajectories that we want to be imputed. Each sparse trajectory is a csv file containing the following metadata: 

<i> car_id, latitude, longtitude, timestamp </i> 

## Running TrImpute 
TrImpute can be run from the command line as following. The input_folder is to be included inside the <i> datasets/input/ </i> folder provided:.
```
python TrImpute.py input_folder output_folder
```

## Adjusting Parameters
We recognize that not all input data is the same. Some will consist of GPS points every 100m and some every 2 kilometers. For that reason we provide a set of parameters that can be adjusted by the user. To change them, refer the code of TrImpute.py in lines 191-194: 

```
    CANDIDATE_POINTS = 6     # N
    CROWD_THRESHOLD = 0.005  # alpha
    ANGLE_THRESHOLD = 120    # delta
    DISTANCE_THRESHOLD = 50  # d
```

## Output
After the script finishes running, the output folder will be populated with csv files containing the imputed GPS points from the corresponding sparse trajectories. 
Here is an example that visualizes the contribution of TrImpute. 

White: Ground Truth 

Red: Sparse Trajectory

Green: TrImpute Result

![Alt text](figures/fig2.png?raw=true "Figure 2")

![Alt text](figures/fig1.png?raw=true "Figure 1")

