package main

import (
	"encoding/json"
	"os"
	"strconv"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
)

type TrajectoryGroup []motionplan.Trajectory

func (tg *TrajectoryGroup) getJoints() [][]float64 {
	allInputs := [][]float64{}
	for _, t := range *tg {
		for _, ins := range t {
			for _, in := range ins {
				// assume any 6dof arm is our arm
				if len(in) == 6 {
					allInputs = append(allInputs, referenceframe.InputsToFloats(in))
				}
			}
		}
	}
	return allInputs
}

func ReadTrajectoriesFromFile(fileName string) (trajectories []*TrajectoryGroup, err error) {
	file, err := os.Open(fileName) //nolint:gosec // File path from user input
	if err != nil {
		return nil, err
	}
	defer func() {
		if closeErr := file.Close(); closeErr != nil && err == nil {
			err = closeErr
		}
	}()
	var allTrajectories []*TrajectoryGroup
	if err = json.NewDecoder(file).Decode(&allTrajectories); err != nil {
		return nil, err
	}
	return allTrajectories, err
}

func parseAndAddPosesCachedPlan(cached string, model referenceframe.Model) ([][]float64, [][]float64, error) {
	tgs, err := ReadTrajectoriesFromFile(cached)
	if err != nil {
		return nil, nil, err
	}
	jointsCached := [][]float64{}
	for _, tg := range tgs {
		jointsCached = append(jointsCached, tg.getJoints()...)
	}
	// // we need to create series for poses which is a 7 dimensional
	// xSeries := dataframe.NewSeriesFloat64("x", nil)
	// ySeries := dataframe.NewSeriesFloat64("y", nil)
	// zSeries := dataframe.NewSeriesFloat64("z", nil)
	// oxSeries := dataframe.NewSeriesFloat64("ox", nil)
	// oySeries := dataframe.NewSeriesFloat64("oy", nil)
	// ozSeries := dataframe.NewSeriesFloat64("oz", nil)
	// thetaSeries := dataframe.NewSeriesFloat64("theta", nil)

	cx := []float64{}
	cy := []float64{}
	cz := []float64{}
	j0 := []float64{}
	j1 := []float64{}
	j2 := []float64{}
	j3 := []float64{}
	j4 := []float64{}
	j5 := []float64{}

	for _, joints := range jointsCached {
		// perform FK to get the pose
		pose, err := model.Transform(referenceframe.FloatsToInputs(joints))
		if err != nil {
			return nil, nil, err
		}
		j0 = append(j0, joints[0])
		j1 = append(j1, joints[1])
		j2 = append(j2, joints[2])
		j3 = append(j3, joints[3])
		j4 = append(j4, joints[4])
		j5 = append(j5, joints[5])

		cx = append(cx, pose.Point().X)
		cy = append(cy, pose.Point().Y)
		cz = append(cz, pose.Point().Z)
	}

	return [][]float64{j0, j1, j2, j3, j4, j5}, [][]float64{cx, cy, cz}, nil
}

func plotJointsAndPosesCachedPlan(joints, poses [][]float64) error {

	waypointTime := []float64{}
	for i := range len(poses[0]) {
		waypointTime = append(waypointTime, float64(i))
	}
	// For joints, just use the index number as string
	if err := plotChartsCachedData("joint", "Joint %s Angle (rad)", waypointTime, joints, func(i int) string {
		return strconv.Itoa(i)
	}); err != nil {
		return err
	}

	// For positions, use letters X, Y, Z
	suffix := []string{"X", "Y", "Z"}
	if err := plotChartsCachedData("position_", "Position %s (mm)", waypointTime, poses, func(i int) string {
		return suffix[i]
	}); err != nil {
		return err
	}
	return nil
}
