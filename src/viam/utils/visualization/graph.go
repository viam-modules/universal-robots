package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strconv"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"github.com/rocketlaunchr/dataframe-go/imports"
	chart "github.com/wcharczuk/go-chart"
)

type trajInput struct {
	Timestamp   float64
	JointInputs []referenceframe.Input
}

type trajPoses struct {
	Timestamp float64
	Pose      spatialmath.Pose
}

type config struct {
	TrajectoryCSV string `json:"trajectory_csv"`
	WaypointsCSV  string `json:"waypoints_csv"`
}

func loadFile(path string) (*os.File, error) {
	expandedPath := path
	if len(path) > 1 && path[:2] == "~/" {
		home, err := os.UserHomeDir()
		if err != nil {
			return nil, err
		}
		expandedPath = filepath.Join(home, path[2:])
	}
	return os.Open(expandedPath)
}

func loadConfig(path string) (config, error) {
	var cfg config
	file, err := os.Open(path)
	if err != nil {
		return cfg, err
	}
	defer file.Close()
	return cfg, json.NewDecoder(file).Decode(&cfg)
}

func readTrajectoryCSV(path string, urModel referenceframe.Model) ([]trajInput, []trajPoses, error) {
	file, err := loadFile(path)
	if err != nil {
		return nil, nil, err
	}
	defer file.Close()

	df, err := imports.LoadFromCSV(context.Background(), file, imports.CSVLoadOptions{
		DictateDataType: map[string]interface{}{
			"t(s)": float64(0),
			"j0":   float64(0),
			"j1":   float64(0),
			"j2":   float64(0),
			"j3":   float64(0),
			"j4":   float64(0),
			"j5":   float64(0),
		},
	})
	if err != nil {
		return nil, nil, err
	}

	var trajectoryInputs []trajInput
	var allInputs [][]referenceframe.Input
	// start at 1 so we skip the header
	for i := 1; i < df.NRows(); i++ {
		row := df.Row(i, false)
		rowInputs := []float64{}
		for j := range 6 {
			inputValue, ok := row["j"+strconv.Itoa(j)].(float64)
			if !ok {
				return nil, nil, fmt.Errorf("j%d not found in row", j)
			}
			rowInputs = append(rowInputs, inputValue)
		}
		timeStep, ok := row["t(s)"].(float64)
		if !ok {
			return nil, nil, fmt.Errorf("t(s) not found in row")
		}
		trajectoryInputs = append(trajectoryInputs,
			trajInput{
				Timestamp:   timeStep,
				JointInputs: referenceframe.FloatsToInputs(rowInputs),
			},
		)
		allInputs = append(allInputs, referenceframe.FloatsToInputs(rowInputs))
	}

	if len(trajectoryInputs) == 0 {
		return nil, nil, fmt.Errorf("readTrajectoryCSV produced []trajInput of length zero")
	}

	posesFromInputs, err := transformInputs(urModel, allInputs)
	if err != nil {
		return nil, nil, err
	}

	var trajectoryPoses []trajPoses
	for i, pose := range posesFromInputs {
		trajectoryPoses = append(trajectoryPoses, trajPoses{
			Timestamp: trajectoryInputs[i].Timestamp,
			Pose:      pose,
		})
	}
	return trajectoryInputs, trajectoryPoses, nil
}

func readWaypointCSV(path string, urModel referenceframe.Model) ([][]referenceframe.Input, []spatialmath.Pose, error) {
	file, err := loadFile(path)
	if err != nil {
		return nil, nil, err
	}
	defer file.Close()

	df, err := imports.LoadFromCSV(context.Background(), file, imports.CSVLoadOptions{
		InferDataTypes: false,
		// induce keys so that we know the value of j_i
		DictateDataType: map[string]interface{}{
			"0": float64(0),
			"1": float64(0),
			"2": float64(0),
			"3": float64(0),
			"4": float64(0),
			"5": float64(0),
		},
	})
	if err != nil {
		return nil, nil, err
	}

	var allInputs [][]referenceframe.Input
	for i := 0; i < df.NRows(); i++ {
		row := df.Row(i, false)
		rowInputs := []float64{}
		for j := 0; j < 6; j++ {
			strVal, ok := row[j].(string)
			if !ok {
				return nil, nil, fmt.Errorf("key %d not found in row %d", j, i)
			}
			parsedVal, err := strconv.ParseFloat(strVal, 64)
			if err != nil {
				return nil, nil, fmt.Errorf("failed to parse float at row %d, col %d with the followin error: %v", i, j, err)
			}

			rowInputs = append(rowInputs, parsedVal)
		}
		allInputs = append(allInputs, referenceframe.FloatsToInputs(rowInputs))
	}

	if len(allInputs) == 0 {
		return nil, nil, fmt.Errorf("readWaypointCSV produced [][]referenceframe.Input of length zero")
	}

	waypointPoses, err := transformInputs(urModel, allInputs)
	if err != nil {
		return nil, nil, err
	}

	return allInputs, waypointPoses, nil
}

func transformInputs(model referenceframe.Model, inputSets [][]referenceframe.Input) ([]spatialmath.Pose, error) {
	var poses []spatialmath.Pose
	for _, inputs := range inputSets {
		pose, err := model.Transform(inputs)
		if err != nil {
			return nil, err
		}
		poses = append(poses, pose)
	}
	if len(poses) == 0 {
		return nil, fmt.Errorf("transformInputs produced []spatialmath.Pose of length zero")
	}
	return poses, nil
}

func main() {
	logger := logging.NewLogger("graph")

	cfg, err := loadConfig("config.json")
	if err != nil {
		logger.Fatalf("failed to load config: %v", err)
	}

	// TODO: when ur20 kinematics exist we should add that as a parameter to config.json
	urModel, err := referenceframe.ParseModelJSONFile("../src/kinematics/ur5e.json", "")
	if err != nil {
		logger.Fatalf("failed to load model: %v", err)
	}

	trajectoryInputs, trajectoryPoses, err := readTrajectoryCSV(cfg.TrajectoryCSV, urModel)
	if err != nil {
		logger.Fatalf("failed to read trajectory CSV: %v", err)
	}

	waypointInputs, waypointPoses, err := readWaypointCSV(cfg.WaypointsCSV, urModel)
	if err != nil {
		logger.Fatalf("failed to read waypoint CSV: %v", err)
	}

	if err := plotJointComparison(trajectoryInputs, waypointInputs); err != nil {
		logger.Fatalf("could not plot joint comparisons: %v", err)
	}

	if err := plotPoseComparison(trajectoryPoses, waypointPoses); err != nil {
		logger.Fatalf("could not plot pose comparisons: %v", err)
	}
}

func evenlySpacedTimes(start, end float64, count int) []float64 {
	times := make([]float64, count)
	if count == 0 {
		return times
	}
	denom := 1.0
	if count > 1 {
		denom = float64(count - 1)
	}
	for i := 0; i < count; i++ {
		times[i] = start + (end-start)*float64(i)/denom
	}
	return times
}

func saveChartPNG(
	fileName, yAxisLabel string,
	xTraj, yTraj, xWay, yWay []float64,
) error {
	trajSeries := chart.ContinuousSeries{
		Name:    "Trajectory",
		XValues: xTraj,
		YValues: yTraj,
		Style: chart.Style{
			Show:        true,
			StrokeColor: chart.ColorBlue,
		},
	}

	waypointSeries := chart.ContinuousSeries{
		Name:    "Waypoints",
		XValues: xWay,
		YValues: yWay,
		Style: chart.Style{
			Show:        true,
			StrokeColor: chart.ColorOrange,
		},
	}

	graph := chart.Chart{
		XAxis: chart.XAxis{
			Name:      "Time (s)",
			NameStyle: chart.StyleShow(),
			Style:     chart.StyleShow(),
		},
		YAxis: chart.YAxis{
			Name:      yAxisLabel,
			NameStyle: chart.StyleShow(),
			Style:     chart.StyleShow(),
		},
		Series: []chart.Series{
			trajSeries,
			waypointSeries,
		},
		Elements: []chart.Renderable{
			chart.Legend(&chart.Chart{
				Series: []chart.Series{
					trajSeries,
					waypointSeries,
				},
			}),
		},
	}

	f, err := os.Create(fileName)
	if err != nil {
		return err
	}
	defer f.Close()
	return graph.Render(chart.PNG, f)
}

func plotCharts(
	prefix, yAxisLabelFormat string,
	trajTimes []float64,
	trajSeries [][]float64,
	waypointTimes []float64,
	waypointSeries [][]float64,
	isPosition bool,
) error {
	if len(trajSeries) != len(waypointSeries) {
		return fmt.Errorf("trajectory and waypoint series count mismatch")
	}
	positionSuffix := []string{"X", "Y", "Z"}

	for i := range trajSeries {
		fileName := fmt.Sprintf("%s%d_comparison.png", prefix, i)
		if isPosition {
			fileName = fmt.Sprintf("%s%s_comparison.png", prefix, positionSuffix[i])
		}
		yAxis := fmt.Sprintf(yAxisLabelFormat, i)
		if err := saveChartPNG(fileName, yAxis, trajTimes, trajSeries[i], waypointTimes, waypointSeries[i]); err != nil {
			return err
		}
	}
	return nil
}

func plotJointComparison(trajInputs []trajInput, waypointInputs [][]referenceframe.Input) error {
	jointCount := 6
	trajTimes := make([]float64, len(trajInputs))
	for i, t := range trajInputs {
		trajTimes[i] = t.Timestamp
	}

	totalTime := trajTimes[len(trajTimes)-1]
	if totalTime <= 0 {
		return fmt.Errorf("invalid totalTime: %v", totalTime)
	}

	numWaypoints := len(waypointInputs)
	waypointTimes := evenlySpacedTimes(trajTimes[0], totalTime, numWaypoints)

	trajSeries := make([][]float64, jointCount)
	waypointSeries := make([][]float64, jointCount)

	for jointIdx := 0; jointIdx < jointCount; jointIdx++ {
		trajJVals := make([]float64, len(trajInputs))
		for i := range trajInputs {
			trajJVals[i] = trajInputs[i].JointInputs[jointIdx].Value
		}
		waypointJVals := make([]float64, numWaypoints)
		for i := range waypointInputs {
			waypointJVals[i] = waypointInputs[i][jointIdx].Value
		}
		trajSeries[jointIdx] = trajJVals
		waypointSeries[jointIdx] = waypointJVals
	}

	return plotCharts("joint", "Joint %d Angle (rad)", trajTimes, trajSeries, waypointTimes, waypointSeries, false)
}

func plotPoseComparison(outputTraj []trajPoses, waypointPoses []spatialmath.Pose) error {
	startTime := outputTraj[0].Timestamp
	endTime := outputTraj[len(outputTraj)-1].Timestamp
	totalTime := endTime - startTime
	if totalTime <= 0 {
		return fmt.Errorf("invalid trajectory duration: %v", totalTime)
	}

	trajTimes := make([]float64, len(outputTraj))
	trajX := make([]float64, len(outputTraj))
	trajY := make([]float64, len(outputTraj))
	trajZ := make([]float64, len(outputTraj))
	for i, tp := range outputTraj {
		trajTimes[i] = tp.Timestamp
		pt := tp.Pose.Point()
		trajX[i] = pt.X
		trajY[i] = pt.Y
		trajZ[i] = pt.Z
	}

	numWaypoints := len(waypointPoses)
	waypointTimes := evenlySpacedTimes(startTime, endTime, numWaypoints)
	waypointX := make([]float64, numWaypoints)
	waypointY := make([]float64, numWaypoints)
	waypointZ := make([]float64, numWaypoints)
	for i, pose := range waypointPoses {
		pt := pose.Point()
		waypointX[i] = pt.X
		waypointY[i] = pt.Y
		waypointZ[i] = pt.Z
	}

	trajSeries := [][]float64{trajX, trajY, trajZ}
	waypointSeries := [][]float64{waypointX, waypointY, waypointZ}

	return plotCharts("position_", "Position %c (mm)", trajTimes, trajSeries, waypointTimes, waypointSeries, true)
}
