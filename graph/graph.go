package main

import (
	"encoding/csv"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"os"
	"path/filepath"
	"strconv"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

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

func expandPath(path string) string {
	if len(path) > 1 && path[:2] == "~/" {
		home, err := os.UserHomeDir()
		if err != nil {
			log.Fatal(err)
		}
		return filepath.Join(home, path[2:])
	}
	return path
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

func parseFloatSlice(fields []string) ([]float64, error) {
	var values []float64
	for _, str := range fields {
		v, err := strconv.ParseFloat(str, 64)
		if err != nil {
			return nil, err
		}
		values = append(values, v)
	}
	return values, nil
}

func readCSV(path string, skipHeader bool) ([][]string, error) {
	file, err := os.Open(expandPath(path))
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	if skipHeader {
		// Skip header if and only if one exists
		_, err = reader.Read()
		if err != nil {
			return nil, err
		}
	}

	var rows [][]string
	for {
		row, err := reader.Read()
		if err == io.EOF {
			break
		}
		if err != nil {
			log.Printf("error reading row: %v", err)
			continue
		}
		rows = append(rows, row)
	}
	return rows, nil
}

func readTrajectoryCSV(path string) ([]trajInput, error) {
	rows, err := readCSV(path, true)
	if err != nil {
		return nil, err
	}
	var data []trajInput
	for _, row := range rows {
		if len(row) != 13 {
			log.Printf("invalid trajectory row length: %d", len(row))
			continue
		}
		t, err := strconv.ParseFloat(row[0], 64)
		if err != nil {
			log.Printf("invalid timestamp: %v", err)
			continue
		}
		jointVals, err := parseFloatSlice(row[1:7])
		if err != nil {
			log.Printf("invalid joint values: %v", err)
			continue
		}
		data = append(data, trajInput{
			Timestamp:   t,
			JointInputs: referenceframe.FloatsToInputs(jointVals),
		})
	}
	if len(data) == 0 {
		return nil, errors.New("readTrajectoryCSV produced []trajInput of length zero")
	}
	return data, nil
}

func readWaypointCSV(path string) ([][]referenceframe.Input, error) {
	rows, err := readCSV(path, false)
	if err != nil {
		return nil, err
	}
	var allInputs [][]referenceframe.Input
	for _, row := range rows {
		if len(row) != 6 {
			log.Printf("invalid waypoint row length: %d", len(row))
			continue
		}
		vals, err := parseFloatSlice(row)
		if err != nil {
			log.Printf("invalid waypoint values: %v", err)
			continue
		}
		allInputs = append(allInputs, referenceframe.FloatsToInputs(vals))
	}
	if len(allInputs) == 0 {
		return nil, errors.New("readWaypointCSV produced [][]referenceframe.Input of length zero")
	}
	return allInputs, nil
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
		return nil, errors.New("transformInputs produced []spatialmath.Pose of length zero")
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

	trajInputs, err := readTrajectoryCSV(cfg.TrajectoryCSV)
	if err != nil {
		logger.Fatalf("failed to read trajectory CSV: %v", err)
	}

	var trajInputSets [][]referenceframe.Input
	for _, t := range trajInputs {
		trajInputSets = append(trajInputSets, t.JointInputs)
	}
	trajectoryPoses, err := transformInputs(urModel, trajInputSets)
	if err != nil {
		logger.Fatal(err)
	}

	var outputTraj []trajPoses
	for i, pose := range trajectoryPoses {
		outputTraj = append(outputTraj, trajPoses{
			Timestamp: trajInputs[i].Timestamp,
			Pose:      pose,
		})
	}

	waypointInputs, err := readWaypointCSV(cfg.WaypointsCSV)
	if err != nil {
		logger.Fatalf("failed to read waypoint CSV: %v", err)
	}

	waypointPoses, err := transformInputs(urModel, waypointInputs)
	if err != nil {
		logger.Fatalf("could not get waypoint poses: %v", err)
	}

	if err := plotJointComparison(trajInputs, waypointInputs); err != nil {
		logger.Fatalf("could not plot joint comparisons: %v", err)
	}

	if err := plotPoseComparison(outputTraj, waypointPoses); err != nil {
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
			chart.ContinuousSeries{
				Name:    "Trajectory",
				XValues: xTraj,
				YValues: yTraj,
				Style: chart.Style{
					Show:        true,
					StrokeColor: chart.ColorBlue,
				},
			},
			chart.ContinuousSeries{
				Name:    "Waypoints",
				XValues: xWay,
				YValues: yWay,
				Style: chart.Style{
					Show:        true,
					StrokeColor: chart.ColorOrange,
				},
			},
		},
	}
	f, err := os.Create(fileName)
	if err != nil {
		return err
	}
	defer f.Close()
	return graph.Render(chart.PNG, f)
}

func plotJointComparison(trajInputs []trajInput, waypointInputs [][]referenceframe.Input) error {
	for jointIdx := range 6 {
		trajTimes := make([]float64, len(trajInputs))
		trajJVals := make([]float64, len(trajInputs))
		for i, t := range trajInputs {
			trajTimes[i] = t.Timestamp
			trajJVals[i] = t.JointInputs[jointIdx].Value
		}

		totalTime := trajTimes[len(trajTimes)-1]
		if totalTime <= 0 {
			return fmt.Errorf("Invalid totalTime: %v", totalTime)
		}

		numWaypoints := len(waypointInputs)
		waypointTimes := evenlySpacedTimes(trajTimes[0], totalTime, numWaypoints)
		waypointJVals := make([]float64, numWaypoints)
		for i := range numWaypoints {
			waypointJVals[i] = waypointInputs[i][jointIdx].Value
		}

		fileName := fmt.Sprintf("joint%d_comparison.png", jointIdx)
		yAxis := fmt.Sprintf("Joint %d Angle (rad)", jointIdx)
		if err := saveChartPNG(fileName, yAxis, trajTimes, trajJVals, waypointTimes, waypointJVals); err != nil {
			return err
		}
	}
	return nil
}

func plotPoseComparison(outputTraj []trajPoses, waypointPoses []spatialmath.Pose) error {
	startTime := outputTraj[0].Timestamp
	endTime := outputTraj[len(outputTraj)-1].Timestamp
	totalTime := endTime - startTime
	if totalTime <= 0 {
		return fmt.Errorf("invalid trajectory duration: %v", totalTime)
	}

	trajTimes := make([]float64, len(outputTraj))
	trajX, trajY, trajZ := make([]float64, len(outputTraj)), make([]float64, len(outputTraj)), make([]float64, len(outputTraj))
	for i, tp := range outputTraj {
		trajTimes[i] = tp.Timestamp
		trajX[i] = tp.Pose.Point().X
		trajY[i] = tp.Pose.Point().Y
		trajZ[i] = tp.Pose.Point().Z
	}

	numWaypoints := len(waypointPoses)
	waypointTimes := evenlySpacedTimes(startTime, endTime, numWaypoints)
	waypointX, waypointY, waypointZ := make([]float64, numWaypoints), make([]float64, numWaypoints), make([]float64, numWaypoints)
	for i, pose := range waypointPoses {
		waypointX[i] = pose.Point().X
		waypointY[i] = pose.Point().Y
		waypointZ[i] = pose.Point().Z
	}

	if err := saveChartPNG("position_X.png", "Position X (mm)", trajTimes, trajX, waypointTimes, waypointX); err != nil {
		return err
	}
	if err := saveChartPNG("position_Y.png", "Position Y (mm)", trajTimes, trajY, waypointTimes, waypointY); err != nil {
		return err
	}
	if err := saveChartPNG("position_Z.png", "Position Z (mm)", trajTimes, trajZ, waypointTimes, waypointZ); err != nil {
		return err
	}
	return nil
}
