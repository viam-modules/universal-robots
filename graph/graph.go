package main

import (
	"encoding/csv"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"os"
	"path/filepath"
	"strconv"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
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
	TrajectoryCSV         string `json:"trajectory_csv"`
	WaypointsCSV          string `json:"waypoints_csv"`
	OutputTrajectoryPoses string `json:"output_trajectory_poses"`
	OutputWaypointPoses   string `json:"output_waypoint_poses"`
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

func transformInputs(model referenceframe.Model, inputSets [][]referenceframe.Input) ([]spatialmath.Pose, error) {
	var poses []spatialmath.Pose
	for _, inputs := range inputSets {
		pose, err := model.Transform(inputs)
		if err != nil {
			return nil, err
		}
		poses = append(poses, pose)
	}
	return poses, nil
}

func readCSV(path string) ([][]string, error) {
	file, err := os.Open(expandPath(path))
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	_, err = reader.Read() // skip header
	if err != nil {
		return nil, err
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
	rows, err := readCSV(path)
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
	return data, nil
}

func readWaypointCSV(path string) ([][]referenceframe.Input, error) {
	rows, err := readCSV(path)
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
	return allInputs, nil
}

func writeCSV(path string, header []string, rows [][]string) error {
	file, err := os.Create(expandPath(path))
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	if err := writer.Write(header); err != nil {
		return err
	}
	for _, row := range rows {
		if err := writer.Write(row); err != nil {
			return err
		}
	}
	return nil
}

func writeTrajectoryPosesCSV(path string, data []trajPoses) error {
	header := []string{"t(s)", "x", "y", "z", "ox", "oy", "oz", "theta"}
	var rows [][]string
	for _, p := range data {
		ov := p.Pose.Orientation().OrientationVectorDegrees()
		row := []string{
			fmt.Sprintf("%.6f", p.Timestamp),
			fmt.Sprintf("%.6f", p.Pose.Point().X),
			fmt.Sprintf("%.6f", p.Pose.Point().Y),
			fmt.Sprintf("%.6f", p.Pose.Point().Z),
			fmt.Sprintf("%.6f", ov.OX),
			fmt.Sprintf("%.6f", ov.OY),
			fmt.Sprintf("%.6f", ov.OZ),
			fmt.Sprintf("%.6f", ov.Theta),
		}
		rows = append(rows, row)
	}
	return writeCSV(path, header, rows)
}

func writeWaypointPosesCSV(path string, poses []spatialmath.Pose) error {
	header := []string{"x", "y", "z", "ox", "oy", "oz", "theta"}
	var rows [][]string
	for _, p := range poses {
		ov := p.Orientation().OrientationVectorDegrees()
		row := []string{
			fmt.Sprintf("%.6f", p.Point().X),
			fmt.Sprintf("%.6f", p.Point().Y),
			fmt.Sprintf("%.6f", p.Point().Z),
			fmt.Sprintf("%.6f", ov.OX),
			fmt.Sprintf("%.6f", ov.OY),
			fmt.Sprintf("%.6f", ov.OZ),
			fmt.Sprintf("%.6f", ov.Theta),
		}
		rows = append(rows, row)
	}
	return writeCSV(path, header, rows)
}

func main() {
	logger := logging.NewLogger("graph")

	cfg, err := loadConfig("config.json")
	if err != nil {
		logger.Fatalf("failed to load config: %v", err)
	}

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
		logger.Error(err)
		return
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
		logger.Error(err)
		return
	}

	if err := writeTrajectoryPosesCSV(cfg.OutputTrajectoryPoses, outputTraj); err != nil {
		logger.Error(err)
	}
	if err := writeWaypointPosesCSV(cfg.OutputWaypointPoses, waypointPoses); err != nil {
		logger.Error(err)
	}
}
