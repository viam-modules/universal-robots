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

func loadConfig(path string) (config, error) {
	var cfg config
	file, err := os.Open(path)
	if err != nil {
		return cfg, err
	}
	defer file.Close()

	decoder := json.NewDecoder(file)
	err = decoder.Decode(&cfg)
	return cfg, err
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

func main() {
	logger := logging.NewLogger("graph")
	cfg, err := loadConfig("config.json")
	if err != nil {
		logger.Fatalf("failed to load config: %v", err)
	}

	urModel, err := referenceframe.ParseModelJSONFile("../src/kinematics/ur5e.json", "")
	if err != nil {
		logger.Fatalf("failed to load ur5e.json: %v", err)
	}

	trajInputs, err := readTrajectoryCSV(cfg.TrajectoryCSV)
	if err != nil {
		logger.Fatalf("failed to read trajectory CSV: %v", err)
	}

	var trajectoryPoses []trajPoses
	for _, trajInput := range trajInputs {
		pose, err := urModel.Transform(trajInput.JointInputs)
		if err != nil {
			logger.Error(err)
			return
		}
		trajectoryPoses = append(trajectoryPoses, trajPoses{Timestamp: trajInput.Timestamp, Pose: pose})
	}

	waypoints, err := readWaypointCSV(cfg.WaypointsCSV)
	if err != nil {
		logger.Fatalf("failed to read waypoint CSV: %v", err)
	}

	var waypointPoses []spatialmath.Pose
	for _, pos := range waypoints {
		pose, err := urModel.Transform(pos)
		if err != nil {
			logger.Error(err)
			return
		}
		waypointPoses = append(waypointPoses, pose)
	}

	if err := writeTrajectoryPosesCSV(cfg.OutputTrajectoryPoses, trajectoryPoses); err != nil {
		logger.Error(err)
		return
	}
	if err := writeWaypointPosesCSV(cfg.OutputWaypointPoses, waypointPoses); err != nil {
		logger.Error(err)
		return
	}
}

func parseTrajectoryRow(row []string) (trajInput, error) {
	if len(row) != 13 {
		return trajInput{}, fmt.Errorf("invalid row length: %d", len(row))
	}
	t, err := strconv.ParseFloat(row[0], 64)
	if err != nil {
		return trajInput{}, err
	}

	var jointPositions []float64
	for i := 1; i <= 6; i++ {
		pos, err := strconv.ParseFloat(row[i], 64)
		if err != nil {
			return trajInput{}, err
		}
		jointPositions = append(jointPositions, pos)
	}

	return trajInput{Timestamp: t, JointInputs: referenceframe.FloatsToInputs(jointPositions)}, nil
}

func readTrajectoryCSV(filepath string) ([]trajInput, error) {
	file, err := os.Open(expandPath(filepath))
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	_, err = reader.Read() // discard header
	if err != nil {
		return nil, err
	}

	var data []trajInput
	for {
		record, err := reader.Read()
		if err == io.EOF {
			break
		}
		if err != nil {
			log.Printf("error reading row: %v", err)
			continue
		}

		input, err := parseTrajectoryRow(record)
		if err != nil {
			log.Printf("error parsing row: %v", err)
			continue
		}
		data = append(data, input)
	}
	return data, nil
}

func readWaypointCSV(filepath string) ([][]referenceframe.Input, error) {
	file, err := os.Open(expandPath(filepath))
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	_, err = reader.Read() // read the first line of the csv file which houses the header
	if err != nil {
		return nil, err
	}

	var positions [][]referenceframe.Input
	for {
		record, err := reader.Read()
		if err == io.EOF {
			break
		}
		if err != nil {
			log.Printf("error reading row: %v", err)
			continue
		}

		if len(record) != 6 {
			log.Printf("invalid waypoint row length: %d", len(record))
			continue
		}

		var joints []float64
		for _, str := range record {
			val, err := strconv.ParseFloat(str, 64)
			if err != nil {
				log.Printf("invalid joint value: %v", err)
				continue
			}
			joints = append(joints, val)
		}
		positions = append(positions, referenceframe.FloatsToInputs(joints))
	}
	return positions, nil
}

func writeTrajectoryPosesCSV(path string, data []trajPoses) error {
	file, err := os.Create(expandPath(path))
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	header := []string{"t(s)", "x", "y", "z", "ox", "oy", "oz", "theta"}
	if err := writer.Write(header); err != nil {
		return err
	}

	for _, p := range data {
		ov := p.Pose.Orientation().OrientationVectorDegrees()
		record := []string{
			fmt.Sprintf("%.6f", p.Timestamp),
			fmt.Sprintf("%.6f", p.Pose.Point().X),
			fmt.Sprintf("%.6f", p.Pose.Point().Y),
			fmt.Sprintf("%.6f", p.Pose.Point().Z),
			fmt.Sprintf("%.6f", ov.OX),
			fmt.Sprintf("%.6f", ov.OY),
			fmt.Sprintf("%.6f", ov.OZ),
			fmt.Sprintf("%.6f", ov.Theta),
		}
		if err := writer.Write(record); err != nil {
			return err
		}
	}
	return nil
}

func writeWaypointPosesCSV(path string, poses []spatialmath.Pose) error {
	file, err := os.Create(expandPath(path))
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	header := []string{"x", "y", "z", "ox", "oy", "oz", "theta"}
	if err := writer.Write(header); err != nil {
		return err
	}

	for _, p := range poses {
		ov := p.Orientation().OrientationVectorDegrees()
		record := []string{
			fmt.Sprintf("%.6f", p.Point().X),
			fmt.Sprintf("%.6f", p.Point().Y),
			fmt.Sprintf("%.6f", p.Point().Z),
			fmt.Sprintf("%.6f", ov.OX),
			fmt.Sprintf("%.6f", ov.OY),
			fmt.Sprintf("%.6f", ov.OZ),
			fmt.Sprintf("%.6f", ov.Theta),
		}
		if err := writer.Write(record); err != nil {
			return err
		}
	}
	return nil
}
