package main

import (
	"context"
	"encoding/csv"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"os"
	"strconv"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
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
	Viam struct {
		Host     string `json:"host"`
		EntityID string `json:"entity_id"`
		APIKey   string `json:"api_key"`
	} `json:"viam"`
	ArmName               string `json:"arm_name"`
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

func main() {
	logger := logging.NewDebugLogger("client")
	ctx := context.Background()

	cfg, err := loadConfig("config.json")
	if err != nil {
		log.Fatalf("failed to load config: %v", err)
	}

	machine, err := client.New(
		context.Background(),
		cfg.Viam.Host,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			cfg.Viam.EntityID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: cfg.Viam.APIKey,
			},
		)),
	)
	if err != nil {
		logger.Fatal(err)
	}
	defer machine.Close(ctx)

	ur, err := arm.FromRobot(machine, cfg.ArmName)
	if err != nil {
		logger.Error(err)
		return
	}

	trajInputs, err := readTrajectoryCSV(cfg.TrajectoryCSV)
	if err != nil {
		log.Fatalf("failed to read trajectory CSV: %v", err)
	}

	var trajectoryPoses []trajPoses
	for _, trajInput := range trajInputs {
		pose, err := ur.ModelFrame().Transform(trajInput.JointInputs)
		if err != nil {
			logger.Error(err)
			return
		}
		trajectoryPoses = append(trajectoryPoses, trajPoses{Timestamp: trajInput.Timestamp, Pose: pose})
	}

	waypoints, err := readWaypointCSV(cfg.WaypointsCSV)
	if err != nil {
		log.Fatalf("failed to read waypoint CSV: %v", err)
	}

	var waypointPoses []spatialmath.Pose
	for _, pos := range waypoints {
		pose, err := ur.ModelFrame().Transform(pos)
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
	file, err := os.Open(filepath)
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
	file, err := os.Open(filepath)
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
	file, err := os.Create(path)
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
	file, err := os.Create(path)
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
