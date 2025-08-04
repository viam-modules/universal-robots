package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strconv"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"

	homedir "github.com/mitchellh/go-homedir"
	"github.com/rocketlaunchr/dataframe-go"
	"github.com/rocketlaunchr/dataframe-go/imports"

	// dfmath "github.com/rocketlaunchr/dataframe-go/math"
	chart "github.com/wcharczuk/go-chart"
)

const jointCount = 6

type config struct {
	TrajectoryCSV     string `json:"trajectory_csv"`
	WaypointsCSV      string `json:"waypoints_csv"`
	ArmKinematicsPath string `json:"arm_kinematics_path"`
	CachedPlanPath    string `json:"cached_plan_path"`
	CachedOnly        bool   `json:"cached_only"`
}

func loadConfig(path string) (config, error) {
	var cfg config
	file, err := os.Open(path)
	if err != nil {
		return config{}, err
	}
	defer file.Close()
	decoder := json.NewDecoder(file)
	if err := decoder.Decode(&cfg); err != nil {
		return config{}, err
	}
	return cfg, nil
}

func getPath(trajectoryPath, waypointPath string) (string, string, error) {
	if trajectoryPath == "" || waypointPath == "" {
		return "", "", fmt.Errorf("empty path passed in")
	}
	trajPath, err := homedir.Expand(trajectoryPath)
	if err != nil {
		return "", "", err
	}
	wpPath, err := homedir.Expand(waypointPath)
	if err != nil {
		return "", "", err
	}
	return trajPath, wpPath, nil
}

func readCSVintoDataframe(path string, headers []string) (*dataframe.DataFrame, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	return imports.LoadFromCSV(context.Background(), file, imports.CSVLoadOptions{
		Headers: headers,
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
}

func parseAndAddPoses(df *dataframe.DataFrame, model referenceframe.Model) (*dataframe.DataFrame, error) {
	// we need to create series for poses which is a 7 dimensional
	xSeries := dataframe.NewSeriesFloat64("x", nil)
	ySeries := dataframe.NewSeriesFloat64("y", nil)
	zSeries := dataframe.NewSeriesFloat64("z", nil)
	oxSeries := dataframe.NewSeriesFloat64("ox", nil)
	oySeries := dataframe.NewSeriesFloat64("oy", nil)
	ozSeries := dataframe.NewSeriesFloat64("oz", nil)
	thetaSeries := dataframe.NewSeriesFloat64("theta", nil)

	for i := range df.NRows() {
		row := df.Row(i, false)
		// iterate through the row and construct the joint positions which are 6 dimensional
		rowInputs := dataframe.NewSeriesFloat64("joints", &dataframe.SeriesInit{})
		for j := range jointCount {
			rowInputs.Append(row["j"+strconv.Itoa(j)])
		}
		// sanitize Nans
		for _, v := range rowInputs.Values {
			if !dataframe.IsValidFloat64(v) {
				return nil, fmt.Errorf("rowInputs values contained a NaN")
			}
		}
		// perform FK to get the pose
		pose, err := model.Transform(referenceframe.FloatsToInputs(rowInputs.Values))
		if err != nil {
			return nil, err
		}
		// add the pose to the series
		xSeries.Append(pose.Point().X)
		ySeries.Append(pose.Point().Y)
		zSeries.Append(pose.Point().Z)
		oxSeries.Append(pose.Orientation().OrientationVectorDegrees().OX)
		oySeries.Append(pose.Orientation().OrientationVectorDegrees().OY)
		ozSeries.Append(pose.Orientation().OrientationVectorDegrees().OZ)
		thetaSeries.Append(pose.Orientation().OrientationVectorDegrees().Theta)
	}

	// add the series to the dataframe
	if err := df.AddSeries(xSeries, nil); err != nil {
		return nil, err
	}
	if err := df.AddSeries(ySeries, nil); err != nil {
		return nil, err
	}
	if err := df.AddSeries(zSeries, nil); err != nil {
		return nil, err
	}
	if err := df.AddSeries(oxSeries, nil); err != nil {
		return nil, err
	}
	if err := df.AddSeries(oySeries, nil); err != nil {
		return nil, err
	}
	if err := df.AddSeries(ozSeries, nil); err != nil {
		return nil, err
	}
	if err := df.AddSeries(thetaSeries, nil); err != nil {
		return nil, err
	}
	return df, nil
}

func extractFloatSeries(df *dataframe.DataFrame, colName string) ([]float64, error) {
	for _, s := range df.Series {
		if s.Name() == colName {
			if floatSeries, ok := s.(*dataframe.SeriesFloat64); ok {
				return floatSeries.Values, nil
			}
			return nil, fmt.Errorf("column %s is not SeriesFloat64", colName)
		}
	}
	return nil, fmt.Errorf("column %s not found", colName)
}

func evenlySpacedTimes(start, end float64, count int) []float64 {
	times := make([]float64, count)
	denom := float64(1)
	if count > 1 {
		denom = float64(count - 1)
	}
	for i := 0; i < count; i++ {
		times[i] = start + (end-start)*float64(i)/denom
	}
	return times
}

func saveComparisonChartPNG(name, yLabel string, x1, y1, x2, y2 []float64) error {
	traj := chart.ContinuousSeries{Name: "Trajectory", XValues: x1, YValues: y1, Style: chart.Style{Show: true, StrokeColor: chart.ColorBlue}}
	way := chart.ContinuousSeries{Name: "Waypoints", XValues: x2, YValues: y2, Style: chart.Style{Show: true, StrokeColor: chart.ColorOrange}}

	// Create title with legend information
	titleWithLegend := fmt.Sprintf("%s (Blue: Trajectory, Orange: Waypoints)", name)

	graph := chart.Chart{
		Title:  titleWithLegend,
		XAxis:  chart.XAxis{Name: "Time (s)", NameStyle: chart.StyleShow(), Style: chart.StyleShow()},
		YAxis:  chart.YAxis{Name: yLabel, NameStyle: chart.StyleShow(), Style: chart.StyleShow()},
		Series: []chart.Series{traj, way},
		Height: 600,
		Elements: []chart.Renderable{chart.LegendThin(
			&chart.Chart{Series: []chart.Series{traj, way}},
			chart.Style{FillColor: chart.ColorTransparent,
				StrokeColor:     chart.ColorTransparent,
				TextLineSpacing: 5,
			}),
		},
	}
	f, err := os.Create(name)
	if err != nil {
		return err
	}
	defer f.Close()
	return graph.Render(chart.PNG, f)
}
func saveChartPNG(name, yLabel string, x1, y1 []float64) error {
	way := chart.ContinuousSeries{Name: "Waypoints", XValues: x1, YValues: y1, Style: chart.Style{Show: true, StrokeColor: chart.ColorOrange}}

	// Create title with legend information
	titleWithLegend := fmt.Sprintf("%s (Orange: Waypoints)", name)

	graph := chart.Chart{
		Title:  titleWithLegend,
		XAxis:  chart.XAxis{Name: "Time (s)", NameStyle: chart.StyleShow(), Style: chart.StyleShow()},
		YAxis:  chart.YAxis{Name: yLabel, NameStyle: chart.StyleShow(), Style: chart.StyleShow()},
		Series: []chart.Series{way},
		Height: 600,
		Elements: []chart.Renderable{chart.LegendThin(
			&chart.Chart{Series: []chart.Series{way}},
			chart.Style{FillColor: chart.ColorTransparent,
				StrokeColor:     chart.ColorTransparent,
				TextLineSpacing: 5,
			}),
		},
	}
	f, err := os.Create(name)
	if err != nil {
		return err
	}
	defer f.Close()
	return graph.Render(chart.PNG, f)
}

func plotCharts(
	prefix, yLabelFormat string,
	trajTime, wayTime []float64,
	trajData, waypointData [][]float64,
	labelFunc func(int) string,
) error {
	if len(trajData) != len(waypointData) {
		return fmt.Errorf("series count mismatch")
	}
	for i := range trajData {
		name := fmt.Sprintf("%s%s_comparison.png", prefix, labelFunc(i))
		yLabel := fmt.Sprintf(yLabelFormat, labelFunc(i))
		if err := saveComparisonChartPNG(name, yLabel, trajTime, trajData[i], wayTime, waypointData[i]); err != nil {
			return err
		}
	}
	return nil
}

func plotChartsCachedData(
	prefix, yLabelFormat string,
	wayTime []float64,
	waypointData [][]float64,
	labelFunc func(int) string,
) error {
	fmt.Println("waytime: ", len(wayTime))

	for i := range waypointData {
		name := fmt.Sprintf("%s%s_cached.png", prefix, labelFunc(i))
		yLabel := fmt.Sprintf(yLabelFormat, labelFunc(i))
		fmt.Println(name, ": ", len(waypointData[i]))
		if err := saveChartPNG(name, yLabel, wayTime, waypointData[i]); err != nil {
			return err
		}
	}
	return nil
}

func plotJointAndPoseComparisonFromDataframes(trajDf, waypointDf *dataframe.DataFrame) error {
	// get the time values which will determine the spacing of trajectory and waypoint data
	trajTime, err := extractFloatSeries(trajDf, "t(s)")
	if err != nil {
		return err
	}
	startTime := trajTime[0]
	totalTime := trajTime[len(trajTime)-1]
	waypointTime := evenlySpacedTimes(startTime, totalTime, waypointDf.NRows())

	// get the joint positions data
	allTrajectoryJointValues := make([][]float64, jointCount)
	allWaypointJointValues := make([][]float64, jointCount)
	for i := range jointCount {
		trajJointValues, err := extractFloatSeries(trajDf, fmt.Sprintf("j%d", i))
		if err != nil {
			return err
		}
		waypointJointValues, err := extractFloatSeries(waypointDf, fmt.Sprintf("j%d", i))
		if err != nil {
			return err
		}
		allTrajectoryJointValues[i] = trajJointValues
		allWaypointJointValues[i] = waypointJointValues
	}

	// get the pose data
	tx, err := extractFloatSeries(trajDf, "x")
	if err != nil {
		return err
	}
	ty, err := extractFloatSeries(trajDf, "y")
	if err != nil {
		return err
	}
	tz, err := extractFloatSeries(trajDf, "z")
	if err != nil {
		return err
	}
	wx, err := extractFloatSeries(waypointDf, "x")
	if err != nil {
		return err
	}
	wy, err := extractFloatSeries(waypointDf, "y")
	if err != nil {
		return err
	}
	wz, err := extractFloatSeries(waypointDf, "z")
	if err != nil {
		return err
	}
	trajectoryPoses := [][]float64{tx, ty, tz}
	waypointPoses := [][]float64{wx, wy, wz}

	// For joints, just use the index number as string
	if err := plotCharts("joint", "Joint %s Angle (rad)", trajTime, waypointTime, allTrajectoryJointValues, allWaypointJointValues, func(i int) string {
		return strconv.Itoa(i)
	}); err != nil {
		return err
	}

	// For positions, use letters X, Y, Z
	suffix := []string{"X", "Y", "Z"}
	if err := plotCharts("position_", "Position %s (mm)", trajTime, waypointTime, trajectoryPoses, waypointPoses, func(i int) string {
		return suffix[i]
	}); err != nil {
		return err
	}

	return nil
}

func main() {
	logger := logging.NewLogger("graph")
	cfg, err := loadConfig("config.json")
	if err != nil {
		logger.Fatal(err)
	}
	if err := realmain(cfg); err != nil {
		logger.Fatal(err)
	}
}

func realmain(cfg config) error {
	model, err := referenceframe.ParseModelJSONFile(cfg.ArmKinematicsPath, "")
	if err != nil {
		return err
	}
	if !cfg.CachedOnly {
		trajectoryPath, waypointPath, err := getPath(cfg.TrajectoryCSV, cfg.WaypointsCSV)
		if err != nil {
			return err
		}

		// get the data frame
		trajDf, err := readCSVintoDataframe(trajectoryPath, nil)
		if err != nil {
			return err
		}
		wayDf, err := readCSVintoDataframe(waypointPath, []string{"j0", "j1", "j2", "j3", "j4", "j5"})
		if err != nil {
			return err
		}
		// parse and perform FK to get poses for each set of joint positions
		trajDf, err = parseAndAddPoses(trajDf, model)
		if err != nil {
			return err
		}
		wayDf, err = parseAndAddPoses(wayDf, model)
		if err != nil {
			return err
		}
		// plot joint positions and poses as .png files
		if err := plotJointAndPoseComparisonFromDataframes(trajDf, wayDf); err != nil {
			return err
		}
	}
	cachedPath := filepath.Clean(cfg.CachedPlanPath)
	if cachedPath != "" {
		joints, poses, err := parseAndAddPosesCachedPlan(cachedPath, model)
		if err != nil {
			return err
		}
		err = plotJointsAndPosesCachedPlan(joints, poses)
		if err != nil {
			return err
		}
	}

	return nil
}

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
