package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"strconv"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"

	homedir "github.com/mitchellh/go-homedir"
	"github.com/rocketlaunchr/dataframe-go"
	"github.com/rocketlaunchr/dataframe-go/imports"
	chart "github.com/wcharczuk/go-chart"
)

type config struct {
	TrajectoryCSV     string `json:"trajectory_csv"`
	WaypointsCSV      string `json:"waypoints_csv"`
	ArmKinematicsPath string `json:"arm_kinematics_path"`
}

func loadConfig(path string) (config, error) {
	var cfg config
	file, err := os.Open(path)
	if err != nil {
		return cfg, err
	}
	defer file.Close()
	decoder := json.NewDecoder(file)
	if err := decoder.Decode(&cfg); err != nil {
		return cfg, err
	}
	return cfg, nil
}

func getPath(trajectoryPath, waypointPath string) (string, string, error) {
	if trajectoryPath == "" || waypointPath == "" {
		return "", "", fmt.Errorf("empty path passed in")
	}
	trajPath, err := homedir.Expand(trajectoryPath)
	if err != nil {
		return "", "", nil
	}
	wpPath, err := homedir.Expand(waypointPath)
	if err != nil {
		return "", "", nil
	}
	return trajPath, wpPath, nil
}

func readCSVintoDataframe(path string, headers []string) (*dataframe.DataFrame, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	df, err := imports.LoadFromCSV(context.Background(), file, imports.CSVLoadOptions{
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
	return df, err
}

func parseCSVandAddPoses(df *dataframe.DataFrame, model referenceframe.Model) (*dataframe.DataFrame, error) {
	xSeries := dataframe.NewSeriesFloat64("x", nil)
	ySeries := dataframe.NewSeriesFloat64("y", nil)
	zSeries := dataframe.NewSeriesFloat64("z", nil)
	oxSeries := dataframe.NewSeriesFloat64("ox", nil)
	oySeries := dataframe.NewSeriesFloat64("oy", nil)
	ozSeries := dataframe.NewSeriesFloat64("oz", nil)
	thetaSeries := dataframe.NewSeriesFloat64("theta", nil)

	for i := 0; i < df.NRows(); i++ {
		row := df.Row(i, false)
		rowInputs := []float64{}
		for j := 0; j < 6; j++ {
			inputValue, ok := row["j"+strconv.Itoa(j)].(float64)
			if !ok {
				return nil, fmt.Errorf("j%d not found in row", j)
			}
			rowInputs = append(rowInputs, inputValue)
		}
		pose, err := model.Transform(referenceframe.FloatsToInputs(rowInputs))
		if err != nil {
			return nil, err
		}
		xSeries.Append(pose.Point().X)
		ySeries.Append(pose.Point().Y)
		zSeries.Append(pose.Point().Z)
		oxSeries.Append(pose.Orientation().OrientationVectorDegrees().OX)
		oySeries.Append(pose.Orientation().OrientationVectorDegrees().OY)
		ozSeries.Append(pose.Orientation().OrientationVectorDegrees().OZ)
		thetaSeries.Append(pose.Orientation().OrientationVectorDegrees().Theta)
	}

	df.AddSeries(xSeries, nil)
	df.AddSeries(ySeries, nil)
	df.AddSeries(zSeries, nil)
	df.AddSeries(oxSeries, nil)
	df.AddSeries(oySeries, nil)
	df.AddSeries(ozSeries, nil)
	df.AddSeries(thetaSeries, nil)
	return df, nil
}

func extractFloatSeries(df *dataframe.DataFrame, colName string) ([]float64, error) {
	var col dataframe.Series
	for _, s := range df.Series {
		if s.Name() == colName {
			col = s
			break
		}
	}
	if col == nil {
		return nil, fmt.Errorf("column %s not found", colName)
	}
	values := make([]float64, col.NRows())
	for i := 0; i < col.NRows(); i++ {
		val := col.Value(i)
		if val == nil {
			values[i] = 0
			continue
		}
		f, ok := val.(float64)
		if !ok {
			return nil, fmt.Errorf("column %s: value at index %d is not float64", colName, i)
		}
		values[i] = f
	}
	return values, nil
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

func saveChartPNG(name, yLabel string, x1, y1, x2, y2 []float64) error {
	traj := chart.ContinuousSeries{Name: "Trajectory", XValues: x1, YValues: y1, Style: chart.Style{Show: true, StrokeColor: chart.ColorBlue}}
	way := chart.ContinuousSeries{Name: "Waypoints", XValues: x2, YValues: y2, Style: chart.Style{Show: true, StrokeColor: chart.ColorOrange}}
	graph := chart.Chart{
		XAxis:    chart.XAxis{Name: "Time (s)", NameStyle: chart.StyleShow(), Style: chart.StyleShow()},
		YAxis:    chart.YAxis{Name: yLabel, NameStyle: chart.StyleShow(), Style: chart.StyleShow()},
		Series:   []chart.Series{traj, way},
		Elements: []chart.Renderable{chart.Legend(&chart.Chart{Series: []chart.Series{traj, way}})},
	}
	f, err := os.Create(name)
	if err != nil {
		return err
	}
	defer f.Close()
	return graph.Render(chart.PNG, f)
}

func plotCharts(prefix, yLabelFormat string, trajT []float64, traj [][]float64, wayT []float64, way [][]float64, isPos bool) error {
	if len(traj) != len(way) {
		return fmt.Errorf("series count mismatch")
	}
	suffix := []string{"X", "Y", "Z"}
	for i := range traj {
		name := fmt.Sprintf("%s%d_comparison.png", prefix, i)
		if isPos {
			name = fmt.Sprintf("%s%s_comparison.png", prefix, suffix[i])
		}
		yLabel := fmt.Sprintf(yLabelFormat, i)
		if err := saveChartPNG(name, yLabel, trajT, traj[i], wayT, way[i]); err != nil {
			return err
		}
	}
	return nil
}

func plotJointComparisonFromDataframes(trajDf, waypointDf *dataframe.DataFrame) error {
	jointCount := 6
	trajT, err := extractFloatSeries(trajDf, "t(s)")
	if err != nil {
		return err
	}
	totalTime := trajT[len(trajT)-1]
	wayT := evenlySpacedTimes(trajT[0], totalTime, waypointDf.NRows())

	traj := make([][]float64, jointCount)
	way := make([][]float64, jointCount)
	for j := 0; j < jointCount; j++ {
		tn, err := extractFloatSeries(trajDf, fmt.Sprintf("j%d", j))
		if err != nil {
			return err
		}
		wn, err := extractFloatSeries(waypointDf, fmt.Sprintf("j%d", j))
		if err != nil {
			return err
		}
		traj[j] = tn
		way[j] = wn
	}
	return plotCharts("joint", "Joint %d Angle (rad)", trajT, traj, wayT, way, false)
}

func plotPoseComparisonFromDataframes(trajDf, waypointDf *dataframe.DataFrame) error {
	trajT, err := extractFloatSeries(trajDf, "t(s)")
	if err != nil {
		return err
	}
	start, end := trajT[0], trajT[len(trajT)-1]
	wayT := evenlySpacedTimes(start, end, waypointDf.NRows())

	tx, _ := extractFloatSeries(trajDf, "x")
	ty, _ := extractFloatSeries(trajDf, "y")
	tz, _ := extractFloatSeries(trajDf, "z")
	wx, _ := extractFloatSeries(waypointDf, "x")
	wy, _ := extractFloatSeries(waypointDf, "y")
	wz, _ := extractFloatSeries(waypointDf, "z")

	traj := [][]float64{tx, ty, tz}
	way := [][]float64{wx, wy, wz}
	return plotCharts("position_", "Position %c (mm)", trajT, traj, wayT, way, true)
}

func main() {
	logger := logging.NewLogger("graph")
	cfg, err := loadConfig("config.json")
	if err != nil {
		logger.Fatal(err)
	}
	trajectoryPath, waypointPath, err := getPath(cfg.TrajectoryCSV, cfg.WaypointsCSV)
	if err := realmain(trajectoryPath, waypointPath, cfg.ArmKinematicsPath); err != nil {
		logger.Fatal(err)
	}
}

func realmain(trajectoryPath, waypointPath, armKinematicsPath string) error {
	model, err := referenceframe.ParseModelJSONFile(armKinematicsPath, "")
	if err != nil {
		return err
	}

	trajDf, err := readCSVintoDataframe(trajectoryPath, nil)
	if err != nil {
		return err
	}
	wayDf, err := readCSVintoDataframe(waypointPath, []string{"j0", "j1", "j2", "j3", "j4", "j5"})
	if err != nil {
		return err
	}

	trajDf, err = parseCSVandAddPoses(trajDf, model)
	if err != nil {
		return err
	}
	wayDf, err = parseCSVandAddPoses(wayDf, model)
	if err != nil {
		return err
	}

	if err := plotJointComparisonFromDataframes(trajDf, wayDf); err != nil {
		return err
	}
	if err := plotPoseComparisonFromDataframes(trajDf, wayDf); err != nil {
		return err
	}
	return nil
}
