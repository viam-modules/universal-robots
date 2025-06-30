package main

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/rocketlaunchr/dataframe-go"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

type tempFile struct {
	name string
}

func (t *tempFile) Cleanup() {
	os.Remove(t.name)
}

func writeTempCSV(t *testing.T, content string) (string, func()) {
	file, err := os.CreateTemp("", "test_*.csv")
	test.That(t, err, test.ShouldBeNil)
	_, err = file.Write([]byte(content))
	test.That(t, err, test.ShouldBeNil)
	err = file.Close()
	test.That(t, err, test.ShouldBeNil)
	return file.Name(), func() { os.Remove(file.Name()) }
}

func TestLoadConfig(t *testing.T) {
	t.Run("proper json", func(t *testing.T) {
		cfgContent := `{
		"trajectory_csv": "~/traj.csv",
		"waypoints_csv": "~/wp.csv",
		"arm_kinematics_path": "~/ur5e.json"
	}`
		file := filepath.Join(os.TempDir(), "config_test.json")
		err := os.WriteFile(file, []byte(cfgContent), 0644)
		test.That(t, err, test.ShouldBeNil)
		defer os.Remove(file)

		cfg, err := loadConfig(file)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, cfg.WaypointsCSV, test.ShouldEqual, "~/wp.csv")
		test.That(t, cfg.TrajectoryCSV, test.ShouldEqual, "~/traj.csv")
		test.That(t, cfg.ArmKinematicsPath, test.ShouldEqual, "~/ur5e.json")
	})
	t.Run("improper json", func(t *testing.T) {
		badContent := `{
		"trajectory_csv": "~/traj.csv",
		"waypoints_csv": "~/wp.csv"  // <-- bad trailing comment not valid JSON
	`
		file := filepath.Join(os.TempDir(), "config_bad_test.json")
		err := os.WriteFile(file, []byte(badContent), 0644)
		test.That(t, err, test.ShouldBeNil)
		defer os.Remove(file)

		_, err = loadConfig(file)
		test.That(t, err, test.ShouldNotBeNil)
	})
	t.Run("missing file", func(t *testing.T) {
		_, err := loadConfig("/nonexistent/path/config.json")
		test.That(t, err, test.ShouldNotBeNil)
	})
}

func TestGetPath(t *testing.T) {
	t.Run("expanding with ~/", func(t *testing.T) {
		home, err := os.UserHomeDir()
		test.That(t, err, test.ShouldBeNil)
		traj, wp, err := getPath("~/test1.csv", "~/test2.csv")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, strings.HasPrefix(traj, home), test.ShouldBeTrue)
		test.That(t, strings.HasPrefix(wp, home), test.ShouldBeTrue)
	})
	t.Run("absolute path", func(t *testing.T) {
		_, _, err := getPath("/home/nick/file0.csv", "/home/nick/file2.csv")
		test.That(t, err, test.ShouldBeNil)
	})
	t.Run("invalid path", func(t *testing.T) {
		_, _, err := getPath("~nosuchdir/file.csv", "~/file.csv")
		test.That(t, err, test.ShouldNotBeNil)
	})
}

func TestReadCSVintoDataframe(t *testing.T) {
	t.Run("properly formatted content", func(t *testing.T) {
		csvContent := "t(s),j0,j1,j2,j3,j4,j5\n0.0,1,2,3,4,5,6\n1.0,2,3,4,5,6,7"
		fileName, cleanup := writeTempCSV(t, csvContent)
		defer cleanup()

		df, err := readCSVintoDataframe(fileName, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, df.NRows(), test.ShouldEqual, 2)
		test.That(t, len(df.Series), test.ShouldEqual, 7)
	})
	t.Run("improper content", func(t *testing.T) {
		csvContent := "badcol1,badcol2\n1,2"
		fileName, cleanup := writeTempCSV(t, csvContent)
		defer cleanup()

		_, err := readCSVintoDataframe(fileName, nil)
		test.That(t, err, test.ShouldBeNil) // DataFrame can still be created

		// But later extracting required columns should fail
		df, _ := readCSVintoDataframe(fileName, nil)
		_, err = extractFloatSeries(df, "j0")
		test.That(t, err, test.ShouldNotBeNil)
	})

}

func TestExtractFloatSeries(t *testing.T) {
	t.Run("success case -- test on floats", func(t *testing.T) {
		series := dataframe.NewSeriesFloat64("test", nil, 1.1, 2.2, 3.3)
		df := dataframe.NewDataFrame(series)
		vals, err := extractFloatSeries(df, "test")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vals, test.ShouldResemble, []float64{1.1, 2.2, 3.3})
	})

	t.Run("failure case -- test on non floats", func(t *testing.T) {
		series := dataframe.NewSeriesString("test", nil, "a", "b", "c")
		df := dataframe.NewDataFrame(series)
		_, err := extractFloatSeries(df, "test")
		test.That(t, err, test.ShouldNotBeNil)
	})
}

func TestEvenlySpacedTimes(t *testing.T) {
	times := evenlySpacedTimes(0.0, 2.0, 3)
	test.That(t, times, test.ShouldResemble, []float64{0.0, 1.0, 2.0})

	times = evenlySpacedTimes(1.0, 1.0, 1)
	test.That(t, times, test.ShouldResemble, []float64{1.0})
}

func TestSaveChartPNG(t *testing.T) {
	name := filepath.Join(os.TempDir(), "test_chart.png")
	err := saveChartPNG(name, "Y Axis", []float64{0, 1}, []float64{1, 2}, []float64{0, 1}, []float64{2, 3})
	test.That(t, err, test.ShouldBeNil)
	_, err = os.Stat(name)
	test.That(t, err, test.ShouldBeNil)
	defer os.Remove(name)
}

func TestParseAndAddPoses(t *testing.T) {
	t.Run("success case", func(t *testing.T) {
		csvContent := "t(s),j0,j1,j2,j3,j4,j5\n0.0,0,0,0,0,0,0"
		fileName, cleanup := writeTempCSV(t, csvContent)
		defer cleanup()

		df, err := readCSVintoDataframe(fileName, nil)
		test.That(t, err, test.ShouldBeNil)

		// model := referenceframe.NewSimpleModel("")
		model, err := referenceframe.ParseModelJSONFile("../../../kinematics/ur5e.json", "")
		test.That(t, err, test.ShouldBeNil)
		_, err = parseAndAddPoses(df, model)
		test.That(t, err, test.ShouldBeNil)
	})
	t.Run("sanitize NaNs", func(t *testing.T) {
		seriesList := []dataframe.Series{}

		// j0 is normal
		seriesList = append(seriesList, dataframe.NewSeriesFloat64("j0", nil, 1.0, 2.0, 3.0))

		// j1..j5 have NaNs
		for i := 1; i < 6; i++ {
			s := dataframe.NewSeriesFloat64(fmt.Sprintf("j%d", i), nil, math.NaN(), 2.0, 3.0)
			seriesList = append(seriesList, s)
		}

		df := dataframe.NewDataFrame(seriesList...)

		model, err := referenceframe.ParseModelJSONFile("../../../kinematics/ur5e.json", "")
		test.That(t, err, test.ShouldBeNil)
		_, err = parseAndAddPoses(df, model)
		test.That(t, err, test.ShouldNotBeNil)
	})

}
