package main

import (
	"fmt"
	"math"
	"os"
	"os/signal"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/sensor_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

type fgm struct {
	BUBBLE_RADIUS            int
	PREPROCESS_CONV_SIZE     int // PREPROCESS_consecutive_SIZE
	BEST_POINT_CONV_SIZE     int
	MAX_LIDAR_DIST           int
	STRAIGHTS_STEERING_ANGLE float64 // 10 degrees

	robot_scale      float64
	radians_per_elem float64
	STRAIGHTS_SPEED  float64
	CORNERS_SPEED    float64

	lsMessages chan *sensor_msgs.LaserScan

	pub *goroslib.Publisher
}

func new_fgm() *fgm {
	f := fgm{}
	f.BUBBLE_RADIUS = 160
	f.PREPROCESS_CONV_SIZE = 100 // PREPROCESS_consecutive_SIZE
	f.BEST_POINT_CONV_SIZE = 80
	f.MAX_LIDAR_DIST = 3000000
	f.STRAIGHTS_STEERING_ANGLE = math.Pi / 18

	f.robot_scale = 0.3302
	f.radians_per_elem = 0
	f.STRAIGHTS_SPEED = 6.0
	f.CORNERS_SPEED = 2.0
	return &f
}

func main() {
	f := new_fgm()

	fgm, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "fgm",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer fgm.Close()

	f.lsMessages = make(chan *sensor_msgs.LaserScan, 10)

	lssub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:  fgm,
		Topic: "/scan",
		Callback: func(msg *sensor_msgs.LaserScan) {
			f.lsMessages <- msg
		},
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	defer lssub.Close()

	f.pub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  fgm,
		Topic: "FGM_CR",
		Msg:   &std_msgs.Float32MultiArray{},
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	defer f.pub.Close()

	f.driving()

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	<-c
}
func (f *fgm) driving() {
	// ticker := time.NewTicker(1000)
	// for msg := range f.lsMessages {
	// 	select {
	// 	case <-ticker.C:
	// 		proc_ranges := f.subCallback_scan(msg)
	// 		closest := f.argMin(proc_ranges)
	// 		min_index := closest - f.BUBBLE_RADIUS
	// 		max_index := closest + f.BUBBLE_RADIUS

	// 		if min_index < 0 {
	// 			min_index = 0
	// 		}
	// 		if max_index >= len(proc_ranges) {
	// 			max_index = len(proc_ranges) - 1
	// 		}
	// 		proc_ranges = f.setRangeToZero(proc_ranges, min_index, max_index)
	// 		gap_start, gap_end := f.findMaxGap(proc_ranges)
	// 		best := f.findBestPoint(gap_start, gap_end, proc_ranges)
	// 		steering_angle := f.getAngle(best, len(proc_ranges))

	// 		speed := 0.0
	// 		if math.Abs(steering_angle) > f.STRAIGHTS_STEERING_ANGLE {
	// 			speed = f.CORNERS_SPEED
	// 		} else {
	// 			speed = f.STRAIGHTS_SPEED
	// 		}
	// 		msg := &std_msgs.Float32MultiArray{
	// 			Data: []float32{float32(steering_angle), float32(speed)},
	// 		}
	// 		// f.ackermann_data.Drive.SteeringAngle = float32(steering_angle)
	// 		// f.ackermann_data.Drive.Speed = float32(speed)

	// 		f.pub.Write(msg)
	// 		fmt.Println(steering_angle, speed)

	// 	}
	// }
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	x_now := time.Now()
	x := time.Since(x_now)
	eps := time.Millisecond * 10

init:
	x = time.Since(x_now)
	select {
	// publish a message every second
	case <-time.After(100*time.Millisecond - x - eps):
		var proc_ranges []float64
		for msg := range f.lsMessages {
			proc_ranges = f.subCallback_scan(msg)
		}
		closest := f.argMin(proc_ranges)
		min_index := closest - f.BUBBLE_RADIUS
		max_index := closest + f.BUBBLE_RADIUS

		if min_index < 0 {
			min_index = 0
		}
		if max_index >= len(proc_ranges) {
			max_index = len(proc_ranges) - 1
		}
		proc_ranges = f.setRangeToZero(proc_ranges, min_index, max_index)
		gap_start, gap_end := f.findMaxGap(proc_ranges)
		best := f.findBestPoint(gap_start, gap_end, proc_ranges)
		steering_angle := f.getAngle(best, len(proc_ranges))

		speed := 0.0
		if math.Abs(steering_angle) > f.STRAIGHTS_STEERING_ANGLE {
			speed = f.CORNERS_SPEED
		} else {
			speed = f.STRAIGHTS_SPEED
		}
		msg := &std_msgs.Float32MultiArray{
			Data: []float32{float32(steering_angle), float32(speed)},
		}
		// f.ackermann_data.Drive.SteeringAngle = float32(steering_angle)
		// f.ackermann_data.Drive.Speed = float32(speed)

		f.pub.Write(msg)
		goto init
	case <-c:
		return
	case <-time.After(100*time.Millisecond - x):
		goto exp
	}
exp:
	fmt.Println("exp location")
}

func (f *fgm) setRangeToZero(procRanges []float64, minIndex, maxIndex int) []float64 {
	if minIndex >= 0 && maxIndex < len(procRanges) && minIndex <= maxIndex {
		for i := minIndex; i <= maxIndex; i++ {
			procRanges[i] = 0
		}
	}
	return procRanges
}
func (f *fgm) subCallback_scan(msg_sub *sensor_msgs.LaserScan) []float64 {
	ranges := msg_sub.Ranges

	f.radians_per_elem = (2 * math.Pi) / float64(len(ranges))
	procRanges := ranges[180 : len(ranges)-180]

	preprocessConvSize := 5 // 임의의 값을 설정해 주세요
	var convKernel []float64
	for i := 0; i < preprocessConvSize; i++ {
		convKernel = append(convKernel, 1.0)
	}

	var convResult []float64
	for i := range procRanges {
		var sum float64
		for j := range convKernel {
			idx := i - preprocessConvSize/2 + j
			if idx < 0 || idx >= len(procRanges) {
				continue
			}
			sum += float64(procRanges[idx])
		}
		convResult = append(convResult, sum/float64(preprocessConvSize))
	}

	maxLidarDist := 100.0 // 임의의 값을 설정해 주세요
	for i := range convResult {
		if convResult[i] < 0 {
			convResult[i] = 0
		} else if convResult[i] > maxLidarDist {
			convResult[i] = maxLidarDist
		}
	}
	return convResult
}
func (f *fgm) argMin(arr []float64) int {
	if len(arr) == 0 {
		return -1 // 배열이 비어있을 경우 -1을 반환합니다.
	}

	minIndex := 0
	minValue := arr[0]

	for i := 1; i < len(arr); i++ {
		if arr[i] < minValue {
			minIndex = i
			minValue = arr[i]
		}
	}

	return minIndex
}

func (f *fgm) findMaxGap(freeSpaceRanges []float64) (int, int) {
	// 마스킹 처리
	masked := make([]float64, len(freeSpaceRanges))
	for i, val := range freeSpaceRanges {
		if val == 0 {
			masked[i] = math.NaN()
		} else {
			masked[i] = val
		}
	}
	// 구간 찾기
	var slices [][]int
	start := -1
	for i, val := range masked {
		if !math.IsNaN(val) {
			if start == -1 {
				start = i
			}
		} else {
			if start != -1 {
				slices = append(slices, []int{start, i})
				start = -1
			}
		}
	}
	if start != -1 {
		slices = append(slices, []int{start, len(masked)})
	}
	// 최대 구간 찾기
	maxLen := slices[0][1] - slices[0][0]
	chosenSlice := slices[0]
	if len(slices) > 1 {
		for _, sl := range slices[1:] {
			slLen := sl[1] - sl[0]
			if slLen > maxLen {
				maxLen = slLen
				chosenSlice = sl
			}
		}
	}

	return chosenSlice[0], chosenSlice[1]
}
func (f *fgm) findBestPoint(startI, endI int, ranges []float64) int {
	bestPointConvSize := 5 // 임의의 값을 설정해 주세요
	var averagedMaxGap []float64
	for i := startI; i < endI; i++ {
		var sum float64
		for j := -bestPointConvSize / 2; j <= bestPointConvSize/2; j++ {
			idx := i + j
			if idx < 0 || idx >= len(ranges) {
				continue
			}
			sum += ranges[idx]
		}
		averagedMaxGap = append(averagedMaxGap, sum/float64(bestPointConvSize))
	}

	maxIndex := 0
	maxValue := averagedMaxGap[0]
	for i := 1; i < len(averagedMaxGap); i++ {
		if averagedMaxGap[i] > maxValue {
			maxIndex = i
			maxValue = averagedMaxGap[i]
		}
	}

	return maxIndex + startI
}

func (f *fgm) getAngle(rangeIndex, rangeLen int) float64 {
	lidarAngle := (float64(rangeIndex) - (float64(rangeLen) / 2)) * f.radians_per_elem
	steeringAngle := lidarAngle / 2

	return steeringAngle
}
