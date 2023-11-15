package main

import (
	"fmt"
	"math"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/sensor_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

const ctimemin int = 1
const ctimemax int = 3
const peridod int = 25

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

	file_exec, err := os.Create("fgm_execute.txt") // hello.txt 파일 생성
	if err != nil {
		fmt.Println(err)
		return
	}
	defer file_exec.Close() // main 함수가 끝나기 직전에 파일을 닫음

	file_period, err := os.Create("fgm_period.txt") // hello.txt 파일 생성
	if err != nil {
		fmt.Println(err)
		return
	}
	defer file_period.Close() // main 함수가 끝나기 직전에 파일을 닫음

	cc := make(chan os.Signal, 1)
	signal.Notify(cc, os.Interrupt)

	c_now := time.Now()
	c := time.Since(c_now)

	t_now := time.Now()
	t := time.Since(t_now)

	p := fgm.TimeRate(time.Duration(peridod) * time.Millisecond)

	fmsg := &std_msgs.Float32MultiArray{
		Data: []float32{float32(0), float32(0)},
	}
	eps := time.Millisecond * 10
	var processing_passage []string
	var wait_passage []string
	goto init

init:
	goto ready
ready:
	c_now = time.Now()

	msg := <-f.lsMessages
	//fmt.Println("aa")
	proc_ranges := f.subCallback_scan(msg)
	closest := f.argMin(proc_ranges)
	min_index := closest - f.BUBBLE_RADIUS
	max_index := closest + f.BUBBLE_RADIUS
	//fmt.Println("aa")
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
	fmsg = &std_msgs.Float32MultiArray{
		Data: []float32{float32(steering_angle), float32(speed)},
	}
	//
	goto processing

processing:
	c = time.Since(c_now)
	processing_passage = []string{"c==ctimemin", "c>ctimemin", "c>ctimemax"}

	switch time_passage(processing_passage, c) {
	case 0:
		goto processing_1
	case 1:
		goto processing_2
	case 2:
		goto processing_3
	case 3:

		goto exp
	}
processing_1:

	c = time.Since(c_now)
	select {
	// publish a message every second
	case <-time.After(time.Duration(ctimemin)*time.Millisecond - c - eps):
		goto processing_2
	case <-cc:
		return
	}
processing_2:
	c = time.Since(c_now)
	select {
	// publish a message every second
	case <-time.After(time.Duration(ctimemin)*time.Millisecond - c):
		goto processing_3
	case <-time.After(0 * time.Millisecond):
		goto mid
	case <-cc:
		return
	}

processing_3:
	fmt.Println("pro3", c)
	c = time.Since(c_now)

	select {
	// publish a message every second
	case <-time.After(time.Duration(ctimemax)*time.Millisecond - c):
		_, err = file_exec.Write([]byte(time.Duration.String(time.Now().Sub(c_now)))) // s를 []byte 바이트 슬라이스로 변환, s를 파일에 저장
		if err != nil {
			fmt.Println(err)
			return
		}
		goto exp
	case <-time.After(0 * time.Millisecond):
		goto mid
	case <-cc:
		return
	}
mid:
	//f.pub.Write(&f.ackermann)
	c = time.Since(c_now)

	_, err = file_exec.Write([]byte(time.Duration.String(time.Now().Sub(c_now)))) // s를 []byte 바이트 슬라이스로 변환, s를 파일에 저장
	if err != nil {
		fmt.Println(err)
		return
	}
	f.pub.Write(fmsg)
	goto wait
wait:
	t = time.Since(t_now)

	wait_passage = []string{"t==peridod", "x>peridod"}
	switch time_passage(wait_passage, t) {
	case 0:
		goto wait_1
	case 1:
		goto wait_2
	case 2:
		goto exp
	}
wait_1:
	t = time.Since(t_now)
	select {
	// publish a message every second
	case <-time.After(time.Duration(peridod)*time.Millisecond - t - eps):
		//case <-p.SleepChan():
		goto wait_2
	case <-cc:
		return
	}
wait_2:
	t = time.Since(t_now)
	select {
	// publish a message every second
	case <-time.After(time.Duration(peridod)*time.Millisecond - t):
		fmt.Println("exp", time.Now().Sub(t_now))
		_, err := file_period.Write([]byte(time.Duration.String(time.Now().Sub(t_now)))) // s를 []byte 바이트 슬라이스로 변환, s를 파일에 저장
		if err != nil {
			fmt.Println(err)
			return
		}
		goto exp
	//case <-time.After(0 * time.Millisecond):
	case <-p.SleepChan():
		fmt.Println("execute", time.Now().Sub(t_now))
		_, err := file_period.Write([]byte(time.Duration.String(time.Now().Sub(t_now)))) // s를 []byte 바이트 슬라이스로 변환, s를 파일에 저장
		if err != nil {
			fmt.Println(err)
			return
		}
		t_now = time.Now()
		goto ready
	case <-cc:
		return
	}
exp:
	<-p.SleepChan()
	t_now = time.Now()
	fmt.Println("exp loc")
	goto ready

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

func time_passage(time_passage []string, ctime time.Duration) int {
	for i, val := range time_passage { // 비교하는거 추가
		if strings.Contains(val, "==") {

			num, _ := strconv.Atoi(val[strings.Index(val, "==")+2:])
			if time.Millisecond*time.Duration(num) < ctime {
				//if time.Millisecond*time.Duration(num).After(ctime *time.Millisecond){
				return i
			}
		} else if strings.Contains(val, ">") {
			num, _ := strconv.Atoi(val[strings.Index(val, ">")+1:])
			if time.Millisecond*time.Duration(num) > ctime {
				//if time.Millisecond*time.Duration(num).Equal(ctime *time.Millisecond){
				return i
			}
		}
	}
	return len(time_passage)
}
