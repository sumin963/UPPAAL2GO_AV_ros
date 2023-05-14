package main

import (
	"fmt"
	"math"
	"os"
	"os/signal"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/ackermann_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/nav_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/sensor_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

type obstacle_dect struct {
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
	odMessages chan *nav_msgs.Odometry

	ackermann_data ackermann_msgs.AckermannDriveStamped
	pub            *goroslib.Publisher
}

func new_obstacle_dect() *obstacle_dect {
	f := obstacle_dect{}
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
	f := new_obstacle_dect()

	od_node, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "od",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer od_node.Close()

	f.lsMessages = make(chan *sensor_msgs.LaserScan, 10)

	lssub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:  od_node,
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
		Node:  od_node,
		Topic: "OD_CR",
		Msg:   &std_msgs.Bool{},
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
func (f *obstacle_dect) driving() {
	ticker := time.NewTicker(1000)
	for msg := range f.lsMessages {
		select {
		case <-ticker.C:
			proc_ranges := f.subCallback_scan(msg)
			obstacle_bool := f.obstacleDetection(proc_ranges)
			fmt.Println(obstacle_bool)
			msg := &std_msgs.Bool{
				Data: obstacle_bool,
			}
			f.pub.Write(msg)
		}
	}
	fmt.Println("asdasf")
}
func (f *obstacle_dect) subCallback_scan(msg_sub *sensor_msgs.LaserScan) []float64 {
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

func (f *obstacle_dect) obstacleDetection(scanFiltered []float64) bool {
	scanObstacle := make([][]float64, 0)
	i := 1
	dGroup := 1.5
	dPi := 0.00628
	scanRange := len(scanFiltered)

	for scanRange-1 > i {
		tempStartIndex := i
		tempEndIndex := i
		tempMaxIndex := i
		tempMinIndex := i
		i++
		_radians_25 := (0.25) * (math.Pi / 180)

		for math.Sqrt(math.Pow(scanFiltered[i]*math.Sin(_radians_25), 2)+math.Pow(scanFiltered[i-1]-scanFiltered[i]*math.Cos(_radians_25), 2)) < dGroup+scanFiltered[i]*dPi && (i+1 < scanRange) {
			if scanFiltered[i] > scanFiltered[tempMaxIndex] {
				tempMaxIndex = i
			}
			if scanFiltered[i] < scanFiltered[tempMinIndex] {
				tempMinIndex = i
			}
			i++
		}

		tempEndIndex = i - 1
		scanObstacle = append(scanObstacle, []float64{float64(tempStartIndex), float64(tempEndIndex), float64(tempMaxIndex), scanFiltered[tempMaxIndex], scanFiltered[tempMinIndex]})
		i++
	}

	obstacleSplit := make([][]float64, 0)
	obstacleDetect := make([][]float64, 0)

	for i := 0; i < len(scanObstacle); i++ {
		if 4 > scanObstacle[i][3] && scanObstacle[i][3] > 0 {
			obstacleSplit = append(obstacleSplit, scanObstacle[i])
			obstacleDetect = append(obstacleDetect, scanObstacle[i])
		}
	}

	obstacleLength := make([][]float64, 0)
	for i := 0; i < len(obstacleDetect); i++ {
		theta := (obstacleDetect[i][1] - obstacleDetect[i][0]) * 0.25
		radians_theta := theta * (math.Pi / 180)
		length := math.Sqrt(math.Pow(scanFiltered[int(obstacleDetect[i][1])]*math.Sin(radians_theta), 2) + math.Pow(scanFiltered[int(obstacleDetect[i][0])]-scanFiltered[int(obstacleDetect[i][1])]*math.Cos(radians_theta), 2))

		if length < 1 {
			obstacleLength = append(obstacleLength, obstacleDetect[i])
		}
	}

	obstacle := false

	for i := 0; i < len(obstacleLength); i++ {
		if obstacleLength[i][0] > 680 || obstacleLength[i][1] < 400 {
			obstacle = false
		} else {
			fmt.Println("Obstacle Caught.")
			obstacle = true
			break
		}
	}

	return obstacle
}
