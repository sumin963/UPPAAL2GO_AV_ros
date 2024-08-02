package main

import (
	"bufio"
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/nav_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

type Pure_Pursuit struct {
	LOOKAHEAD_MAX     float64
	LOOKAHEAD_MIN     float64
	SPEED_MAX         float64
	SPEED_MIN         float64
	MSC_MUXSIZE       int
	MU                float64
	RATE              float64
	WPS_FILE_LOCATION string

	CURRENT_WP_CHECK_OFFSET float64
	DX_GAIN                 float64
	RACECAR_LENGTH          float64
	GRAVITY_ACCELERATION    float64

	waypoints                 [][]float64
	wp_len                    int
	wp_index_current          int
	current_position          []float64
	lookahead_desired         float64
	steering_direction        float64
	goal_path_radius          float64
	goal_path_theta           float64
	actual_lookahead          float64
	transformed_desired_point []float64
	desired_point             []float64
	dx                        float64

	nearest_distance float64
	manualSpeedArray []float64
	wa               float64

	STRAIGHTS_SPEED          float64
	CORNERS_SPEED            float64
	STRAIGHTS_STEERING_ANGLE float64 // 10 degrees

	odMessages chan *nav_msgs.Odometry

	// pp_cr PP_CR
	pub *goroslib.Publisher
}

func new_Pure_Pursuit() *Pure_Pursuit {
	f := Pure_Pursuit{}

	f.LOOKAHEAD_MAX = 2.5
	f.LOOKAHEAD_MIN = 1.0
	f.SPEED_MAX = 4.0
	f.SPEED_MIN = 1.5
	f.MSC_MUXSIZE = 0
	f.MU = 1
	f.RATE = 100
	f.WPS_FILE_LOCATION = "/home/rtc/workspace_golang/src/map/SOCHI.csv"

	f.CURRENT_WP_CHECK_OFFSET = 2.0
	f.DX_GAIN = 2.5
	f.RACECAR_LENGTH = 0.325
	f.GRAVITY_ACCELERATION = 9.81

	f.waypoints = f.get_waypoint()
	f.wp_len = len(f.waypoints)
	f.wp_index_current = 0
	f.current_position = []float64{}
	f.lookahead_desired = 0
	f.steering_direction = 0
	f.goal_path_radius = 0
	f.goal_path_theta = 0
	f.actual_lookahead = 0
	f.transformed_desired_point = []float64{}
	f.desired_point = []float64{}
	f.dx = 0
	f.nearest_distance = 0
	f.manualSpeedArray = []float64{}
	f.wa = 0
	f.STRAIGHTS_SPEED = 6.0
	f.CORNERS_SPEED = 2.0
	f.STRAIGHTS_STEERING_ANGLE = math.Pi / 18

	return &f
}

func main() {
	f := new_Pure_Pursuit()

	pp_node, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "pure_pursuit",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer pp_node.Close()

	f.odMessages = make(chan *nav_msgs.Odometry, 10)

	odomsub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:  pp_node,
		Topic: "/odom",
		Callback: func(msg *nav_msgs.Odometry) {
			f.odMessages <- msg
		},
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	defer odomsub.Close()

	f.pub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  pp_node,
		Topic: "PP_CR",
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
func (f *Pure_Pursuit) driving() {
	ticker := time.NewTicker(1000)
	for msg := range f.odMessages {
		select {
		case <-ticker.C:
			f.Odome(msg)

			f.getDx()

			f.getLookaheadDesired()

			f.find_desired_wp()

			f.transformed_desired_point = f.transformPoint(f.current_position, f.desired_point)
			f.findPath()
			steering_angle := f.setSteeringAngle()
			speed := 0.0
			if math.Abs(steering_angle) > f.STRAIGHTS_STEERING_ANGLE {
				speed = f.CORNERS_SPEED
			} else {
				speed = f.STRAIGHTS_SPEED
			}

			msg := &std_msgs.Float32MultiArray{
				Data: []float32{float32(steering_angle), float32(speed)},
			}

			f.pub.Write(msg)
			fmt.Println(steering_angle, speed)
		}
	}
	fmt.Println("asdasf")
}
func (f *Pure_Pursuit) get_waypoint() [][]float64 {
	file_wps, _ := os.Open(f.WPS_FILE_LOCATION)
	rdr := csv.NewReader(bufio.NewReader(file_wps))
	csvs, _ := rdr.ReadAll()
	var temp_waypoint [][]float64

	for i, row := range csvs {
		var wprow []float64
		if i == 0 {
			continue
		}

		splited_row := strings.Split(row[0], ";")
		sm_num, _ := strconv.ParseFloat(splited_row[0], 64)
		x_num, _ := strconv.ParseFloat(splited_row[1], 64)
		y_num, _ := strconv.ParseFloat(splited_row[2], 64)
		wprow = append(wprow, sm_num, x_num, y_num)
		temp_waypoint = append(temp_waypoint, wprow)

	}
	// fmt.Println(temp_waypoint)

	return temp_waypoint
}

func (f *Pure_Pursuit) Odome(msg *nav_msgs.Odometry) {
	qx := msg.Pose.Pose.Orientation.X
	qy := msg.Pose.Pose.Orientation.Y
	qz := msg.Pose.Pose.Orientation.Z
	qw := msg.Pose.Pose.Orientation.W

	siny_cosp := 2.0 * (qw*qz + qx*qy)
	cosy_cosp := 1.0 - 2.0*(qy*qy+qz*qz)

	current_position_theta := math.Atan2(siny_cosp, cosy_cosp)
	current_position_x := msg.Pose.Pose.Position.X
	current_position_y := msg.Pose.Pose.Position.Y
	f.current_position = make([]float64, 0)
	f.current_position = append(f.current_position, current_position_x)
	f.current_position = append(f.current_position, current_position_y)
	f.current_position = append(f.current_position, current_position_theta)
}

func (f *Pure_Pursuit) find_desired_wp() {
	wpIndexTemp := f.wp_index_current

	for {
		if wpIndexTemp >= len(f.waypoints)-1 {
			wpIndexTemp = 0
		}

		distance := f.getDistance(f.waypoints[wpIndexTemp], f.current_position)

		if distance >= f.lookahead_desired {

			if wpIndexTemp-2 >= 0 && wpIndexTemp+2 < len(f.waypoints)-1 {
				f.waypoints[wpIndexTemp][2] = math.Atan((f.waypoints[wpIndexTemp+2][1] - f.waypoints[wpIndexTemp-2][1]) / (f.waypoints[wpIndexTemp+2][0] - f.waypoints[wpIndexTemp-2][0]))
			}

			f.desired_point = f.waypoints[wpIndexTemp]
			f.actual_lookahead = distance
			break

		}
		wpIndexTemp++
	}
}

func (f *Pure_Pursuit) getDistance(a []float64, b []float64) float64 {
	dx := a[0] - b[0]
	dy := a[1] - b[1]
	result := math.Sqrt(math.Pow(dx, 2) + math.Pow(dy, 2))
	return result
}

func (f *Pure_Pursuit) transformPoint(origin []float64, target []float64) []float64 { //한번보기
	theta := math.Pi - origin[2]

	dx := target[0] - origin[0]
	dy := target[1] - origin[1]
	dtheta := target[2] + theta

	tf_point_x := dx*math.Cos(theta) - dy*math.Sin(theta)
	tf_point_y := dx*math.Sin(theta) + dy*math.Cos(theta)
	tf_point_theta := dtheta
	var tf_point []float64
	tf_point = append(tf_point, tf_point_x)
	tf_point = append(tf_point, tf_point_y)
	tf_point = append(tf_point, tf_point_theta)

	return tf_point
}
func (f *Pure_Pursuit) getDx() {
	wpMin := f.findLookaheadWP(f.LOOKAHEAD_MIN)
	wpMax := f.findLookaheadWP(f.LOOKAHEAD_MAX)

	wpMin = f.transformPoint(f.current_position, wpMin)
	wpMax = f.transformPoint(f.current_position, wpMax)

	f.dx = wpMax[0] - wpMin[0]

}
func (f *Pure_Pursuit) findLookaheadWP(length float64) []float64 {
	wpIndexTemp := f.wp_index_current
	for {
		if wpIndexTemp >= f.wp_len-1 {
			wpIndexTemp = 0
		}
		distance := f.getDistance(f.waypoints[wpIndexTemp], f.current_position)
		if distance >= length {
			break
		}
		wpIndexTemp++
	}
	return f.waypoints[wpIndexTemp]
}

func (f *Pure_Pursuit) getLookaheadDesired() {
	f.lookahead_desired = math.Exp(-(f.DX_GAIN*math.Abs(f.dx) - math.Log(f.LOOKAHEAD_MAX-f.LOOKAHEAD_MIN))) + f.LOOKAHEAD_MIN
}

func (f *Pure_Pursuit) findPath() {

	// Right cornering
	if f.transformed_desired_point[0] > 0 {
		f.goal_path_radius = math.Pow(f.actual_lookahead, 2) / (2 * f.transformed_desired_point[0])
		f.goal_path_theta = math.Asin(f.transformed_desired_point[1] / f.goal_path_radius)
		f.steering_direction = -1
	} else if f.transformed_desired_point[0] < 0 {
		f.goal_path_radius = math.Pow(f.actual_lookahead, 2) / ((-2) * f.transformed_desired_point[0])
		f.goal_path_theta = math.Asin(f.transformed_desired_point[1] / f.goal_path_radius)
		f.steering_direction = 1
	} else {
		fmt.Printf("raised else option. %v - %v\n", f.transformed_desired_point, time.Now())
	}
}

func (f *Pure_Pursuit) setSteeringAngle() float64 {
	steeringAngle := math.Atan2(f.RACECAR_LENGTH, f.goal_path_radius)
	return float64(f.steering_direction)*steeringAngle - 0.006
}

func (f *Pure_Pursuit) getWaypoint_ros1(wpsFileLocation string) [][]float64 {
	file, err := os.Open(wpsFileLocation)
	if err != nil {
		fmt.Errorf("can't read waypoint file: %v", err)
		return nil
	}
	defer file.Close()

	reader := csv.NewReader(file)
	reader.Comma = ','

	records, err := reader.ReadAll()
	if err != nil {
		fmt.Errorf("failed to read waypoints from file: %v", err)
		return nil
	}

	waypoints := make([][]float64, len(records))
	for i, record := range records {
		waypoint := make([]float64, len(record))
		for j, value := range record {

			num, err := strconv.ParseFloat(value, 64)

			if err != nil {
				fmt.Errorf("failed to parse waypoint value: %v", err)
				return nil
			}
			waypoint[j] = num
		}
		waypoints[i] = waypoint

	}

	return waypoints
}
