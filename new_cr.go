package main

import (
	"fmt"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/ackermann_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

const ctimemin int = 1
const ctimemax int = 3
const peridod int = 18

type CR struct {
	pp_steering_angle float32
	pp_speed          float32

	fgm_steering_angle float32
	fgm_speed          float32

	obstacle bool

	ackermann ackermann_msgs.AckermannDriveStamped
	pub       *goroslib.Publisher
}

func new_cr() *CR {
	f := CR{}
	f.pp_steering_angle = 0
	f.pp_speed = 0
	f.fgm_speed = 0
	f.fgm_steering_angle = 0
	f.obstacle = false
	return &f
}

func main() {
	f := new_cr()

	cr_node, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "cr",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer cr_node.Close()

	odsub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     cr_node,
		Topic:    "/OD_CR",
		Callback: f.onMessage_OD,
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	defer odsub.Close()

	ppsub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     cr_node,
		Topic:    "/PP_CR",
		Callback: f.onMessage_PP,
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	defer ppsub.Close()

	fgmsub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     cr_node,
		Topic:    "/FGM_CR",
		Callback: f.onMessage_FGM,
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	defer fgmsub.Close()

	f.pub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  cr_node,
		Topic: "/drive",
		Msg:   &ackermann_msgs.AckermannDriveStamped{},
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	defer f.pub.Close()

	//r := cr_node.TimeRate(100*time.Millisecond - 1*time.Millisecond)

	cc := make(chan os.Signal, 1)
	signal.Notify(cc, os.Interrupt)
	c_now := time.Now()
	c := time.Since(c_now)

	t_now := time.Now()
	t := time.Since(t_now)
	eps := time.Millisecond * 10
	var processing_passage []string
	var wait_passage []string
	goto init

init:
	goto ready
ready:
	c_now = time.Now()
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
		if f.obstacle {
			f.ackermann.Drive.SteeringAngle = float32(f.fgm_steering_angle)
			f.ackermann.Drive.Speed = float32(f.fgm_speed)
		} else {
			f.ackermann.Drive.SteeringAngle = float32(f.pp_steering_angle)
			f.ackermann.Drive.Speed = float32(f.pp_speed)
		}
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
	c = time.Since(c_now)

	select {
	// publish a message every second
	case <-time.After(time.Duration(ctimemax)*time.Millisecond - c):
		goto exp
	case <-time.After(0 * time.Millisecond):
		goto mid
	case <-cc:
		return
	}
mid:
	f.pub.Write(&f.ackermann)
	t_now = time.Now()
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
		goto wait_2
	case <-cc:
		return
	}
wait_2:
	t = time.Since(t_now)
	select {
	// publish a message every second
	case <-time.After(time.Duration(peridod)*time.Millisecond - t):
		goto exp
	case <-time.After(0 * time.Millisecond):
		goto ready
	case <-cc:
		return
	}
exp:
	fmt.Println("exp loc")
	goto ready

	// select {
	// // publish a message every second
	// case <-time.After(100*time.Millisecond - x - eps):
	// 	if f.obstacle {
	// 		f.ackermann.Drive.SteeringAngle = float32(f.fgm_steering_angle)
	// 		f.ackermann.Drive.Speed = float32(f.fgm_speed)
	// 	} else {
	// 		f.ackermann.Drive.SteeringAngle = float32(f.pp_steering_angle)
	// 		f.ackermann.Drive.Speed = float32(f.pp_speed)
	// 	}
	// 	f.pub.Write(&f.ackermann)
	// 	goto init
	// case <-c:
	// 	return
	// }

}
func (f *CR) driving(node *goroslib.Node) {
	r := node.TimeRate(1 * time.Millisecond)

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	for {
		select {
		// publish a message every second
		case <-r.SleepChan():
			if f.obstacle {
				f.ackermann.Drive.SteeringAngle = float32(f.fgm_steering_angle)
				f.ackermann.Drive.Speed = float32(f.fgm_speed)
			} else {
				f.ackermann.Drive.SteeringAngle = float32(f.pp_steering_angle)
				f.ackermann.Drive.Speed = float32(f.pp_speed)
			}
			f.pub.Write(&f.ackermann)
		case <-c:
			return
		}
	}
	fmt.Println("asdasf")
}

func (f *CR) onMessage_PP(msg *std_msgs.Float32MultiArray) {
	f.pp_steering_angle = msg.Data[0]
	f.pp_speed = msg.Data[1]
	//fmt.Println("pp")
}

func (f *CR) onMessage_FGM(msg *std_msgs.Float32MultiArray) {
	f.fgm_steering_angle = msg.Data[0]
	f.fgm_speed = msg.Data[1]
	//fmt.Println("fgm")

}
func (f *CR) onMessage_OD(msg *std_msgs.Bool) {
	f.obstacle = msg.Data
	//fmt.Println("od")

}
func time_passage(time_passage []string, ctime time.Duration) int {
	for i, val := range time_passage { // 비교하는거 추가
		if strings.Contains(val, "==") {

			num, _ := strconv.Atoi(val[strings.Index(val, "==")+2:])
			if time.Second*time.Duration(num) > ctime {
				return i
			}
		} else if strings.Contains(val, "<") {
			num, _ := strconv.Atoi(val[strings.Index(val, "<")+1:])

			if time.Second*time.Duration(num) == ctime {
				return i
			}
		}
	}
	return len(time_passage)
}
