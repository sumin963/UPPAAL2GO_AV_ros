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
const peridod int = 25

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

	file_exec, err := os.Create("cr_execute.txt") 
	if err != nil {
		fmt.Println(err)
		return
	}
	defer file_exec.Close()
	cm := strconv.Itoa(ctimemax)
	pd := strconv.Itoa(peridod)
	_, err = file_exec.Write([]byte(cm)) 
	if err != nil {
		fmt.Println(err)
		return
	}

	file_period, err := os.Create("cr_period.txt") 
	if err != nil {
		fmt.Println(err)
		return
	}
	defer file_period.Close() 

	_, err = file_period.Write([]byte(pd))
	if err != nil {
		fmt.Println(err)
		return
	}

	cc := make(chan os.Signal, 1)
	signal.Notify(cc, os.Interrupt)
	c_now := time.Now()
	c := time.Since(c_now)

	t_now := time.Now()
	t := time.Since(t_now)

	p := cr_node.TimeRate(time.Duration(peridod) * time.Millisecond)

	eps := time.Millisecond * 10
	var processing_passage []string
	var wait_passage []string
	goto init

// 초기화 단계: 현재 시간 기록
init:
	c_now = time.Now() // 처리 시작 시간 기록
	t_now = time.Now() // 전체 주기 시작 시간 기록
	goto ready

// 주 처리 루프 시작
ready:
	// 장애물이 있는 경우 FGM(Follow-the-Gap) 값을 사용
	if f.obstacle {
		f.ackermann.Drive.SteeringAngle = float32(f.fgm_steering_angle)
		f.ackermann.Drive.Speed = float32(f.fgm_speed)
	} else {
		// 장애물이 없을 경우 PP(Pure Pursuit) 값을 사용
		f.ackermann.Drive.SteeringAngle = float32(f.pp_steering_angle)
		f.ackermann.Drive.Speed = float32(f.pp_speed)
	}

	c_now = time.Now() // 처리 시작 시간 갱신
	goto processing

// 처리 시간 측정 및 시간 범위 분류
processing:
	c = time.Since(c_now) // 현재까지의 경과 시간 측정
	// 시간 조건에 따라 다음 단계 분기 (ctimemin, ctimemax: 상한/하한 경계)
	processing_passage = []string{"c==ctimemin", "c>ctimemin", "c>ctimemax"}

	switch time_passage(processing_passage, c) {
	case 0:
		goto processing_1 // 최소 실행 시간 도달 전
	case 1:
		goto processing_2 // 최소 시간 초과, 최대는 아직 아님
	case 2:
		goto processing_3 // 최대 시간 도달 전
	case 3:
		goto exp // 시간 초과로 처리 종료
	}

// 최소 시간까지 대기
processing_1:
	c = time.Since(c_now)
	select {
	case <-time.After(time.Duration(ctimemin)*time.Millisecond - c - eps):
		goto processing_2
	case <-cc: // 종료 시그널
		return
	}

// 중간 상태 처리 또는 빨리 지나가기
processing_2:
	c = time.Since(c_now)
	select {
	case <-time.After(time.Duration(ctimemin)*time.Millisecond - c):
		goto processing_3 // 남은 최소 시간 대기
	case <-time.After(0 * time.Millisecond):
		goto mid // 비동기 상황으로 바로 발행
	case <-cc:
		return
	}

// 최대 시간까지 대기하거나 중간 발행
processing_3:
	fmt.Println("pro3", c)
	c = time.Since(c_now)

	select {
	case <-time.After(time.Duration(ctimemax)*time.Millisecond - c):
		// 최대 시간 도달 후 처리 시간 기록
		_, err = file_exec.Write([]byte(time.Duration.String(time.Now().Sub(c_now)))) 
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

// 제어 명령 발행 및 처리 시간 기록
mid:
	_, err = file_exec.Write([]byte(time.Duration.String(time.Now().Sub(c_now)))) 
	if err != nil {
		fmt.Println(err)
		return
	}
	f.pub.Write(&f.ackermann) // 실제 제어 명령 발행 (예: /cmd_vel)
	goto wait

// 다음 주기 대기
wait:
	t = time.Since(t_now)

	// 주기 도달 여부 확인 (peridod: 목표 주기)
	wait_passage = []string{"t==peridod", "x>peridod"}
	switch time_passage(wait_passage, t) {
	case 0:
		goto wait_1
	case 1:
		goto wait_2
	case 2:
		goto exp
	}

// 남은 주기 시간까지 대기
wait_1:
	t = time.Since(t_now)
	select {
	case <-time.After(time.Duration(peridod)*time.Millisecond - t - eps):
		goto wait_2
	case <-cc:
		return
	}

// 주기 종료 또는 sleep 채널을 통해 제어
wait_2:
	t = time.Since(t_now)
	select {
	case <-time.After(time.Duration(peridod)*time.Millisecond - t):
		// 주기 내 종료 시 처리 시간 기록
		_, err := file_period.Write([]byte(time.Duration.String(time.Now().Sub(t_now)))) 
		if err != nil {
			fmt.Println(err)
			return
		}
		goto exp
	case <-p.SleepChan(): // sleep 이벤트 발생 시 주기 갱신 및 루프 재시작
		_, err := file_period.Write([]byte(time.Duration.String(time.Now().Sub(t_now)))) 
		if err != nil {
			fmt.Println(err)
			return
		}
		t_now = time.Now()
		goto ready
	case <-cc:
		return
	}

// 종료 또는 반복 루프 재시작
exp:
	<-p.SleepChan() // SleepChan으로 블록 후 재시작
	t_now = time.Now()
	fmt.Println("exp loc")
	goto ready
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
	for i, val := range time_passage { 
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
