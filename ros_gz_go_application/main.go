package main

import (
	"context"
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	geometryMsgs "github.com/iema-group/goros2/msgs/geometry_msgs/msg"
	"github.com/ATIinc/rclgo/pkg/rclgo"
)

func run() error {
	rclArgs, _, err := rclgo.ParseArgs(os.Args[1:])
	if err != nil {
		return fmt.Errorf("failed to parse ROS args: %v", err)
	}

	if err := rclgo.Init(rclArgs); err != nil {
		return fmt.Errorf("failed to initialize rclgo: %v", err)
	}
	defer rclgo.Uninit()

	node, err := rclgo.NewNode("twist_publisher", "")
	if err != nil {
		return fmt.Errorf("failed to create node: %v", err)
	}
	defer node.Close()

	pubOpts := rclgo.NewDefaultPublisherOptions()
	pub, err := geometryMsgs.NewTwistPublisher(node, "/diff_drive/cmd_vel", pubOpts)
	if err != nil {
		return fmt.Errorf("failed to create publisher: %v", err)
	}
	defer pub.Close()

	msg := geometryMsgs.NewTwist()
	msg.Linear.X = 0.5;
	msg.Angular.Z = 0.2;

	ctx, cancel := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer cancel()

	for {
		node.Logger().Infof("Publishing: %#v", msg)
		if err := pub.Publish(msg); err != nil {
			return fmt.Errorf("failed to publish: %v", err)
		}
		select {
		case <-ctx.Done():
			return nil
		case <-time.After(time.Second):
		}
	}
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
