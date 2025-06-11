package main

import (
	"fmt"
	"github.com/r3labs/sse/v2"
)

func main() {
	client := sse.NewClient("http://127.0.0.1:5000/events?stream=messages")
	client.Subscribe("messages", func(msg *sse.Event) {
		fmt.Println(string(msg.Data))
	})
}
