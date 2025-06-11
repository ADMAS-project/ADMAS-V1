package main

import (
	"fmt"

	"github.com/google/uuid"
)

func main() {
	for {
		uuidStr := uuid.New().String()
		fmt.Println("Press any key to generate a new one")
		fmt.Scanf("%v")
		fmt.Println(uuidStr)
		fmt.Println()
	}
}
