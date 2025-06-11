package main

import (
	"fmt"
	"gp-backend/crypto/generator"
	"gp-backend/database"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
)

func main() {
	if !isRoot() {
		panic("ERROR:MUST RUN AS ROOT")
	}

	args, err := getArgs()
	if err != nil {
		panic(err)
	}



	// creating a dir to mount the flash drive defaulting to /tmp/usb
	if err := createMntDir(); err != nil {
		panic(err)
	}


	// mounting usb drive to write config info into it
	// getting the usb device path 
	usbPath, err := getUSBPath(args)
	if err != nil {
		panic(err)
	}

	if err := createMntDir(); err != nil {
		panic(err)
	}

	// mounting usb device
	if err := MntDrive(usbPath, "/tmp/usb"); err != nil {
		panic(err)
	}
	// defer the unmount to unmout the usb drive
	defer exec.Command("umount", "/tmp/usb")


	// generate private key
	privateKey, err := generator.GeneratePrivateKey()
	if err != nil {
		panic(err)
	}
	// write private key to memory (usb flashdrive)
	privateKeyPEM := generator.GeneratePrivateKeyPEM(privateKey)
	err = os.WriteFile("/tmp/usb/private_key.pem", privateKeyPEM, 0600)
	if err != nil {
		panic(err)
	}

	// Generate public key for authentication
	publicKey := generator.GeneratePublicKey(privateKey)
	publicKeyPEM := generator.GeneratePublicKeyPEM(publicKey)
	fmt.Println(string(publicKeyPEM))

	fmt.Println("Initialization completed successfully")
	fmt.Print("Save token to database? [Y/n] ")
	var choice rune
	fmt.Scanf("%v", choice)
	switch strings.ToLower(string(choice)) {
	case "n":
		return
	default:
		var userId uint
		fmt.Print("Enter user id: ")
		if _, err := fmt.Scanf("%d", &userId); err != nil {
			log.Fatalln(err)
		}

		db, err := database.OpenDB()
		if err != nil {
			panic(err)
		}
		if err := db.AddToken(userId, string(publicKeyPEM)); err != nil {
			panic(err)
		}
		fmt.Println("Token added to database successfully")
		return
	}
	
}

func getArgs() ([]string, error) {
	if len(os.Args) < 3 {
		return nil, fmt.Errorf("No suffecient args are providied\nusage tokenGen <drive path> <user password>")
	}

	return os.Args, nil
}

func getUSBPath(args []string) (string, error) {
	matches, err := filepath.Glob(args[1])
	if err != nil {
		return "", err
	}

	return matches[0], nil
}

func getPassword(args []string) (string) {
	return args[2]
}

func isRoot() (bool) {
	return os.Getuid() == 0
}

func createMntDir() (error) {
	 return os.MkdirAll("/tmp/usb", 0755)
}

func MntDrive(usbPath, mountPath string) (error) {
	cmd := exec.Command("mount", usbPath, mountPath)
	return cmd.Run()
}
