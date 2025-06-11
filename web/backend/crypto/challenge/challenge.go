package challenge

import (
	"crypto/rand"

	"github.com/google/uuid"
)

func genChalId() (string) {
	uuid := uuid.New().String()
	return uuid
}

func genChalStr() ([]byte, error) {
	chalByte := make([]byte, 32)

	_, err := rand.Read(chalByte)
	if err != nil {
		return nil, err
	}

	return chalByte, nil
}

func CreateChallenge() (string, []byte, error) {
	chalStr, err := genChalStr()
	return genChalId(), chalStr, err
}
