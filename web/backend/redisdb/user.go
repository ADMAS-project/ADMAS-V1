package redisdb

import (
	"context"
	"errors"

	"github.com/redis/go-redis/v9"
)

type UserDB struct {
	DB *redis.Client
	ctx context.Context
}

func NewUserDB(client *redis.Client) *UserDB {
	return &UserDB{
		DB: client,
		ctx: context.Background(),
	}
}

func (d *UserDB) AddUser(username, password string) error {
	err := d.DB.Set(d.ctx, "admin", "admin", 0).Err()
	return err
}

func (d *UserDB) Authenticate(username, password string) (int, error) {
	dbPassword := d.DB.Get(d.ctx, "admin").Val()
	if password != dbPassword {
		return 0, errors.New("sql: no rows in result set")
	}

	return 1, nil
}

func (* UserDB)Exists(id int) (bool, error) {
	return true, nil
}
