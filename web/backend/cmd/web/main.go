package main

import (
	"database/sql"
	"gp-backend/database"
	"gp-backend/models"
	"gp-backend/redisdb"
	"log"
	"net/http"
	"os"
	"time"

	"github.com/alexedwards/scs/postgresstore"
	"github.com/alexedwards/scs/redisstore"
	"github.com/alexedwards/scs/v2"
	_ "github.com/lib/pq"
	"github.com/r3labs/sse/v2"
	"github.com/gomodule/redigo/redis"
)

type UserDB interface {
	AddCar(models.CarInfo) (error)
	GetCar(string) (*models.CarInfo, error)
	AddUser(string, string) error
	Authenticate(string, string) (int, error)
	Exists(int) (bool, error)
	Serve(string) (error)
}

type application struct {
	sse *sse.Server
	db any
	udb UserDB
	sessionManager *scs.SessionManager
}

func main() {
	defer func() {
		if err := recover(); err != nil {
			log.Println("panic occurred:", err)
		}
	}()
	app, err := NewApplication("messages")
	if err != nil {
		log.Fatalln(err.Error())
	}

	server := http.Server{
		Handler: app.sessionManager.LoadAndSave(app.routes()),
		Addr: ":5000",
	}
	log.Println("SERVER STARTED")
	
	server.ListenAndServe()


}


func OpenDB() (*sql.DB, error) {
	// TODO: connect through ssl (enable sslmode)
	db, err := sql.Open("postgres", "host=localhost user=emergency password=emergency dbname=cars sslmode=disable")
	db.SetMaxOpenConns(20)               // Max total connections
	db.SetMaxIdleConns(5)                // Max idle connections
	db.SetConnMaxLifetime(time.Hour)     // Lifetime of each connection
	db.SetConnMaxIdleTime(30 * time.Minute) // Idle timeout

	if err != nil {
		return nil, err
	}

	if err := db.Ping(); err != nil {
		return nil, err
	}
	
	log.Println("CONNECTED TO DATABASE CARS")

	return db, nil
}


func NewApplication(sseParameter string) (*application, error) {
	env := os.Getenv("ENV")
	if sseParameter == "" {
		sseParameter = "messages"
	}

	// Creating the SSE server
	server := sse.New()
	server.CreateStream(sseParameter)
	if env != "TEST" {
		db, err := OpenDB()
		if err != nil {
			return nil, err
		}
		udb := &database.UserDB{
			DB: db,
		}

		sessionManager := scs.New()
		sessionManager.Store = postgresstore.New(db)
		sessionManager.Lifetime = 12 * time.Hour
		return &application{
			sse: server,
			db: db,
			udb: udb,
			sessionManager: sessionManager,
		}, nil

	} else {
		db, err := redisdb.OpenDB()
		if err != nil {
			return nil, err
		}
		udb := redisdb.NewUserDB(db)

		pool := &redis.Pool{
			MaxIdle: 10,
			Dial: func() (redis.Conn, error) {
				return redis.Dial("tcp", "localhost:6379")
			},
		}
		sessionManager := scs.New()
		sessionManager.Store = redisstore.New(pool)
		sessionManager.Lifetime = 12 * time.Hour
		return &application{
			sse: server,
			db: db,
			udb: udb,
			sessionManager: sessionManager,
		}, nil

	}

}
