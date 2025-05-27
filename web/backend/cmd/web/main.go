package main

import (
	"database/sql"
	"gp-backend/database"
	"log"
	"net/http"
	"time"
	"github.com/alexedwards/scs/postgresstore"
	"github.com/alexedwards/scs/v2"
	_ "github.com/lib/pq"
	"github.com/r3labs/sse/v2"
)

type application struct {
	sse *sse.Server
	db *sql.DB
	udb database.UserDB
	sessionManager *scs.SessionManager
}

func main() {
	app, err := NewApplication("messages")
	if err != nil {
		log.Fatalln(err.Error())
	}

	server := http.Server{
		Handler: app.routes(),
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
	if sseParameter == "" {
		sseParameter = "messages"
	}

	// Creating the SSE server
	server := sse.New()
	server.CreateStream(sseParameter)

	db, err := OpenDB()
	if err != nil {
		return nil, err
	}
	udb := database.UserDB{
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
}
