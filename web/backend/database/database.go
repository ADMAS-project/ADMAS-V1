package database

import (
	"database/sql"

	_ "github.com/lib/pq"
)

type dbConnector struct {
	db *sql.DB
}

func OpenDB() (*dbConnector, error) {
	// TODO: connect through ssl (enable sslmode)
	db, err := sql.Open("postgres", "host=localhost user=emergency password=emergency dbname=cars sslmode=disable")
	if err != nil {
		return nil, err
	}

	if err := db.Ping(); err != nil {
		return nil, err
	}
	
	return &dbConnector{
		db,
	}, nil
}



