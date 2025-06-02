package database

import (
	"gp-backend/models"
)

func (d UserDB) AddCar(carInfo models.CarInfo) (error) {
	stmt := `INSERT INTO cars(uuid, longitude, latitude, driver_status, color, model, served) VALUES ($1, $2, $3, $4, $5, $6, false)`
	_, err := d.DB.Exec(stmt, carInfo.UUID, carInfo.Longitude, carInfo.Latitude, carInfo.DriverStatus, carInfo.Color, carInfo.Model)
	if err != nil {
		return err
	}

	return nil
}

// this function gets the cars info from database
func (d UserDB) GetCar(carUUID string) (*models.CarInfo, error) {
	var carinfo models.CarInfo
	stmt := `SELECT longitude, latitude, driver_status, color, model FROM cars WHERE uuid=$1`
	if err := d.DB.QueryRow(stmt, carUUID).Scan(&carinfo.Longitude, &carinfo.Latitude, &carinfo.DriverStatus, &carinfo.Color, &carinfo.Model); err != nil {
		return nil, err
	}

	return &carinfo, nil
}
