package redisdb

import (
	"fmt"
	"gp-backend/models"
	"strconv"
)

func (d *UserDB) AddCar(carInfo models.CarInfo) error {
	carMap := map[string]interface{}{
		"UUID":         carInfo.UUID,
		"Longitude":    strconv.FormatFloat(carInfo.Longitude, 'f', -1, 64),
		"Latitude":     strconv.FormatFloat(carInfo.Latitude, 'f', -1, 64),
		"DriverStatus": carInfo.DriverStatus,
		"Model":        carInfo.Model,
		"Color":        carInfo.Color,
		"Served": 		"false",
	}
	return d.DB.HSet(d.ctx, carInfo.UUID, carMap).Err()
}

func (d *UserDB) GetCar(carUUID string) (*models.CarInfo, error) {
	val, err := d.DB.HGetAll(d.ctx, carUUID).Result()
	if err != nil {
		return nil, err
	}
	if len(val) == 0 {
		return nil, fmt.Errorf("car with UUID %s not found", carUUID)
	}

	carInfo := models.CarInfo{
		UUID:         val["UUID"],
		DriverStatus: val["DriverStatus"],
		Model:        val["Model"],
		Color:        val["Color"],
	}

	carInfo.Longitude, err = strconv.ParseFloat(val["Longitude"], 64)
	if err != nil {
		return nil, err
	}
	carInfo.Latitude, err = strconv.ParseFloat(val["Latitude"], 64)
	if err != nil {
		return nil, err
	}

	return &carInfo, nil
}

func (d *UserDB) Serve(carUUID string) (error) {
	val, err := d.DB.HGetAll(d.ctx, carUUID).Result()
	if err != nil {
		return err
	}

	val["served"] = "true"
	if err := d.DB.HSet(d.ctx, carUUID, val).Err(); err != nil {
		return err
	}

	return nil
}
