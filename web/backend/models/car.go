package models

type CarInfo struct {
	UUID string `json:"UUID"`
	Longitude float64 `json:"longitude"`
	Latitude float64 `json:"latitude"`
	DriverStatus string `json:"driver_status"`
	Model string `json:"model"`
	Color string `json:"color"`
}
