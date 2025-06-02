package main

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"gp-backend/database"
	"gp-backend/models"
	"gp-backend/validate"
	"log"
	"net/http"

	"github.com/google/uuid"
	"github.com/r3labs/sse/v2"
)

var (
	password validate.Key = "password"
	token validate.Key = "token"
	UUID validate.Key = "UUID"
	driverStatus validate.Key = "driver_status"
	longitude validate.Key = "longitude"
	latitude validate.Key = "latitude"
)

func (a *application) login(w http.ResponseWriter, r *http.Request) {
	a.udb.AddUser("admin", "admin")
	var userInfo models.User
	err := readJSON(w, r, &userInfo)
	if err != nil {
		clientError(w, http.StatusBadRequest, err)
	}
	id, err := a.udb.Authenticate(userInfo.Username, userInfo.Password)
	if err != nil {
		serverError(w, err)
		return
	}
	userInfo.Id = id

	if err := a.sessionManager.RenewToken(r.Context()); err != nil {
		serverError(w, err)
		return
	}

	// // creating challenge
	// chalUUID, challBody, err := challenge.CreateChallenge()
	// if err != nil {
	// 	serverError(w, err)
	// 	return
	// }

	// DONE: 2024/12/02 21:32:42 pq: invalid byte sequence for encoding "UTF8": 0x92
	// convert the type of database column into blob or some shit

	// saving challenge to database
	// if err := database.AddChallenge(a.db, uint(userInfo.Id), chalUUID, challBody); err != nil {
	// 	serverError(w, err)
	// 	return
	// }
	//
	a.sessionManager.Put(r.Context(), "id", id)
	a.sessionManager.Put(r.Context(), "isAuthenticated", true)
	// a.sessionManager.Put(r.Context(), "secondAuthDone", false)
	
	err = a.sessionManager.RenewToken(r.Context())
	if err != nil {
		serverError(w, err)
		return
	}

	token := a.sessionManager.Token(r.Context())

	jsonSuccess := struct {
		Token string `json:"token"`
		Message string `json:"message"`
		// Next string `json:"next"`
	} {
		Token: token,
		Message: "success",
		// Next: fmt.Sprintf("/challenge/%v", chalUUID), 
	}

	if err := writeJSON(w, http.StatusSeeOther, jsonSuccess, nil); err != nil {
		serverError(w, err)
		return
	}
	
}

func (a *application) challenge(w http.ResponseWriter, r *http.Request) {
	Authenticated := a.sessionManager.GetBool(r.Context(), "isAuthenticated")
	secondAuthDone := a.sessionManager.GetBool(r.Context(), "secondAuthDone")

	if !Authenticated || secondAuthDone {
		clientError(w, http.StatusMethodNotAllowed, fmt.Errorf("Not Allowed"))
		return
	}

	var chall struct {
		chalUUID string
	}

	if err := readJSON(w, r, &chall); err != nil {
		clientError(w, http.StatusBadRequest, err)
		return
	}

	solved, err := database.GetChallengeStatus(a.db, chall.chalUUID)
	if err != nil {
		serverError(w, err)
		return
	}

	if !solved {
		clientError(w, http.StatusOK, fmt.Errorf("Challenge not solved"))
		return
	}

	if err := a.sessionManager.RenewToken(r.Context()); err != nil {
		serverError(w, err)
		return 
	}
	a.sessionManager.Put(r.Context(), "secondAuthDone", true)
	if err := a.sessionManager.RenewToken(r.Context()); err != nil {
		serverError(w, err)
		return 
	}

	jsonSuccess := struct{
		Status string `json:"status"`
		Message string `json:"message"`
		Next string `json:"next"`
	} {
		Status: "redirect",
		Message: "success",
		Next: "/dashboard",
	}

	// TODO: CHECK IF YOU WILL LEAVE THE CHALLENGE DELETION AFTER THE BUTTON CLICKING OR AFTER CHALLENGE SOLUTION
	if err := database.DeleteChallenge(a.db, chall.chalUUID); err != nil {
		serverError(w, err)
		return
	}

	if err := writeJSON(w, http.StatusSeeOther, jsonSuccess, nil); err != nil {
		serverError(w, err)
		return
	}

}

func (a *application) stream(w http.ResponseWriter, r *http.Request) {
	a.sse.ServeHTTP(w, r)
}

func (a *application) ping(w http.ResponseWriter, r *http.Request) {
	a.sse.Publish("messages", &sse.Event{
		Data: []byte("ping"),
	})
	writeJSON(w, http.StatusOK, models.Msg{
		Status: "success",
		Message: "success",
	}, nil)
	log.Println("TRIGGER SENT")
}

func (a *application) sos(w http.ResponseWriter, r *http.Request) {
	var data models.CarInfo

	// error handling for json reading, json sent is read into data variable
	err := readJSON(w, r, &data)
	if err != nil {
		clientError(w, http.StatusUnprocessableEntity, err)
		return
	}

	// validating json input
	validator := validate.New()
	// UUID validation
	// Checking not empty
	validator.Check(validate.NotEmpty(data.UUID), UUID, "must not be empty")
	// Checking the UUID format
	if err := uuid.Validate(data.UUID); err != nil {
		validator.AddError(UUID, "Not valid format")
	}

	// driver_status validation
	validator.Check(validate.NotEmpty(data.DriverStatus), driverStatus, "must not be empty")
	validator.Check(validate.In(data.DriverStatus, []string{"sleeping", "fainted", "awake"}), driverStatus, "is a value outside the intended list")

	// longitude & latitude validation
	validator.Check(!(data.Longitude == 0), longitude, "must not be zero")
	validator.Check(!(data.Latitude == 0), latitude, "must not be zero")


	if !validator.Valid() {
		buf := bytes.NewBufferString("")
		for key, value := range validator.Errors {
			format := fmt.Sprintf("%v: %v\n", key, value)
			buf.WriteString(format)
		}

		err := errors.New(buf.String())
		clientError(w, http.StatusBadRequest, err)
		return
	}

	if err := database.AddCar(a.db, data); err != nil {
		serverError(w, err)
		return
	}

	msg := models.Msg{
		Status: "success",
		Message: "success",
	}
	writeJSON(w, http.StatusOK, msg, nil)

	jsonData, err := json.Marshal(data)
	if err != nil {
		log.Println("Couldn't generate cars' data")
		return
	}

	a.sse.Publish("messages", &sse.Event{
		Data: jsonData,
	})
}

// TODO: FILL ME PLEASE
func (a *application) viewCar(w http.ResponseWriter, r *http.Request) {
	carUUID := r.PathValue("carUUID")
	if err := uuid.Validate(carUUID); err != nil {
		clientError(w, http.StatusBadRequest, err)
		return
	}

	carInfo, err := database.GetCar(a.db, carUUID)
	if err != nil {
		serverError(w, err)
		return
	}

	carInfo.UUID = carUUID

	if err := writeJSON(w, http.StatusOK, carInfo, nil); err != nil {
		serverError(w, err)
		return
	}
}
