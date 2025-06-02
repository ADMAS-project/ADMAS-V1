package main

import (
	"encoding/base64"
	"encoding/json"
	"fmt"
	"gp-backend/crypto/validator"
	"gp-backend/database"
	"net/http"

	"github.com/google/uuid"
)

func (a *application) challGetter(w http.ResponseWriter, r *http.Request) {
	challUUID := r.PathValue("challUUID")
	if err := uuid.Validate(challUUID); err != nil {
		clientError(w, http.StatusBadRequest, err)
		return
	}
	challenge, err := database.GetChallengeStr(a.db, challUUID)
	if err != nil {
		serverError(w, err)
		return
	}

	chall := base64.StdEncoding.EncodeToString(challenge)

	jsonMsg := struct{
		Status string `json:"status"`
		Challenge string `json:"challenge"`
	}{
		Status: "success", 
		Challenge: chall,
	}

	if err := writeJSON(w, http.StatusOK, jsonMsg, nil); err != nil {
		serverError(w, err)
		return
	}
}

func (a *application) challengeValidator(w http.ResponseWriter, r *http.Request) {
	challUUID := r.PathValue("challUUID")
	if err := uuid.Validate(challUUID); err != nil {
		clientError(w, http.StatusBadRequest, err)
		return
	}

	var clientSol map[string]string
	if err := readJSON(w, r, &clientSol); err != nil {
		clientError(w, http.StatusBadRequest, err)
		return
	}

	challStr, err := database.GetChallengeStr(a.db, challUUID)
	if err != nil {
		serverError(w, err)
		return
	}

	pub, err := database.GetPubKey(a.db, challUUID)
	if err != nil {
		serverError(w, err)
		return
	}

	resolved, err := validator.ResolveChallenge(pub, challStr, clientSol["signature"])
	if err != nil {
		serverError(w, err)
		return
	}

	if !resolved {
		clientError(w, http.StatusUnauthorized, fmt.Errorf("challenge solution is wrong"))
		return
	}

	if err := database.SolveChall(a.db, challUUID); err != nil {
		serverError(w, err)
		return
	}

	jsonSuccess, err := json.Marshal(map[string]string{
		"status":"success",
		"message":"success, validate at client-side",
	})

	if err := writeJSON(w, http.StatusOK, jsonSuccess, nil); err != nil {
		serverError(w, err)
		return
	}

	return
}
