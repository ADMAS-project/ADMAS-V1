package main

import (
	"net/http"

	"github.com/justinas/alice"
)

func (a *application) routes() http.Handler {
	mux := http.NewServeMux()

	middleware := alice.New(a.sessionManager.LoadAndSave, a.ensureJsonContentType)
	mux.HandleFunc("GET /challenge/{challUUID}", a.challGetter)
	mux.HandleFunc("GET /events", a.stream)
	mux.HandleFunc("GET /ping", a.ping)
	mux.HandleFunc("GET /view/{carUUID}", a.viewCar)
	// mux.HandleFunc("POST /user/login", middleware.Then(a.login))
	mux.Handle("POST /user/login", middleware.ThenFunc(a.login))
	mux.HandleFunc("POST /user/challenge", a.challenge) // this endpoint is used after the challenge was solved with the validator, to tell the front end to redirect to dashboard
	mux.HandleFunc("POST /challenge/{challUUID}", a.challengeValidator)
	mux.HandleFunc("POST /cars/sos", a.sos)


	return alice.New(a.logger, a.cors("http://localhost:5173")).Then(mux)
}
