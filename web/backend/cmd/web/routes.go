package main

import (
	"net/http"

	"github.com/justinas/alice"
)

func (a *application) routes() http.Handler {
	mux := http.NewServeMux()

	middlewares := alice.New(a.ensureJsonContentType)
	authMiddleware := alice.New(a.requireAuthentication)
	// mux.HandleFunc("GET /challenge/{challUUID}", a.challGetter)
	mux.Handle("GET /events", authMiddleware.ThenFunc(a.stream))
	mux.HandleFunc("GET /ping", a.ping)
	mux.Handle("GET /view/{carUUID}", authMiddleware.ThenFunc(a.viewCar))
	// mux.HandleFunc("POST /user/login", middleware.Then(a.login))
	mux.Handle("POST /user/login", middlewares.ThenFunc(a.login))
	// mux.HandleFunc("POST /user/challenge", a.challenge) // this endpoint is used after the challenge was solved with the validator, to tell the front end to redirect to dashboard
	// mux.HandleFunc("POST /challenge/{challUUID}", a.challengeValidator)
	mux.HandleFunc("POST /cars/sos", a.sos)


	return alice.New(a.logger, a.cors("http://localhost:5173")).Then(mux)
}
