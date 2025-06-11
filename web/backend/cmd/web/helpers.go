package main

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"
)

// function that is used to read json data from request and send to dst interface
func readJSON(w http.ResponseWriter, r *http.Request, dst any) error {
	err := json.NewDecoder(r.Body).Decode(dst)
	if err != nil {
		// If there is an error during decoding, start the triage...
		var syntaxError *json.SyntaxError
		var unmarshalTypeError *json.UnmarshalTypeError
		var invalidUnmarshalError *json.InvalidUnmarshalError
		switch {
			// Use the errors.As() function to check whether the error has the type
			// *json.SyntaxError. If it does, then return a plain-english error message
			// which includes the location of the problem.
		case errors.As(err, &syntaxError):
			return fmt.Errorf("body contains badly-formed JSON (at character %d)", syntaxError.Offset)
			// In some circumstances Decode() may also return an io.ErrUnexpectedEOF error
			// for syntax errors in the JSON. So we check for this using errors.Is() and
			// return a generic error message.
		case errors.Is(err, io.ErrUnexpectedEOF):
			return errors.New("body contains badly-formed JSON")
			// Likewise, catch any *json.UnmarshalTypeError errors. These occur when the
			// JSON value is the wrong type for the target destination. If the error relates
			// to a specific field, then we include that in our error message to make it
			// easier for the client to debug.
		case errors.As(err, &unmarshalTypeError):
			if unmarshalTypeError.Field != "" {
				return fmt.Errorf("body contains incorrect JSON type for field %q", unmarshalTypeError.Field)
			}
			return fmt.Errorf("body contains incorrect JSON type (at character %d)", unmarshalTypeError.Offset)
			// An io.EOF error will be returned by Decode() if the request body is empty. We
			// check for this with errors.Is() and return a plain-english error message
			// instead.
			// An io.EOF error will be returned by Decode() if the request body is empty. We
			// check for this with errors.Is() and return a plain-english error message
			// instead.
		case errors.Is(err, io.EOF):
			return errors.New("body must not be empty")
			// A json.InvalidUnmarshalError error will be returned if we pass a non-nil
			// pointer to Decode(). We catch this and panic, rather than returning an error
			// to our handler. At the end of this chapter we'll talk about panicking
			// versus returning errors, and discuss why it's an appropriate thing to do in
			// this specific situation.
		case errors.As(err, &invalidUnmarshalError):
			panic(err)
			// For anything else, return the error message as-is.
		default:
			return err
		}

	}

	return nil
}


// function that is used to write data into w http.ResponseWriter attaching headers and status code
func writeJSON(w http.ResponseWriter, status int, data any, headers http.Header) error {
	js, err := json.Marshal(data)
	if err != nil {
		return err
	}

	js = append(js, '\n')

	for key, value := range headers {
		w.Header()[key] = value
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	w.Write(js)

	return nil
} 

// built on writeJSON this function returns a error to client
func clientError(w http.ResponseWriter, status int, err error) error {
	jsonError := struct{
		Status string `json:"status"`
		Message string `json:"message"`
	}{
		Status: "error",
		Message: err.Error(),
	}

	return writeJSON(w, status, jsonError, nil)
}

func serverError(w http.ResponseWriter, err error) error {
	jsonError := struct{
		Status string `json:"status"`
		Message string `json:"message"`
	} {
		Status: "server error",
		Message: "500 internal server error ",
	}

	log.Println(err)

	return writeJSON(w, http.StatusInternalServerError, jsonError, nil)
}
