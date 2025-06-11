package main

import (
	"fmt"
	"log"
	"net/http"
)
 
type Middleware struct {}

// TODO: is authenticated middleware
func (a *application) requireAuthentication(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		authenticated := a.sessionManager.GetBool(r.Context(), "isAuthenticated")
		if !authenticated {
			clientError(w, http.StatusUnauthorized, fmt.Errorf("Unauthorized access to function"))
			return
		}

		// authenticated = a.sessionManager.GetBool(r.Context(), "secondAuthDone")
		// if !authenticated {
		// 	clientError(w, http.StatusUnauthorized, fmt.Errorf("Unauthorized access to function"))
		// 	return
		// }

		id := a.sessionManager.GetInt(r.Context(), "id")
		if id == 0 {
			clientError(w, http.StatusUnauthorized, fmt.Errorf("Unauthorized access to function"))
			return
		}

		exists, err := a.udb.Exists(id)
		if err != nil {
			clientError(w, http.StatusUnauthorized, fmt.Errorf("Unauthorized access to function"))
			return
		}

		if !exists {
			clientError(w, http.StatusUnauthorized, fmt.Errorf("Unauthorized access to function"))
			return
		}
		
		next.ServeHTTP(w, r)
	})
}

// TODO: ensure json middleware
func (a *application) ensureJsonContentType(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		headers, exists := r.Header["Content-Type"]
		if !exists {
			clientError(w, http.StatusBadRequest, fmt.Errorf("ensure json content-type"))
			return
		}

		if headers[0] != "application/json" {
			clientError(w, http.StatusBadRequest, fmt.Errorf("ensure json content-type"))
			return
		}

		next.ServeHTTP(w, r)
	})
} 


// NOTE: Logger middleware
func (a *application) logger(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		  log.Printf("[%s] %s", r.Method, r.URL.Path)
        // Call the next handler
        next.ServeHTTP(w, r)
	})
}

// NOTE: CORS MIDDLEWARE
func (a *application) cors(origin string) func(http.Handler) http.Handler {
	return func (next http.Handler) http.Handler {
		return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Header().Set("Access-Control-Allow-Origin", origin)
			w.Header().Set("access-control-allow-credentials", "true")
			w.Header().Set("Access-Control-Allow-Headers", "Content-Type")
			if r.Method == http.MethodOptions {
				return
			}
			next.ServeHTTP(w, r)
		})
	}
}

