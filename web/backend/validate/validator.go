package validate

import (
	"regexp"
)

// this special data type will be used to create enums for the validator
type Key string

var (
	EmailRX = regexp.MustCompile("^[a-zA-Z0-9.!#$%&'*+\\/=?^_`{|}~-]@emergency.sys")
	Email Key = "email"
)

type validator struct {
	Errors map[Key]string
}


// constructor to create a validator struct
func New() *validator {
	return &validator{
		Errors: make(map[Key]string),
	}
}

// valid function that returns true if no errors are catched in the validation process
func (v *validator) Valid() bool {
	return len(v.Errors) == 0
}

// Add Error to the errors map in validator struct if the key is not exists
// params: key: key -> takes the error type
// params: message: string -> takes the error type
func (v *validator) AddError(key Key, message string) {
	if _, exist := v.Errors[key]; !exist {
		v.Errors[key] = message
	}
}

func (v *validator) Check(ok bool, key Key, message string) {
	if !ok {
		v.AddError(key, message)
	}
}

func Matches(value string, rx regexp.Regexp) bool {
	return rx.MatchString(value)
}

func NotEmpty(value string) bool {
	return len(value) > 0
}

func In(value string, list []string) bool {
	for _, val := range list {
		if value == val {
			return true
		}
	}

	return false
}
