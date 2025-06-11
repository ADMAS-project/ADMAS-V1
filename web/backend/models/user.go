package models

type User struct {
	Id		  int
	Username  string `json:"username"`
	Password  string `json:"password"`
	Role	  string `json:"-"`
	PublicKey string `json:"-"`
}
