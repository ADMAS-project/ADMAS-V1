module gp-backend

go 1.23.1

require (
	github.com/alexedwards/scs/postgresstore v0.0.0-20240316134038-7e11d57e8885
	github.com/alexedwards/scs/redisstore v0.0.0-20250417082927-ab20b3feb5e9
	github.com/alexedwards/scs/v2 v2.8.0
	github.com/gomodule/redigo v1.8.0
	github.com/google/uuid v1.6.0
	github.com/justinas/alice v1.2.0
	github.com/lib/pq v1.10.9
	github.com/r3labs/sse/v2 v2.10.0
	github.com/redis/go-redis/v9 v9.9.0
	golang.org/x/crypto v0.29.0
)

require (
	github.com/cespare/xxhash/v2 v2.3.0 // indirect
	github.com/dgryski/go-rendezvous v0.0.0-20200823014737-9f7001d12a5f // indirect
	golang.org/x/net v0.21.0 // indirect
	gopkg.in/cenkalti/backoff.v1 v1.1.0 // indirect
)
