## Backend System 
In this system the backend has the following responsibilities:
- Handling authentication, and authorization of the users.
- Handling, parsing, and validating the incoming SOS.
- Streaming the valid SOS signals via SSE stream to front-end as JSON objects.
- Logging vehicles signals and info.

All this information are handled and logged using 2 datastorage options you can choose between them:
- PostgreSQL
- Redis

Something like that is possible due to the backend following Hexagonal architecture which allow us to separate the system into distinct components using ports that can be plugged with whatever plugs that follow the defined architecture.

## Backend system architecture
```bash
 .
├──  cmd
│   ├──  client
│   │   └──  main.go # Testing client for the backend
│   └──  web # The Core Logic directory
│       ├──  database.go
│       ├──  handlers.go
│       ├──  helpers.go
│       ├──  main.go
│       ├──  middleware.go
│       └──  routes.go
├──  crypto # Physical key 2FA authentication system
│   ├──  challenge
│   │   └──  challenge.go
│   ├──  generator
│   │   └──  generate.go
│   └──  validator
│       └──  validator.go
├──  database # PostgreSQL database adapter
│   ├──  cars.go
│   ├──  database.go
│   ├──  tokens.go
│   └──  user.go
├──  redisdb # Redis database adapter
│   ├──  car.go
│   ├──  redis.go
│   └──  user.go
├──  models # Main Entities for the core system
│   ├──  car.go
│   ├──  msg.go
│   └──  user.go
└──  validate # Input validation module
    └──  validator.go
```

## Database objects
There are 2 database object
```bash
Entities
├── Car # Car entity represents the main target of our project
│   ├── UUID
│   ├── Longitude
│   ├── Latitude
│   ├── Driver status
│   ├── Color
│   └── Model
└── User # User entity represents the hospital (emergency organization) employee
    ├── ID
    ├── Username
    ├── Passowrd
    ├── Public Key
    └── Role
```

## Security
Several security consideration were taken, and solved including
- Authentication and authorization application
- CSRF protection
- XSS protection
- CORS strict configuration

