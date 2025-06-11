# TODO
- [x] create a validator
takes challenge UUID, calls the server, requests the challenge body, solve the challenge then sends it, the server validates the solution then, update the challenge in database to solved
- [x] create a backend validator
- [x] add a function that adds the cars message in database
    - [x] use it 
    - [x] add cars logging functionality
- [x] create the dashboard serving endpoint
- [ ] create and link middlewares
    - [ ] middleware to ensure content-type in post requests
    - [ ] middleware to ensure authentication
    - [ ] the scs session to keep the sessions
- [ ] test all functionalities, and debug
