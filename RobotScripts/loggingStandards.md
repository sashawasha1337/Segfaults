# Logging Standards

All nodes should log

* Initialization start
* Initialization complete
* First succesful publish `once=True`
* Shutdown initiated
* Critical errors with context
* Debug information at the DEBUG level


### Levels

* DEBUG : Execution details
* INFO :  Useful for the user
* WARN : Unintended behavior
* ERROR : Failure
* FATAL : Crashes

### Formatting
Use python f-strings for formatting variables into log messages.
Include units where possible

### Rate Limiting
High frequency operations (many times a minute) should use `throttle_duration_sec=` to prevent spamming logs.