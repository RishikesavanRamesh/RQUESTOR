# Rover
jsut scheking



## issues : solved?

1) Problem with gui in dev layer for a developer as a non root user ?
        Description: When you try to run a gui application in a docker container as a non-root user, with X server it is directly not possible to access the x server. 
        What works: as a non root user, using sudo to run an application can work, eg. "user@34f23j3kgkr23g1234:- sudo gedit "
                Why it is not for us. : We have to run gui applications as non root user for debugging and developing ros project (eg: rviz2 and gazebo), which needs the environment sourced with ros and our packages. So running packages with non root user, and running gui applications with sudo / root , may cause problem as they have separate environment.

        Solution: One work around is, to use the credentials of the host user having access to the X server to create the development_user in the container , so that it matches with the host, thus allowing to access the x server, as non root user. eg: "user@34f23j3kgkr23g1234:- gedit "