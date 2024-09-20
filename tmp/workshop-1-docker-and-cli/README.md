
# DOCKER, COMMAND LINE TOOLS, AND GIT


## LEARNING GOALS

In this workshop, you will set up `docker`, `git`, learn some basic functionality of the command line as well as using `git` to track your changes.

Here is the [link to the Google Form](https://docs.google.com/forms/d/e/1FAIpQLScAvwI2c99VhRM8TwnMnb8_J5miD7Piwa6N05_W-Y5ylyEeGg/viewform?usp=sf_link) you'll use to submit this workshop.

This workshop has commands that differ depending on your host operating system (i.e. Windows, MacOS, or Linux) and on which terminal program you're using (Windows WSL vs Windows Powershell).   

_Windows Users_:  Recommended that you use either `Powershell` or `wsl` ([Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/install)) and **_not_** `CMD` the command prompt.

## THIS WORKSHOP ASSUMES:  
* NC State GitHub Account: [https://github.ncsu.edu/](https://github.ncsu.edu/)
* [`docker` installed](https://docs.docker.com/get-docker/).  Choose the academic (student) license if you need to sign up to create a Docker account).  If you really want an open-sourse alternative, I have tested [`podman` and found that it worked for me](https://podman.io/docs/installation).   If you're using `wsl`, I believe you'll need to install `docker` from within `wsl`, not at the windows level.
* All Users: You have some version of `git` installed and have access to something like "Terminal" (Linux), "Terminal (MacOS), "Powershell" (Windows), or "WSL" (Windows, WSL2 preferred).
* Windows Users:  You can either run this workshop from Powershell (recommended) or try installing wsl2. Powershell is great but has a few idiosyncrasies when trying to run some unix commands. If you don't already have `git`, then either set up a git repo in your IDE ([like `VSCode` from Microsoft](https://code.visualstudio.com/)) or use the [`git` command line installer](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) -- **I don't recommend using Git Bash for anything other than `git`, so don't use git bash to try to run Docker, for example.**.

## SETUP `git` and `ssh keys`
[https://github.ncsu.edu/software-engineering-for-robotics/coursewiki/Getting-Started-with-Command-line-Git](https://github.ncsu.edu/software-engineering-for-robotics/course/wiki/Getting-Started-with-Command-line-Git)


## Get the Docker container for this workshop

Navigate in the terminal to somewhere you can do work, like `Downloads` or `Documents`

```
cd ~/Downloads
```

`Clone` the repo for today's workshop.

```
git clone git@github.ncsu.edu:software-engineering-for-robotics/cli-workshop.git
```

Navigate to the workshop's folder:

```
cd cli-workshop
```

Add execution permissions on `ros_entrypoint.sh` using the `chmod` command.

```
chmod +x ros_entrypoint.sh
```

Build the Docker image (this could take a few minutes):

```
docker build . -t cli-workshop
```
In the previous `docker` command, the `-t cli-workshop` creates a docker *image*  from the `Dockerfile` build file.  

Start the `docker` container in interactive mode (`-it` flags) :


```
docker run -it cli-workshop
```

You should now see the container's command prompt, something like:

```
root@4a4e3ef43aff:/#
```
...but with a different hex code.

Notice that the appearance of the terminal window is nearly the same, but now we're "inside" a terminal shell program running in a container.  This is a subtile but important different -- now we're inside the container, and before we were inside a terminal on your local machine.


Now try out some unix commands, by typing (or copying) this command and then typing the "return" or "enter" key.

```
ls
```
This will show a "list" of files (ls is called the "list" command).

Now let's make a new file from the command line, like this:
```
echo 'hello there!' > hi.txt
```

The command `echo 'hello there!` is a small program that prints "hello there!".  You can run, `echo 'hello there!` on it's own, and see that it prints out this text.

Next, the `>` part of the commend is a "redirect" command that writes the ouput of the `echo` program and "redirects" it to a file named `hi.txt`.

Viewing file permissions, owner, and file size from the command line. 

```
ls -alg hi.txt
```

This should show `-rw-r--r-- 1 root 7 Aug 11 21:33 hi.txt`, which shows the "permission bits" (`-rw-r--r--`), the owner (`root`), the size in kilobytes `7`, the date and time `Aug 11 21:33`

Let's look at contents of the file we just made, `hi.txt`

```
more hi.txt
```
should show:
```
hello there!
```

The `more` command renders the contents of the file to the terminal window and this works best for text files and poorly for binary files (like images or executables).  

Now, we're going to stop the container and see if the file we just created still exists.  

Since we're interacting with the container from _inside the container_, and the container is running a terminal program (the 'Shell'), we can exit the container by telling the shell (Terminal Program) inside the container that we're done.  To do this, we type into the terminal container:

```
exit
```

Try `ls` now, do you see `hi.txt`?   Why not?

Now run (Unix/MacOS/WSL only) 
```
docker run -v $(pwd)/scratch:/scratch  -it cli-workshop
```

Windows powershell users use `${PWD}` instead of `$(pwd)`.
```
docker run -v ${PWD}/scratch:/scratch  -it cli-workshop
```

The `-v $(pwd)...` (or `${PWD}`) part **creates a connection between you local machines and the environment inside the container, otherwise what happens in a container stays in the container during the containers lifetime**.

Now that we've created this connection and we're in the container's terminal program (a terminal in a terminal!), let's create a file inside this container.

```
echo 'hello again!' > scratch/hi.txt
```

```
exit
```

Now try doing a list in your terminal:

```
ls scratch
```

Do you see `hi.txt`?  (You should!, please ask for assistance if you do not)



## INSTALLING NEW SOFTWARE

Restart the docker container:

```
docker run -v $(pwd)/scratch:/scratch  -it cli-workshop
```

Update the list of downloadable software packages (for Ubuntu).  [`sudo` is a weird command for increasing the privilege of your commands -- use `sudo` carefully and only when necessary](https://en.wikipedia.org/wiki/Sudo).



```
sudo apt update
```

[`apt` is a package management tool for Ubuntu](https://ubuntu.com/server/docs/package-management), and is the usualy way to get pre-built binaries of software for the ubuntu (and ROS ecosystem).

Install the program `cloc`  (*Count Lines of Code*) within the container:
```
sudo apt install cloc
```

Download some robotic software from [ROS2 "Navigation 2" ("Nav-Two" or "Nav2")](https://github.com/ros-planning/navigation2.git):
```
git clone https://github.com/ros-planning/navigation2.git
```

Run `cloc`:
```
cloc navigation2
```

For your workshop deliverable, type your name in the terminal before taking a screen shot showing the output of `cloc` and your name.

Upload the screenshot and complete the questions here:
[Google Form file upload](https://docs.google.com/forms/d/e/1FAIpQLScAvwI2c99VhRM8TwnMnb8_J5miD7Piwa6N05_W-Y5ylyEeGg/viewform?usp=sf_link)


## That's it :)  Good job.

If you want to go further, open a container and type:

```
ros2
```

and start playing with the `ros2` commands built into this docker image.  You can type `ros2 help COMMAND` for more details.

If you're really feeling ambitious and are up for a possibly multi-day software installation adventure, try [installing ROS2 Iron on your machine using this install instructions].


## Evaluation

This workshop is worth 4 points.

You'll be graded on the following rubric:

| ITEM | POINTS |
|--|--|
| Screen shot | 1 |
| Questions   | 3 |
