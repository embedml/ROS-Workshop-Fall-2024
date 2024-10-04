
# DOCKER, COMMAND LINE TOOLS, AND GIT

## LEARNING GOALS

In this workshop, you will set up `docker`, `git`, learn some basic functionality of the command line as well as using `git` to track your changes.

This workshop has commands that differ slightly depending on your host operating system (i.e. Windows, MacOS, or Linux) and on which terminal program you're using (Windows WSL vs Windows Powershell).

_Windows Users_:  Recommended that you use either `Powershell` or `wsl` ([Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/install)) and **_not_** `CMD` the command prompt.

## Get the Docker container for this workshop

From following the setup instructions, you should have a local copy of the workshop code already downloaded in the directory where you ran the `git clone` command. Navigate to the same folder where the ROS-Workshop-Fall-2024 folder was placed.

```bash
cd ROS-Workshop-Fall-2024
cd part-1/module-1-docker-and-cli
#Then look at what is in this folder
ls
```

The first task is to build the Docker image from the Dockerfile in this folder.

Add execution permissions on `ros_entrypoint.sh` using the `chmod` command.

```bash
chmod +x ros_entrypoint.sh
```

Build the Docker image (this could take a few minutes):

```bash
docker build . -t cli-workshop
```

In the previous `docker` command, the `-t cli-workshop` creates a docker *image*  from the `Dockerfile` build file.  

Start the `docker` container in interactive mode (`-it` flags) :

```bash
docker run -it cli-workshop
```

You should now see the container's command prompt, something like:

```bash
root@4a4e3ef43aff:/#
```

...but with a different hex code.

Notice that the appearance of the terminal window is nearly the same, but now we're "inside" a terminal shell program running in a container.  This is a subtile but important different -- now we're inside the container, and before we were inside a terminal on your local machine.

Now try out some unix commands, by typing (or copying) this command and then typing the "return" or "enter" key.

```bash
ls
```

This will show a "list" of files (ls is called the "list" command).

Now let's make a new file from the command line, like this:

```bash
echo 'hello there!' > hi.txt
```

The command `echo 'hello there!` is a small program that prints "hello there!".  You can run, `echo 'hello there!` on it's own, and see that it prints out this text.

Next, the `>` part of the commend is a "redirect" command that writes the ouput of the `echo` program and "redirects" it to a file named `hi.txt`.

Viewing file permissions, owner, and file size from the command line. 

```bash
ls -alg hi.txt
```

This should show `-rw-r--r-- 1 root 7 Aug 11 21:33 hi.txt`, which shows the "permission bits" (`-rw-r--r--`), the owner (`root`), the size in kilobytes `7`, the date and time `Aug 11 21:33`

Let's look at contents of the file we just made, `hi.txt`

```bash
more hi.txt
```

should show:

```bash
hello there!
```

The `more` command renders the contents of the file to the terminal window and this works best for text files and poorly for binary files (like images or executables).  

Now, we're going to stop the container and see if the file we just created still exists.  

Since we're interacting with the container from _inside the container_, and the container is running a terminal program (the 'Shell'), we can exit the container by telling the shell (Terminal Program) inside the container that we're done.  To do this, we type into the terminal container:

```bash
exit
```

Try `ls` now, do you see `hi.txt`?   Why not?

Now run (Unix/MacOS/WSL only)

```bash
docker run -v $(pwd)/scratch:/scratch  -it cli-workshop
```

Windows powershell users use `${PWD}` instead of `$(pwd)`.

```ps
docker run -v ${PWD}/scratch:/scratch  -it cli-workshop
```

The `-v $(pwd)...` (or `${PWD}`) part **creates a connection between you local machines and the environment inside the container, otherwise what happens in a container stays in the container during the containers lifetime**.

Now that we've created this connection and we're in the container's terminal program (a terminal in a terminal!), let's create a file inside this container.

```bash
echo 'hello again!' > scratch/hi.txt
```

```bash
exit
```

Now try doing a list in your terminal:

```bash
ls scratch
```

Do you see `hi.txt`?  (You should!, please ask for assistance if you do not)

## INSTALLING NEW SOFTWARE

Restart the docker container:

```bash
docker run -v $(pwd)/scratch:/scratch  -it cli-workshop
```

Update the list of downloadable software packages (for Ubuntu).  [`sudo` is a weird command for increasing the privilege of your commands -- use `sudo` carefully and only when necessary](https://en.wikipedia.org/wiki/Sudo).

```bash
sudo apt update
```

[`apt` is a package management tool for Ubuntu](https://ubuntu.com/server/docs/package-management), and is the usualy way to get pre-built binaries of software for the ubuntu (and ROS ecosystem).

Install the program `cloc`  (*Count Lines of Code*) within the container:

```bash
sudo apt install cloc
```

Download some robotic software from [ROS2 "Navigation 2" ("Nav-Two" or "Nav2")](https://github.com/ros-planning/navigation2.git):

```bash
git clone https://github.com/ros-planning/navigation2.git
```

Run `cloc`:

```bash
cloc navigation2
```

## That's it :)  Good job.

If you want to go further, open a container and type:

```bash
ros2
```

and start playing with the `ros2` commands built into this docker image.  You can type `ros2 help COMMAND` for more details.

If you're really feeling ambitious, have a compatible machine, and are up for a possibly multi-day software installation adventure, try [installing ROS2 Humble on your machine using these install instructions](https://docs.ros.org/en/humble/Installation.html). This will likely exceed the time available in this workshop, but it is often useful to have a bare-metal installation for ROS development.
